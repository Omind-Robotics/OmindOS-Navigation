#!/usr/bin/env python3
"""Local robot parameter registry. Stores versioned data; never enables hardware."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

MAX_BYTES = 2 * 1024 * 1024
ROBOT_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
PROFILE_ID = re.compile(r"[0-9a-f]{64}\Z")
DRIVE_FIELDS = {"motor_model", "reduction_ratio", "direction", "zero_offset_rad",
                "torque_limit_nm", "velocity_limit_rad_s"}


def number(value, label, positive=False, nonnegative=False):
    if isinstance(value, bool):
        raise ValueError(label + " must be a finite number")
    try:
        parsed = float(value)
    except (ValueError, TypeError, OverflowError):
        raise ValueError(label + " must be a finite number") from None
    if not math.isfinite(parsed) or (positive and parsed <= 0) or (nonnegative and parsed < 0):
        raise ValueError(label + " is outside its valid range")
    return parsed


def validate_parameters(payload):
    """Check a tree URDF and SI-unit rotary joint drive declarations.

    Link inertias belong in the URDF. This is a configuration check, not a
    physics simulation, controller tuning result, or driver compatibility test.
    """
    if not isinstance(payload, dict) or set(payload) != {"robot_id", "kinematics", "dynamics"}:
        raise ValueError("expected robot_id, kinematics and dynamics")
    rid = payload["robot_id"]
    if not isinstance(rid, str) or not ROBOT_ID.fullmatch(rid):
        raise ValueError("invalid robot_id")
    kin, dyn = payload["kinematics"], payload["dynamics"]
    if not isinstance(kin, dict) or set(kin) != {"urdf_xml"}:
        raise ValueError("kinematics must contain urdf_xml")
    if not isinstance(dyn, dict) or set(dyn) != {"joints"} or not isinstance(dyn["joints"], dict):
        raise ValueError("dynamics must contain a joints object")
    xml = kin["urdf_xml"]
    if not isinstance(xml, str) or len(xml.encode("utf-8")) > MAX_BYTES:
        raise ValueError("URDF must be text within the size limit")
    if "<!DOCTYPE" in xml.upper() or "<!ENTITY" in xml.upper():
        raise ValueError("DTD and entity declarations are not supported")
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as error:
        raise ValueError("invalid URDF XML: " + str(error)) from None
    if root.tag != "robot":
        raise ValueError("URDF root must be robot")
    links, joints = {}, {}
    for tag, target in (("link", links), ("joint", joints)):
        for node in root.findall(tag):
            name = node.get("name")
            if not name or name in target:
                raise ValueError("missing or duplicate " + tag + " name")
            target[name] = node
    if not links or not joints:
        raise ValueError("URDF must contain links and joints")
    parent_of, active = {}, {}
    for name, joint in joints.items():
        kind = joint.get("type")
        if kind not in {"fixed", "revolute", "continuous"} or joint.find("mimic") is not None:
            raise ValueError(name + ": this API supports independent rotary and fixed joints")
        parent, child = joint.find("parent"), joint.find("child")
        a = parent.get("link") if parent is not None else None
        b = child.get("link") if child is not None else None
        if a not in links or b not in links or a == b or b in parent_of:
            raise ValueError(name + ": invalid parent/child link relationship")
        parent_of[b] = a
        if kind != "fixed":
            active[name] = joint
            axis = joint.find("axis")
            axis_values = (axis.get("xyz", "1 0 0") if axis is not None else "1 0 0").split()
            if len(axis_values) != 3 or sum(number(x, name + ".axis")**2 for x in axis_values) < 1e-12:
                raise ValueError(name + ": invalid rotation axis")
    if len(set(links) - set(parent_of)) != 1:
        raise ValueError("URDF must have exactly one root link")
    for name in links:
        seen, cursor = set(), name
        while cursor in parent_of:
            if cursor in seen:
                raise ValueError("URDF link graph contains a cycle")
            seen.add(cursor)
            cursor = parent_of[cursor]
    if not active or set(dyn["joints"]) != set(active):
        raise ValueError("dynamics.joints must match every active URDF joint by name")
    # Require declared physical properties on all links in this first API.
    # Frames without physical bodies should be represented by TF, not dummy links.
    for name, link in links.items():
        inertial = link.find("inertial")
        mass = inertial.find("mass") if inertial is not None else None
        inertia = inertial.find("inertia") if inertial is not None else None
        if mass is None or inertia is None:
            raise ValueError(name + ": inertial mass and tensor are required")
        number(mass.get("value"), name + ".mass", positive=True)
        xx, yy, zz = [number(inertia.get(k), name + "." + k, positive=True)
                      for k in ("ixx", "iyy", "izz")]
        xy, xz, yz = [number(inertia.get(k), name + "." + k) for k in ("ixy", "ixz", "iyz")]
        determinant = xx*yy*zz + 2*xy*xz*yz - xx*yz*yz - yy*xz*xz - zz*xy*xy
        if xx*yy - xy*xy <= 0 or determinant <= 0:
            raise ValueError(name + ": inertia tensor must be positive definite")
    for name, joint in active.items():
        drive = dyn["joints"][name]
        if not isinstance(drive, dict) or set(drive) != DRIVE_FIELDS:
            raise ValueError(name + ": expected drive fields " + ", ".join(sorted(DRIVE_FIELDS)))
        if not isinstance(drive["motor_model"], str) or not drive["motor_model"].strip():
            raise ValueError(name + ": motor_model is required")
        if type(drive["direction"]) is not int or drive["direction"] not in (-1, 1):
            raise ValueError(name + ": direction must be 1 or -1")
        for key in ("reduction_ratio", "torque_limit_nm", "velocity_limit_rad_s"):
            if type(drive[key]) not in (float, int):
                raise ValueError(name + "." + key + " must be a JSON number")
            number(drive[key], name + "." + key, positive=True)
        if type(drive["zero_offset_rad"]) not in (float, int):
            raise ValueError(name + ".zero_offset_rad must be a JSON number")
        number(drive["zero_offset_rad"], name + ".zero_offset_rad")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(name + ": URDF effort and velocity limits are required")
        effort = number(limit.get("effort"), name + ".effort", positive=True)
        velocity = number(limit.get("velocity"), name + ".velocity", positive=True)
        if drive["torque_limit_nm"] > effort or drive["velocity_limit_rad_s"] > velocity:
            raise ValueError(name + ": drive limits exceed URDF joint limits")
        if joint.get("type") == "revolute":
            lower = number(limit.get("lower"), name + ".lower")
            upper = number(limit.get("upper"), name + ".upper")
            if lower >= upper:
                raise ValueError(name + ": lower joint limit must be below upper")
        dynamics = joint.find("dynamics")
        if dynamics is not None:
            for key in ("damping", "friction"):
                number(dynamics.get(key, "0"), name + "." + key, nonnegative=True)
    for origin in root.iter("origin"):
        for attribute in ("xyz", "rpy"):
            values = origin.get(attribute, "0 0 0").split()
            if len(values) != 3:
                raise ValueError("origin " + attribute + " must contain 3 values")
            for value in values:
                number(value, "origin " + attribute)
    return {"robot_id": rid, "joint_names": list(active),
            "validation_scope": "parameter_structure_and_basic_consistency",
            "runtime_applied": False, "hardware_validated": False}


def register_robot_parameters(payload, store_dir):
    metadata = validate_parameters(payload)
    canonical = json.dumps(payload, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    profile_id = hashlib.sha256(canonical.encode()).hexdigest()
    folder = Path(store_dir)
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / (profile_id + ".json")
    # Exclusive creation makes identical submissions idempotent without overwriting a version.
    try:
        with target.open("x", encoding="utf-8") as handle:
            handle.write(canonical + "\n")
    except FileExistsError:
        if target.read_text(encoding="utf-8").strip() != canonical:
            raise ValueError("stored profile content mismatch")
    return {"profile_id": profile_id, **metadata}


def load_robot_profile(profile_id, store_dir):
    if not isinstance(profile_id, str) or not PROFILE_ID.fullmatch(profile_id):
        raise ValueError("invalid profile_id")
    payload = json.loads((Path(store_dir) / (profile_id + ".json")).read_text(encoding="utf-8"))
    validate_parameters(payload)
    canonical = json.dumps(payload, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    if hashlib.sha256(canonical.encode()).hexdigest() != profile_id:
        raise ValueError("stored profile checksum mismatch")
    return payload


def make_handler(store_dir):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, status, body):
            data = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):
            self.connection.settimeout(10)
            if self.path != "/v1/robot-profiles":
                return self.respond(404, {"error": "route not found"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > MAX_BYTES or self.headers.get("Transfer-Encoding"):
                    return self.respond(413, {"error": "invalid request size or transfer encoding"})
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError("incomplete request")
                payload = json.loads(raw)
                result = register_robot_parameters(payload, store_dir)
                self.respond(201, result)
            except (ValueError, TypeError, OverflowError, RecursionError) as error:
                self.respond(400, {"error": str(error)})
            except OSError:
                self.respond(500, {"error": "profile storage or request I/O failed"})

        def do_GET(self):
            prefix = "/v1/robot-profiles/"
            if not self.path.startswith(prefix):
                return self.respond(404, {"error": "route not found"})
            try:
                result = load_robot_profile(self.path[len(prefix):], store_dir)
                self.respond(200, result)
            except FileNotFoundError:
                self.respond(404, {"error": "profile not found"})
            except ValueError as error:
                self.respond(400, {"error": str(error)})
            except OSError:
                self.respond(500, {"error": "profile storage unavailable"})

        def log_message(self, *_args):
            pass
    return Handler


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--store", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8085)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(args.store))
    print(f"Parameter API: http://127.0.0.1:{server.server_port}/v1/robot-profiles", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
