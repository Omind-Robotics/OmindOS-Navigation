"""URDF forward kinematics and static gravity loads, in SI units.

No controller, contact solver, collision detection or hardware commands.
"""
import math
import xml.etree.ElementTree as ET

from robot_parameter_api import MAX_BYTES, number, validate_parameters


def vec(text, size=3):
    values = [number(x, "URDF vector") for x in text.split()]
    if len(values) != size:
        raise ValueError(f"expected {size} vector components")
    return values


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def mul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform(t, p, direction=False):
    return [sum(t[i][j]*p[j] for j in range(3)) + (0 if direction else t[i][3]) for i in range(3)]


def rotation(axis, q):
    norm = math.hypot(*axis)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError("invalid joint axis")
    x, y, z = [v / norm for v in axis]
    c, s, t = math.cos(q), math.sin(q), 1-math.cos(q)
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y, 0],
            [t*x*y+s*z, t*y*y+c, t*y*z-s*x, 0],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c, 0], [0, 0, 0, 1]]


def origin(node):
    o = node.find("origin")
    xyz = vec(o.get("xyz", "0 0 0")) if o is not None else [0, 0, 0]
    rpy = vec(o.get("rpy", "0 0 0")) if o is not None else [0, 0, 0]
    # URDF fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll).
    t = mul(mul(rotation([0, 0, 1], rpy[2]), rotation([0, 1, 0], rpy[1])), rotation([1, 0, 0], rpy[0]))
    for i in range(3):
        t[i][3] = xyz[i]
    return t


def build_model(xml):
    if not isinstance(xml, str) or len(xml.encode()) > MAX_BYTES:
        raise ValueError("URDF exceeds text size limit")
    if "<!DOCTYPE" in xml.upper() or "<!ENTITY" in xml.upper():
        raise ValueError("DTD and entities are unsupported")
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as e:
        raise ValueError("invalid URDF: " + str(e)) from None
    if root.tag != "robot":
        raise ValueError("expected robot root")
    links, joints, parents, warnings = {}, [], {}, []
    materials = {m.get('name'): m.find('color') for m in root.findall('material')}
    if len(root.findall("link")) > 128 or len(root.findall("joint")) > 127:
        raise ValueError("workbench supports at most 128 links and 127 joints")
    for link in root.findall("link"):
        name = link.get("name")
        if not name or name in links:
            raise ValueError("missing or duplicate link name")
        visuals = []
        if len(link.findall("visual")) > 16:
            raise ValueError("at most 16 visuals per link")
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            if geometry is None or len(geometry) != 1:
                raise ValueError(name + ": expected one visual geometry")
            g = geometry[0]
            item = {"type": g.tag, "transform": origin(visual)}
            material = visual.find('material')
            if material is not None:
                color = material.find('color')
                if color is None:
                    color = materials.get(material.get('name'))
                if color is not None:
                    item['color'] = vec(color.get('rgba', ''), 4)
                    if any(not 0 <= c <= 1 for c in item['color']):
                        raise ValueError('material rgba must be between 0 and 1')
            if g.tag == "box":
                item["size"] = vec(g.get("size", ""))
                if any(x <= 0 for x in item["size"]):
                    raise ValueError("box sizes must be positive")
            elif g.tag in ("cylinder", "sphere"):
                item["radius"] = number(g.get("radius"), "radius", positive=True)
                if g.tag == "cylinder":
                    item["length"] = number(g.get("length"), "length", positive=True)
            elif g.tag == "mesh":
                warnings.append(name + ": mesh 未加载，以关节骨架显示")
            else:
                raise ValueError("unsupported visual geometry: " + g.tag)
            visuals.append(item)
        inertial = link.find("inertial")
        mass = inertial.find("mass") if inertial is not None else None
        links[name] = {"name": name, "visuals": visuals,
                       "mass": number(mass.get("value"), "mass", positive=True) if mass is not None else None,
                       "com": transform(origin(inertial), [0, 0, 0]) if inertial is not None else [0, 0, 0]}
    names = set()
    for j in root.findall("joint"):
        name, kind = j.get("name"), j.get("type")
        if not name or name in names or kind not in ("fixed", "revolute", "continuous") or j.find("mimic") is not None:
            raise ValueError("use unique independent rotary or fixed joints")
        names.add(name)
        a, b = j.find("parent"), j.find("child")
        parent, child = a.get("link") if a is not None else None, b.get("link") if b is not None else None
        if parent not in links or child not in links or parent == child or child in parents:
            raise ValueError(name + ": invalid joint tree")
        parents[child] = parent
        axis_node, limit = j.find("axis"), j.find("limit")
        axis = vec(axis_node.get("xyz", "1 0 0")) if axis_node is not None else [1, 0, 0]
        rotation(axis, 0)
        lower = number(limit.get("lower"), "lower") if kind == "revolute" and limit is not None else -math.pi
        upper = number(limit.get("upper"), "upper") if kind == "revolute" and limit is not None else math.pi
        if kind == "revolute" and (limit is None or lower >= upper):
            raise ValueError(name + ": invalid rotary limits")
        joints.append({"name": name, "type": kind, "parent": parent, "child": child,
                       "origin": origin(j), "axis": axis, "lower": lower, "upper": upper})
    roots = set(links) - set(parents)
    if not links or len(roots) != 1:
        raise ValueError("expected exactly one root link")
    base = next(iter(roots))
    ordered, reached = [], {base}
    while len(ordered) < len(joints):
        batch = [j for j in joints if j["parent"] in reached and j["child"] not in reached]
        if not batch:
            raise ValueError("disconnected or cyclic URDF")
        ordered.extend(batch)
        reached.update(j["child"] for j in batch)
    return {"name": root.get("name", "robot"), "root": base, "links": list(links.values()),
            "joints": ordered, "warnings": list(dict.fromkeys(warnings)),
            "scope": "kinematics_and_static_gravity"}


def positions(model, values):
    active = {j["name"]: j for j in model["joints"] if j["type"] != "fixed"}
    if not isinstance(values, dict) or set(values) - set(active):
        raise ValueError("positions must name only active joints")
    q = {}
    for name, j in active.items():
        value = number(values.get(name, min(j["upper"], max(j["lower"], 0))), name)
        if j["type"] == "revolute" and not j["lower"] <= value <= j["upper"]:
            raise ValueError(name + ": position exceeds joint limits")
        if abs(value) > 1e6:
            raise ValueError(name + ": angle exceeds preview range")
        q[name] = value
    return q


def base_position(value=None):
    value = {} if value is None else value
    if not isinstance(value, dict) or set(value) - {'x_m', 'y_m', 'yaw_rad'}:
        raise ValueError('base_pose must contain only x_m, y_m and yaw_rad')
    result = {k: number(value.get(k, 0), 'base_pose.'+k) for k in ('x_m', 'y_m', 'yaw_rad')}
    if any(abs(result[k]) > 5 for k in ('x_m', 'y_m')) or abs(result['yaw_rad']) > 2*math.pi:
        raise ValueError('base_pose exceeds preview range: XY +/-5 m, yaw +/-2pi rad')
    return result


def pose(model, values, base_pose=None):
    q = positions(model, values)
    base = base_position(base_pose)
    root_frame = rotation([0, 0, 1], base['yaw_rad'])
    root_frame[0][3], root_frame[1][3] = base['x_m'], base['y_m']
    world, pivot, axes = {model["root"]: root_frame}, {}, {}
    ancestors = {model["root"]: []}
    for j in model["joints"]:
        frame = mul(world[j["parent"]], j["origin"])
        pivot[j["name"]] = transform(frame, [0, 0, 0])
        axes[j["name"]] = transform(frame, j["axis"], True)
        norm = math.hypot(*axes[j["name"]])
        axes[j["name"]] = [x / norm for x in axes[j["name"]]]
        world[j["child"]] = mul(frame, rotation(j["axis"], q.get(j["name"], 0)))
        ancestors[j["child"]] = ancestors[j["parent"]] + ([j["name"]] if j["type"] != "fixed" else [])
    torque = {name: 0.0 for name in q}
    coms = {}
    complete_mass = all(link["mass"] is not None for link in model["links"])
    for link in model["links"]:
        com = transform(world[link["name"]], link["com"])
        coms[link["name"]] = com
        if link["mass"] is None:
            continue
        for name in ancestors[link["name"]]:
            r = [com[i] - pivot[name][i] for i in range(3)]
            # Holding torque = -axis dot (r cross m*g), with world gravity -Z.
            force = -9.80665 * link["mass"]
            cross = [r[1]*force, -r[0]*force, 0]
            torque[name] -= sum(axes[name][i]*cross[i] for i in range(3))
    return {"positions": q, "base_pose": base, "link_transforms": world, "joint_positions": pivot, "centers_of_mass": coms,
            "holding_torque_nm": torque if complete_mass else None}


def trial(payload):
    if not isinstance(payload, dict) or set(payload) - {"profile", "start", "target", "duration_s", "base_start", "base_target"} or "profile" not in payload:
        raise ValueError("expected profile, start, target, duration_s and optional base_start/base_target")
    profile = payload["profile"]
    validate_parameters(profile)
    model = build_model(profile["kinematics"]["urdf_xml"])
    start, target = positions(model, payload.get("start", {})), positions(model, payload.get("target", {}))
    base_start, base_target = base_position(payload.get('base_start')), base_position(payload.get('base_target'))
    distance = math.hypot(base_target['x_m']-base_start['x_m'], base_target['y_m']-base_start['y_m'])
    turn = abs(base_target['yaw_rad']-base_start['yaw_rad'])
    drives = profile["dynamics"]["joints"]
    minimum = max([1.5*abs(target[n]-start[n])/drives[n]["velocity_limit_rad_s"] for n in start] or [0])
    # Preview pacing only: not specifications or wheel-space limits of a real chassis.
    minimum = max(minimum, 1.5*distance/.5, 1.5*turn/.8)
    duration = number(payload.get("duration_s", max(2, minimum)), "duration_s", positive=True)
    if duration > 60 or duration + 1e-10 < minimum:
        raise ValueError(f"duration_s must be between {minimum:.3f} and 60 seconds to respect velocity limits")
    count = min(181, max(2, math.ceil(duration*30)+1))
    frames, peaks = [], {n: 0 for n in start}
    for i in range(count):
        u = i / (count-1)
        smooth = u*u*(3-2*u)
        q = {n: start[n] + (target[n]-start[n])*smooth for n in start}
        if i == 0:
            q = start.copy()
        elif i == count-1:
            q = target.copy()
        base = {k: base_start[k] + (base_target[k]-base_start[k])*smooth for k in base_start}
        if i == 0:
            base = base_start.copy()
        elif i == count-1:
            base = base_target.copy()
        frame = pose(model, q, base)
        frame["time_s"] = u*duration
        frame["motor_positions_rad"] = {
            n: (q[n]-drives[n]["zero_offset_rad"])*drives[n]["reduction_ratio"]/drives[n]["direction"] for n in q}
        for n, torque in frame["holding_torque_nm"].items():
            peaks[n] = max(peaks[n], abs(torque))
        frames.append(frame)
    exceeded = [n for n in peaks if peaks[n] > drives[n]["torque_limit_nm"]]
    return {"model": model, "frames": frames, "duration_s": duration, "minimum_duration_s": minimum,
            "peak_holding_torque_nm": peaks, "gravity_limit_exceeded": exceeded,
            "peak_joint_velocity_rad_s": {n: 1.5*abs(target[n]-start[n])/duration for n in start},
            "peak_base_velocity": {'linear_m_s': 1.5*distance/duration, 'angular_rad_s': 1.5*turn/duration},
            "scope": 'prescribed_planar_base_kinematics_and_static_gravity' if any(base_start.values()) or any(base_target.values()) else 'fixed_base_kinematics_and_static_gravity',
            "runtime_applied": False, "hardware_validated": False,
            "limitations": ["底盘为给定平面位姿预览，未计算轮速、轮地接触、打滑或导航。", "没有碰撞、平衡或惯性动力学积分；静态重力不包含加速度、摩擦或外部负载。"]}
