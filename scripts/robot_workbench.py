#!/usr/bin/env python3
"""Local URDF tuning workbench with versioned parameter registry."""
import argparse
import json
from pathlib import Path
from urllib.parse import urlsplit

from robot_parameter_api import MAX_BYTES, ThreadingHTTPServer, make_handler as registry_handler
from robot_kinematics import build_model, pose, trial, jog_trial, sine_trial, wheel_trial
from robot_assets import MESH_ASSETS
from robot_sequence import sequence_trial
from workbench_state import (WorkbenchState, MotionLocked, _register_workbench_profile, _load_workbench_profile)
from robot_kinematic_preview import motion as kinematic_motion
from robot_model_bundle import register_bundle, inventory as model_inventory, model as bundled_model
from robot_connections import connection_catalog, validate_connections
from robot_runtime_client import RuntimeClient
from robot_planning import PlanningJobs, default_mapping
from robot_navigation import NavigationJobs, default_config as navigation_config

ROOT = Path(__file__).resolve().parents[1]
ASSETS = {"/cad/v11/kinematics.json": ("config/robot_models/originmind_quadruped_v11/kinematics.json", "application/json; charset=utf-8"),
          "/cad-panel.js": ("web/robot-workbench/cad-panel.js", "text/javascript; charset=utf-8"),
          "/cad-viewer.js": ("web/robot-workbench/cad-viewer.js", "text/javascript; charset=utf-8"),
          "/cad/v11/manifest.json": ("config/robot_models/originmind_quadruped_v11/manifest.json", "application/json; charset=utf-8"),
          "/cad/v11/assembly.glb": ("config/robot_models/originmind_quadruped_v11/assembly.glb", "model/gltf-binary"),
          "/": ("web/robot-workbench/index.html", "text/html; charset=utf-8"),
          "/rm75-workbench.js": ("web/robot-workbench/rm75-workbench.js", "text/javascript; charset=utf-8"),
          "/workbench.js": ("web/robot-workbench/workbench.js", "text/javascript; charset=utf-8"),
          "/robot-scene.js": ("web/robot-workbench/robot-scene.js", "text/javascript; charset=utf-8"),
          "/runtime.js": ("web/robot-workbench/runtime.js", "text/javascript; charset=utf-8"),
          "/workbench.css": ("web/robot-workbench/workbench.css", "text/css; charset=utf-8"),
          "/cooperative-demo.json": ("web/robot-workbench/cooperative-demo.json", "application/json; charset=utf-8"),
          "/mda1-reference.webp": ("web/robot-workbench/mda1-reference.webp", "image/webp"),
          "/example.json": ("config/robot_profiles/workbench-demo.json", "application/json; charset=utf-8")}


def make_handler(store_dir, runtime_url=None):
    runtime = WorkbenchState(store_dir)
    planning = PlanningJobs(runtime)
    navigation = NavigationJobs(runtime, planning)
    ros_runtime = RuntimeClient(runtime_url) if runtime_url else None
    ros_session = {'lease_token': None}
    class Handler(registry_handler(store_dir)):
        def end_headers(self):
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
            super().end_headers()

        def permitted(self):
            allowed = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
            host = self.headers.get("Host", "")
            origin = self.headers.get("Origin")
            if host not in allowed or (origin and origin != "http://" + host):
                self.respond(403, {"error": "local same-origin requests only"})
                return False
            return True

        def do_GET(self):
            if not self.permitted():
                return
            if self.path == '/v1/runtime/status':
                code, body = ros_runtime.request(self.path) if ros_runtime else (503, {'error': 'ROS 2 preview not configured; use --runtime-url http://127.0.0.1:8086'})
                return self.respond(code, body)
            if self.path == '/v1/workbench-models':
                return self.respond(200, model_inventory())
            if self.path.startswith('/v1/workbench-model/'):
                try:
                    return self.respond(200, bundled_model(self.path[len('/v1/workbench-model/'):]))
                except ValueError as e:
                    return self.respond(404, {'error': str(e)})
            if self.path.startswith('/v1/workbench-profiles/'):
                try:
                    return self.respond(200, _load_workbench_profile(self.path[len('/v1/workbench-profiles/'):], store_dir))
                except FileNotFoundError:
                    return self.respond(404, {'error': 'profile not found'})
                except ValueError as e:
                    return self.respond(400, {'error': str(e)})
                except OSError:
                    return self.respond(500, {'error': 'profile storage unavailable'})
            if self.path == '/v1/workbench-state':
                return self.respond(200, runtime.snapshot())
            if self.path == '/v1/navigation-config':
                return self.respond(200, {'config':navigation_config(), 'drive_types':['differential','omnidirectional'], 'simulation_only':True})
            if self.path.startswith('/v1/navigation-heartbeat/'):
                try:
                    return self.respond(200, navigation.heartbeat(self.path.rsplit('/',1)[-1]))
                except ValueError as e:
                    return self.respond(404, {'error':str(e)})
            if self.path.startswith('/v1/navigation-task/'):
                try:
                    return self.respond(200, navigation.snapshot(self.path.rsplit('/',1)[-1]))
                except ValueError as e:
                    return self.respond(404, {'error':str(e)})
            if self.path == '/v1/planning-config':
                return self.respond(200, {'mapping':default_mapping(), 'kernel':'runtime.2',
                                         'collision_geometry':'conservative_oriented_boxes'})
            if self.path.startswith('/v1/planning-task/'):
                try:
                    return self.respond(200, planning.snapshot(self.path.rsplit('/',1)[-1]))
                except ValueError as e:
                    return self.respond(404, {'error':str(e)})
            if self.path == '/v1/connection-options':
                return self.respond(200, connection_catalog())
            if self.path == '/v1/workbench-active-profile':
                try:
                    with runtime.lock:
                        return self.respond(200, {**runtime.snapshot(), 'profile': runtime.active_profile()})
                except (ValueError, OSError):
                    return self.respond(500, {'error': 'active workbench configuration unavailable'})
            mesh = MESH_ASSETS.get(urlsplit(self.path).path)
            if mesh:
                try:
                    data = (ROOT / mesh).read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except OSError:
                    self.respond(404, {"error": "approved mesh unavailable"})
                return
            asset = ASSETS.get(urlsplit(self.path).path)
            if asset:
                try:
                    data = (ROOT / asset[0]).read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", asset[1])
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except OSError:
                    self.respond(500, {"error": "workbench assets missing"})
                return
            super().do_GET()

        def do_POST(self):
            if not self.permitted():
                return
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                return self.respond(415, {"error": "Content-Type must be application/json"})
            runtime_route = self.path in ['/v1/runtime/' + op for op in ('load', 'run', 'stop', 'heartbeat')]
            if self.path not in ("/v1/workbench-profiles", "/v1/robot-model", "/v1/robot-preview", '/v1/robot-jog', '/v1/robot-sine', '/v1/robot-wheel-base', '/v1/robot-sequence',
                                 '/v1/workbench-active-profile', '/v1/workbench-stop', '/v1/planning-start', '/v1/planning-cancel', '/v1/navigation-start', '/v1/navigation-cancel') and not runtime_route:
                return super().do_POST()
            try:
                self.connection.settimeout(10)
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= MAX_BYTES or self.headers.get("Transfer-Encoding"):
                    return self.respond(413, {"error": "invalid request size"})
                raw = self.rfile.read(size)
                if len(raw) != size:
                    raise ValueError("incomplete request")
                payload = json.loads(raw)
                if self.path == '/v1/workbench-profiles':
                    return self.respond(201, _register_workbench_profile(payload, store_dir))
                if runtime_route:
                    if self.path == '/v1/runtime/run': navigation.ensure_idle()
                    with runtime.lock:
                        if self.path == '/v1/runtime/run':
                            runtime.check_motion()
                        code, body = ros_runtime.request(self.path, payload) if ros_runtime else (503, {'error': 'ROS 2 preview is not configured'})
                        if code == 200 and self.path == '/v1/runtime/load':
                            ros_session['lease_token'] = body.get('lease_token')
                    return self.respond(code, body)
                if self.path == '/v1/navigation-start':
                    result = navigation.submit(payload)
                elif self.path == '/v1/navigation-cancel':
                    if not isinstance(payload,dict) or set(payload) != {'task_id'}:
                        raise ValueError('expected task_id')
                    result = navigation.cancel(payload['task_id'])
                elif self.path == '/v1/planning-start':
                    with navigation.lock:
                        navigation.ensure_idle()
                        result = planning.submit(payload)
                elif self.path == '/v1/planning-cancel':
                    if not isinstance(payload,dict) or set(payload) != {'task_id'}:
                        raise ValueError('expected task_id')
                    result = planning.cancel(payload['task_id'])
                elif self.path == '/v1/workbench-active-profile':
                    if navigation.active(): navigation.cancel()
                    result = runtime.apply(payload)
                elif self.path == '/v1/workbench-stop':
                    if isinstance(payload,dict) and payload.get('engaged') is True and navigation.active(): navigation.cancel()
                    with runtime.lock:
                        result = runtime.set_stop(payload)
                        if payload['engaged'] and ros_runtime and ros_session['lease_token']:
                            code, body = ros_runtime.request('/v1/runtime/stop', ros_session)
                            result['ros_preview_stop_confirmed'] = code == 200
                            if code != 200:
                                result['ros_preview_stop_error'] = body.get('error', 'stop not confirmed')
                elif self.path == "/v1/robot-model":
                    if not isinstance(payload, dict) or set(payload) - {"urdf_xml", "positions", "base_pose", "connections"} or "urdf_xml" not in payload:
                        raise ValueError("expected urdf_xml and optional positions/base_pose")
                    if 'connections' in payload:
                        validate_connections(payload['connections'])
                    model = build_model(payload["urdf_xml"])
                    result = {"model": model, "pose": pose(model, payload.get("positions", {}), payload.get('base_pose'))}
                else:
                    navigation.ensure_idle()
                    epoch = runtime.check_motion()
                    if isinstance(payload, dict) and 'profile' not in payload:
                        profile = runtime.active_profile()
                        if profile is None:
                            raise ValueError('请先将参数写入程序，或提供完整 profile。')
                        payload = {**payload, 'profile': profile}
                    function = {'/v1/robot-preview': trial, '/v1/robot-jog': jog_trial, '/v1/robot-sine': sine_trial, '/v1/robot-wheel-base': wheel_trial, '/v1/robot-sequence': sequence_trial}[self.path]
                    result = (kinematic_motion(self.path, payload) if isinstance(payload, dict) and isinstance(payload.get('profile'), dict) and payload['profile'].get('profile_mode') == 'kinematic_preview' else function(payload))
                    runtime.check_motion(epoch)
                    result['motion_epoch'] = epoch
                self.respond(200, result)
            except MotionLocked as e:
                self.respond(423, {'error': str(e), **runtime.snapshot()})
            except (ValueError, TypeError, OverflowError, RecursionError) as e:
                self.respond(400, {"error": str(e)})
            except OSError:
                self.respond(500, {"error": "request I/O failed"})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--store", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8085)
    parser.add_argument("--runtime-url", help="Optional localhost ROS 2 joint-preview API")
    parser.add_argument("--model-bundle", type=Path, help="Explicit local RM75 model resource directory")
    args = parser.parse_args()
    bundle = args.model_bundle or ROOT / "config/robot_models/rm75-cad"
    if args.model_bundle is not None or bundle.is_dir():
        register_bundle(bundle)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(args.store, args.runtime_url))
    print(f"OmindOS robot workbench: http://127.0.0.1:{server.server_port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

