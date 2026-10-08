#!/usr/bin/env python3
"""Local URDF tuning workbench with versioned parameter registry."""
import argparse
import json
from pathlib import Path
from urllib.parse import urlsplit

from robot_parameter_api import MAX_BYTES, ThreadingHTTPServer, make_handler as registry_handler
from robot_kinematics import build_model, pose, trial
from robot_runtime_client import RuntimeClient

ROOT = Path(__file__).resolve().parents[1]
ASSETS = {"/": ("web/robot-workbench/index.html", "text/html; charset=utf-8"),
          "/workbench.js": ("web/robot-workbench/workbench.js", "text/javascript; charset=utf-8"),
          "/robot-scene.js": ("web/robot-workbench/robot-scene.js", "text/javascript; charset=utf-8"),
          "/workbench.css": ("web/robot-workbench/workbench.css", "text/css; charset=utf-8"),
          "/runtime.js": ("web/robot-workbench/runtime.js", "text/javascript; charset=utf-8"),
          "/example.json": ("config/robot_profiles/workbench-demo.json", "application/json; charset=utf-8")}


def make_handler(store_dir, runtime_url=None):
    runtime = RuntimeClient(runtime_url) if runtime_url else None
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
                code, body = runtime.request(self.path) if runtime else (503, {'error': 'ROS 2 preview not configured; start workbench with --runtime-url http://127.0.0.1:8086'})
                return self.respond(code, body)
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
            if self.path not in ("/v1/robot-model", "/v1/robot-preview") and not runtime_route:
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
                if runtime_route:
                    code, body = runtime.request(self.path, payload) if runtime else (503, {'error': 'ROS 2 preview is not configured'})
                    return self.respond(code, body)
                if self.path == "/v1/robot-model":
                    if not isinstance(payload, dict) or set(payload) - {"urdf_xml", "positions"} or "urdf_xml" not in payload:
                        raise ValueError("expected urdf_xml and optional positions")
                    model = build_model(payload["urdf_xml"])
                    result = {"model": model, "pose": pose(model, payload.get("positions", {}))}
                else:
                    result = trial(payload)
                self.respond(200, result)
            except (ValueError, TypeError, OverflowError, RecursionError) as e:
                self.respond(400, {"error": str(e)})
            except OSError:
                self.respond(500, {"error": "request I/O failed"})

    return Handler


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--store", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8085)
    parser.add_argument("--runtime-url", help="Optional localhost ROS 2 joint-preview API")
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(args.store, args.runtime_url))
    print(f"OmindOS robot workbench: http://127.0.0.1:{server.server_port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
