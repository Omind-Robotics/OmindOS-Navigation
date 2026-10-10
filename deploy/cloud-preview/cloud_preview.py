#!/usr/bin/env python3
"""Password-protected, single-operator OmindOS preview. No public signup."""
import base64
import hashlib
import hmac
import json
import os
from pathlib import Path
import sys
from urllib.parse import urlsplit

SOURCE = Path(os.environ["OMINDOS_SOURCE"])
sys.path.insert(0, str(SOURCE / "scripts"))
import robot_workbench
from robot_model_bundle import register_bundle

PREFIX = "/_cloud-preview/"
ORIGIN = "https://omindos.cn"
PORT = int(os.environ.get("OMINDOS_PREVIEW_PORT", "18885"))
STORE = Path(os.environ.get("OMINDOS_PREVIEW_STORE", "/data"))
SECRET = json.loads(Path(os.environ["OMINDOS_PREVIEW_AUTH_FILE"]).read_text())
APP = Path(__file__).resolve().parent
robot_workbench.ROOT = SOURCE
register_bundle(Path(os.environ["OMINDOS_MODEL_BUNDLE"]))
Base = robot_workbench.make_handler(STORE)

class Handler(Base):
    def permitted(self):
        if self.headers.get("Host") not in {f"127.0.0.1:{PORT}", f"localhost:{PORT}"}:
            self.respond(403, {"error": "invalid preview host"})
            return False
        try:
            kind, value = self.headers.get("Authorization", "").split(" ", 1)
            username, password = base64.b64decode(value, validate=True).decode("utf-8").split(":", 1)
            accepted = (kind.lower() == "basic"
                        and hmac.compare_digest(username, SECRET["username"])
                        and hmac.compare_digest(hashlib.sha256(password.encode()).hexdigest(), SECRET["sha256"]))
        except (ValueError, UnicodeError, TypeError):
            accepted = False
        if not accepted:
            self.send_response(401)
            self.send_header("WWW-Authenticate", 'Basic realm="OmindOS private preview", charset="UTF-8"')
            self.send_header("Content-Length", "0")
            self.end_headers()
            return False
        origin = self.headers.get("Origin")
        if origin is not None and origin != ORIGIN:
            self.respond(403, {"error": "cross-origin preview request rejected"})
            return False
        return True

    def end_headers(self):
        self.send_header("X-Robots-Tag", "noindex, nofollow, noarchive")
        self.send_header("Referrer-Policy", "same-origin")
        super().end_headers()

    def do_GET(self):
        if not self.permitted():
            return
        route = urlsplit(self.path).path
        if route == "/preview-bootstrap.js":
            data = (APP / "preview-bootstrap.js").read_bytes()
            mime = "text/javascript; charset=utf-8"
        elif route in robot_workbench.ASSETS and route.endswith(("/", ".js", ".css")):
            relative, mime = robot_workbench.ASSETS[route]
            try:
                content = (SOURCE / relative).read_text()
            except OSError:
                return self.respond(404, {"error": "preview asset unavailable"})
            if route == "/":
                content = content.replace('<title>OmindOS · 机器人调参工作台</title>',
                                          '<title>OmindOS 云端体验 · 单人预览</title>')
                content = content.replace('src="/', 'src="' + PREFIX).replace('href="/', 'href="' + PREFIX)
                content = content.replace("</head>", '<script src="' + PREFIX + 'preview-bootstrap.js"></script></head>')
                content = content.replace("<body>", '<body><aside class="cloud-preview-note">OmindOS 云端体验 · 单人预览｜计算在腾讯云运行，当前为几何仿真。修改内容保存在此预览空间。</aside>')
            if route.endswith(".js"):
                content = content.replace("import('/", "import('" + PREFIX).replace('import("/', 'import("' + PREFIX)
            if route.endswith(".css"):
                content += "\n.cloud-preview-note{padding:10px 18px;background:#102c40;color:#e4f3ff;font:500 14px/1.6 system-ui,sans-serif;border-bottom:1px solid #28627d;}\n"
            if route == "/workbench.js":
                anchor = "async function runNavigation(thenArm=false){\n"
                if content.count(anchor) != 1:
                    return self.respond(503, {"error": "preview source version mismatch"})
                content = content.replace(anchor, (APP / "navigation-ready.js").read_text(), 1)
            for old, new in [
                ("本机几何仿真", "云端几何仿真"),
                ("本机整机资源", "云端整机资源"),
                ("本机已安装整机资源", "云端已准备整机资源"),
                ("参数保存在本机服务目录", "参数保存在云端预览空间"),
            ]:
                content = content.replace(old, new)
            data = content.encode("utf-8")
        else:
            return super().do_GET()
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

server = robot_workbench.ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
print("OmindOS private preview listening on loopback", flush=True)
try:
    server.serve_forever()
finally:
    server.server_close()
