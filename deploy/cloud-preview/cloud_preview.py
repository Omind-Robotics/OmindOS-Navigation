#!/usr/bin/env python3
"""Password-protected, single-operator OmindOS preview. No public signup."""
import base64
import hashlib
import hmac
from http.cookies import SimpleCookie, CookieError
import json
import os
from pathlib import Path
import secrets
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit

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
SESSION_COOKIE = "__Secure-omindos-preview"
CSRF_COOKIE = "__Secure-omindos-preview-login"
SESSION_SECONDS = 2 * 60 * 60
SESSIONS = {}
FAILED_LOGINS = {}
AUTH_LOCK = threading.Lock()

class Handler(Base):
    def send_bytes(self, status, data, mime, headers=()):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        for name, value in headers:
            self.send_header(name, value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(data)

    def respond(self, status, body):
        self.send_bytes(status, json.dumps(body, ensure_ascii=False).encode(),
                        "application/json; charset=utf-8")

    def valid_origin(self):
        if self.headers.get("Host") not in {f"127.0.0.1:{PORT}", f"localhost:{PORT}"}:
            self.respond(403, {"error": "invalid preview host"})
            return False
        origin = self.headers.get("Origin")
        if origin is not None and origin != ORIGIN:
            self.respond(403, {"error": "cross-origin preview request rejected"})
            return False
        return True

    def cookies(self):
        cookies = SimpleCookie()
        try:
            cookies.load(self.headers.get("Cookie", ""))
        except CookieError:
            pass
        return cookies

    def authenticated(self):
        cookie = self.cookies().get(SESSION_COOKIE)
        if cookie:
            with AUTH_LOCK:
                expiry = SESSIONS.get(cookie.value, 0)
                if expiry > time.monotonic():
                    return True
                SESSIONS.pop(cookie.value, None)
        # Preserve existing command-line clients without triggering a browser popup.
        try:
            kind, value = self.headers.get("Authorization", "").split(" ", 1)
            username, password = base64.b64decode(value, validate=True).decode("utf-8").split(":", 1)
            return (kind.lower() == "basic" and self.credentials_match(username, password))
        except (ValueError, UnicodeError, TypeError):
            return False

    @staticmethod
    def credentials_match(username, password):
        return (hmac.compare_digest(username.encode(), SECRET["username"].encode())
                and hmac.compare_digest(hashlib.sha256(password.encode()).hexdigest(), SECRET["sha256"]))

    def permitted(self):
        if not self.valid_origin():
            return False
        if not self.authenticated():
            self.respond(401, {"error": "请先登录云端预览。", "login_url": PREFIX})
            return False
        return True

    def login_page(self, status=200, message=""):
        nonce = secrets.token_urlsafe(32)
        content = (APP / "preview-login.html").read_text().replace("{{csrf}}", nonce)
        content = content.replace("{{message}}", message)
        self.send_bytes(status, content.encode(), "text/html; charset=utf-8", (
            ("Set-Cookie", f"{CSRF_COOKIE}={nonce}; Path={PREFIX}; Max-Age=600; Secure; HttpOnly; SameSite=Strict"),
        ))

    def do_POST(self):
        route = urlsplit(self.path).path
        if route not in ("/login", "/logout"):
            return super().do_POST()
        if not self.valid_origin():
            return
        if self.headers.get("Origin") != ORIGIN:
            return self.respond(403, {"error": "same-origin form submission required"})
        if route == "/logout":
            cookie = self.cookies().get(SESSION_COOKIE)
            if cookie:
                with AUTH_LOCK:
                    SESSIONS.pop(cookie.value, None)
            return self.send_bytes(303, b"", "text/plain", (
                ("Location", PREFIX),
                ("Set-Cookie", f"{SESSION_COOKIE}=; Path={PREFIX}; Max-Age=0; Secure; HttpOnly; SameSite=Strict"),
            ))
        try:
            self.connection.settimeout(10)
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 4096 or self.headers.get("Transfer-Encoding"):
                return self.respond(413, {"error": "invalid login request size"})
            if self.headers.get("Content-Type", "").split(";")[0] != "application/x-www-form-urlencoded":
                return self.respond(415, {"error": "expected a login form"})
            raw = self.rfile.read(size)
            if len(raw) != size:
                raise ValueError("incomplete login form")
            form = parse_qs(raw.decode("utf-8"), keep_blank_values=True, max_num_fields=3)
            if set(form) != {"username", "password", "csrf"} or any(len(v) != 1 for v in form.values()):
                raise ValueError("invalid login form")
            username, password, nonce = (form[k][0] for k in ("username", "password", "csrf"))
            if len(username) > 128 or len(password) > 256 or len(nonce) > 128:
                raise ValueError("login field too long")
        except (ValueError, UnicodeError, OSError):
            return self.respond(400, {"error": "invalid login form"})
        csrf = self.cookies().get(CSRF_COOKIE)
        if not csrf or not hmac.compare_digest(nonce.encode(), csrf.value.encode()):
            return self.login_page(403, "登录页面已过期，请重新输入账号和密码。")
        address = self.headers.get("X-Real-IP", self.client_address[0])
        now = time.monotonic()
        with AUTH_LOCK:
            recent = [t for t in FAILED_LOGINS.get(address, []) if now - t < 60]
            if len(recent) >= 5:
                return self.login_page(429, "尝试次数过多，请一分钟后再试。")
        if not self.credentials_match(username, password):
            with AUTH_LOCK:
                if len(FAILED_LOGINS) >= 1024:
                    FAILED_LOGINS.clear()
                FAILED_LOGINS[address] = recent + [now]
            return self.login_page(200, "账号或密码不正确，请复制上一条消息中的账号和密码。")
        token = secrets.token_urlsafe(32)
        with AUTH_LOCK:
            FAILED_LOGINS.pop(address, None)
            for expired in [key for key, expiry in SESSIONS.items() if expiry <= now]:
                SESSIONS.pop(expired, None)
            if len(SESSIONS) >= 128:
                SESSIONS.pop(next(iter(SESSIONS)))
            SESSIONS[token] = now + SESSION_SECONDS
        self.send_bytes(303, b"", "text/plain", (
            ("Location", PREFIX),
            ("Set-Cookie", f"{SESSION_COOKIE}={token}; Path={PREFIX}; Max-Age={SESSION_SECONDS}; Secure; HttpOnly; SameSite=Strict"),
            ("Set-Cookie", f"{CSRF_COOKIE}=; Path={PREFIX}; Max-Age=0; Secure; HttpOnly; SameSite=Strict"),
        ))

    def end_headers(self):
        self.send_header("X-Robots-Tag", "noindex, nofollow, noarchive")
        self.send_header("Referrer-Policy", "same-origin")
        super().end_headers()

    def do_GET(self):
        if not self.valid_origin():
            return
        route = urlsplit(self.path).path
        if route == "/preview-login.css":
            return self.send_bytes(200, (APP / "preview-login.css").read_bytes(), "text/css; charset=utf-8")
        if route in ("/", "/login") and not self.authenticated():
            return self.login_page()
        if not self.permitted():
            return
        if route == "/preview-layout.css":
            return self.send_bytes(200, (APP / "preview-layout.css").read_bytes(), "text/css; charset=utf-8")
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
                content = content.replace("<body>", '<body><aside class="cloud-preview-note"><span>OmindOS 云端体验 · 单人预览｜计算在腾讯云运行，当前为几何仿真。修改内容保存在此预览空间。</span><form action="' + PREFIX + 'logout" method="post"><button type="submit">退出</button></form></aside>')
            if route.endswith(".js"):
                content = content.replace("import('/", "import('" + PREFIX).replace('import("/', 'import("' + PREFIX)
            if route.endswith(".css"):
                content += "\n.cloud-preview-note{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:8px;padding:10px 18px;background:#102c40;color:#e4f3ff;font:500 14px/1.6 system-ui,sans-serif;border-bottom:1px solid #28627d;}.cloud-preview-note form{margin:0}.cloud-preview-note button{width:auto;margin:0;padding:8px 16px;background:#19445c;color:#e4f3ff;border:1px solid #4b7c95;border-radius:8px;font-size:14px;}\n"
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
        self.send_bytes(200, data, mime)

    def do_HEAD(self):
        route = urlsplit(self.path).path
        if route in ("/", "/login", "/preview-login.css", "/preview-bootstrap.js") or route in robot_workbench.ASSETS:
            return self.do_GET()
        if self.permitted():
            self.send_bytes(405, b"", "text/plain", (("Allow", "GET, POST"),))

server = robot_workbench.ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
print("OmindOS private preview listening on loopback", flush=True)
try:
    server.serve_forever()
finally:
    server.server_close()