#!/usr/bin/env python3
"""Adapt an isolated copy of the verified live adapter for private acceptance.

Never point this at the public app. The pinned hashes reject a different live
version before writing anything; navigation Python/JS changes ship separately.
"""
import argparse
import hashlib
from pathlib import Path

EXPECTED = {
    'cloud_preview.py': '5c89e1890877c59982223fbd98954504486f5630a218d644d40ea681bb428671',
    'preview-bootstrap.js': '5ab8913e3be2895a19f54230544e2ae2aa5635dff26cae25b1be5247c539a76e',
    'preview-sw.js': 'c8b3c183bbc61857bd1eee78af47825c60135e3f7ab74f64825b1be3a07350fd',
    'preview-login.html': 'ec37b68876420f19de8d97a493abb2890f8ef7408e792ff6ed29ed3d7664a974',
}


def replace(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError(f'adapter insertion point changed: {old[:70]}')
    return text.replace(old, new)


def prepare(app, output):
    sources = {}
    for name, expected in EXPECTED.items():
        data = (app / name).read_bytes()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f'{name}: live adapter version changed; review before preparing')
        sources[name] = data.decode()
    text = sources['cloud_preview.py']
    text = replace(text, '{"/", "/workbench/"}', '{"/", "/workbench/", "/_navigation-preview/"}')
    text = replace(text, '{"https://omindos.cn", "https://cloud.omindos.cn"}',
                   '{"https://omindos.cn", "https://cloud.omindos.cn", "https://cloud.omindos.cn:19443"}')
    text = replace(text, 'SESSION_COOKIE = "__Secure-omindos-preview"',
                   'SESSION_COOKIE = "__Secure-omindos-navigation-preview"')
    text = replace(text, 'CSRF_COOKIE = "__Secure-omindos-preview-login"',
                   'CSRF_COOKIE = "__Secure-omindos-navigation-preview-login"')
    text = replace(text, '    def public_origin(self):\n',
                   '    def public_origin(self):\n'
                   '        if self.headers.get("X-OmindOS-Navigation-Preview") == "1":\n'
                   '            return "https://cloud.omindos.cn"\n')
    marker = "        if route.startswith('/preview-meshes/'):"
    text = replace(text, marker, '''        if route.startswith('/preview-meshes-v2/'):
            match=re.fullmatch(r'/preview-meshes-v2/([a-f0-9]{64}\\.bin)',route)
            file=Path('/packed')/match[1] if match else None
            if not file or not file.is_file():return self.respond(404,{'error':'display mesh unavailable'})
            headers=[]
            if 'gzip' in self.headers.get('Accept-Encoding','') and file.with_suffix('.bin.gz').is_file():
                file=file.with_suffix('.bin.gz');headers.append(('Content-Encoding','gzip'))
            return self.send_bytes(200,file.read_bytes(),'application/octet-stream',headers)
''' + marker)
    # The current router already separates guest/user runtimes. Gate access
    # before dispatch, retaining its origin checks and authorized handlers.
    marker = "        route = urlsplit(self.path).path\n        handler_class = AuthenticatedHandler"
    text = replace(text, marker, "        route = urlsplit(self.path).path\n"
                   "        if not self.authenticated() and route not in ('/login','/preview-login.css'):\n"
                   "            if route == '/': return self.login_page()\n"
                   "            return self.respond(401,{'error':'请先登录导航验收预览。'})\n"
                   "        handler_class = AuthenticatedHandler")
    compile(text, 'cloud_preview.py', 'exec')
    sources['cloud_preview.py'] = text
    text = sources['preview-bootstrap.js']
    for name in ('getItem', 'setItem', 'removeItem'):
        text = replace(text, 'window.localStorage.' + name + '(key',
                       'window.localStorage.' + name + "('omindos-navigation-preview:'+key")
    sources['preview-bootstrap.js'] = text
    sources['preview-sw.js'] = replace(sources['preview-sw.js'], 'omindos-preview-pwa-',
                                       'omindos-navigation-preview-pwa-', count=2)
    text = sources['preview-login.html']
    text = replace(text, '体验和视频预览无需登录。注册登录后，可下载视频、配置并保存自己的系统。',
                   '导航稳定性验收预览需要授权登录。云端几何仿真，未连接真机。')
    for fragment in ('<p class="help"><a href="{{switch_url}}">{{switch}}</a></p>',
                     '<p class="help"><a href="/workbench/">直接体验，无需登录 →</a></p>',
                     '<button type="button" data-pwa-install class="login-install">安装应用</button>'):
        text = replace(text, fragment, '')
    begin, end = text.index('<link rel="manifest"'), text.index('</head>')
    text = text[:begin] + text[end:]
    sources['preview-login.html'] = text
    output.mkdir(parents=True, exist_ok=True)
    for name, text in sources.items():
        (output / name).write_text(text)
        print(name, hashlib.sha256(text.encode()).hexdigest())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', required=True, type=Path, help='verified live app, read only')
    parser.add_argument('--output', required=True, type=Path, help='separate candidate app copy')
    args = parser.parse_args()
    if args.app.resolve() == args.output.resolve():
        parser.error('source and candidate directories must be different')
    prepare(args.app, args.output)
