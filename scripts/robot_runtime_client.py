"""Optional workbench bridge to a explicitly selected localhost ROS preview."""
import json
from urllib.error import HTTPError,URLError
from urllib.parse import urlsplit
from urllib.request import Request,urlopen


class RuntimeClient:
    def __init__(self,base_url):
        parsed=urlsplit(base_url)
        if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or not parsed.port or parsed.path not in ('','/') or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError('--runtime-url must be http://127.0.0.1:PORT or http://localhost:PORT')
        self.base=base_url.rstrip('/')

    def request(self,path,payload=None):
        allowed={'/v1/runtime/'+p for p in ('status','load','run','stop','heartbeat')}
        if path not in allowed:
            return 404,{'error':'runtime route not found'}
        req=Request(self.base+path,data=None if payload is None else json.dumps(payload,allow_nan=False).encode(),
                    headers={} if payload is None else {'Content-Type':'application/json'})
        try:
            with urlopen(req,timeout=7) as response:
                data=json.load(response)
                if not isinstance(data,dict): raise ValueError('invalid runtime response')
                return response.status,data
        except HTTPError as e:
            try: return e.code,json.load(e)
            except (ValueError,OSError): return 502,{'error':'invalid runtime error response'}
        except (URLError,OSError,ValueError):
            return 503,{'error':'ROS 2 preview unavailable; start robot_profile_runtime.py with the same configuration store'}
