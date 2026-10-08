#!/usr/bin/env python3
"""ROS 2 joint-preview node plus localhost control API; no hardware driver."""
import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import threading
import time

from robot_parameter_api import MAX_BYTES
from robot_profile_runtime_core import ProfileRuntime, RuntimeErrorResponse


class RuntimeMailbox:
    """Serialize HTTP commands on the ROS executor; expired requests never run later."""
    def __init__(self):
        self.requests = queue.Queue(maxsize=64)
        self.lock = threading.Lock()
        self.snapshot = None
        self.updated_at = 0.0

    def submit(self, op, data, timeout=5):
        request = {'op':op,'data':data,'done':threading.Event(),'deadline':time.monotonic()+timeout,
                   'lock':threading.Lock(),'expired':False}
        try:
            self.requests.put_nowait(request)
        except queue.Full:
            return 503, {'error':'runtime command queue full'}
        if not request['done'].wait(timeout):
            with request['lock']:
                if not request['done'].is_set():
                    request['expired'] = True
                    return 503, {'error':'runtime response timed out; query status before retrying'}
        return request['result']

    def process(self, core):
        for _ in range(16):
            try: request = self.requests.get_nowait()
            except queue.Empty: break
            with request['lock']:
                if request['expired'] or time.monotonic()>request['deadline']:
                    result = (503, {'error':'command expired before execution'})
                else:
                    try: result = (200, core.command(request['op'],request['data']))
                    except RuntimeErrorResponse as e: result = (e.code, {'error':str(e)})
                    except (ValueError,TypeError,OverflowError,RecursionError) as e: result = (400, {'error':str(e)})
                    except OSError: result = (500, {'error':'profile storage unavailable'})
                request['result'] = result
                request['done'].set()


def make_handler(mailbox):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, code, body):
            data = json.dumps(body,allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError): pass

        def allowed(self):
            host=self.headers.get('Host','')
            if host not in {f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'} or self.headers.get('Origin') not in (None,'http://'+host):
                self.respond(403,{'error':'local same-origin requests only'})
                return False
            return True

        def do_GET(self):
            if not self.allowed(): return
            if self.path!='/v1/runtime/status': return self.respond(404,{'error':'route not found'})
            with mailbox.lock:
                snapshot=copy.deepcopy(mailbox.snapshot) if time.monotonic()-mailbox.updated_at<1 else None
            self.respond(200 if snapshot else 503,snapshot or {'error':'ROS executor not ready'})

        def do_POST(self):
            if not self.allowed(): return
            if self.path not in ['/v1/runtime/'+op for op in ('load','run','stop','heartbeat')]:
                return self.respond(404,{'error':'route not found'})
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':
                return self.respond(415,{'error':'Content-Type must be application/json'})
            try:
                self.connection.settimeout(6)
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=MAX_BYTES or self.headers.get('Transfer-Encoding'):
                    return self.respond(413,{'error':'invalid request size'})
                raw=self.rfile.read(size)
                if len(raw)!=size: raise ValueError('incomplete request')
                code,body=mailbox.submit(self.path.rsplit('/',1)[1],json.loads(raw))
                self.respond(code,body)
            except (ValueError,TypeError,RecursionError) as e: self.respond(400,{'error':str(e)})
            except OSError: self.respond(500,{'error':'request I/O failed'})

        def log_message(self,*args): pass
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8086)
    args,ros_args=parser.parse_known_args()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
    except ImportError as e:
        raise SystemExit('Source ROS 2 Humble setup.bash before starting the preview runtime: '+str(e))
    box=RuntimeMailbox()
    core=ProfileRuntime(args.store)

    class PreviewNode(Node):
        def __init__(self):
            super().__init__('robot_profile_preview')
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.description=self.create_publisher(String,'/omindos_workbench/robot_description',latched)
            self.state=self.create_publisher(String,'/omindos_workbench/runtime_status',latched)
            self.joints=self.create_publisher(JointState,'/omindos_workbench/joint_states',10)
            self.last_generation=-1
            self.ticks=0
            self.create_timer(1/30,self.update)

        def update(self):
            box.process(core)
            core.tick()
            snapshot=core.snapshot()
            if core.profile and core.generation!=self.last_generation:
                desc=String(); desc.data=core.profile['kinematics']['urdf_xml']; self.description.publish(desc)
                self.last_generation=core.generation
            if core.profile:
                msg=JointState(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=core.model['root']
                msg.name=list(core.q); msg.position=[float(core.q[n]) for n in msg.name]
                msg.velocity=[float(core.velocity[n]) for n in msg.name]
                msg.effort=[float(snapshot['pose']['holding_torque_nm'][n]) for n in msg.name]
                self.joints.publish(msg)
            with box.lock:
                box.snapshot=snapshot
                box.updated_at=time.monotonic()
            if self.ticks%3==0:
                info=String(); info.data=json.dumps({k:v for k,v in snapshot.items() if k not in ('model','pose')},allow_nan=False)
                self.state.publish(info)
            self.ticks+=1

    rclpy.init(args=ros_args)
    node=PreviewNode()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(box))
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    print(f'ROS 2 joint preview: http://127.0.0.1:{server.server_port}/v1/runtime/status',flush=True)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
