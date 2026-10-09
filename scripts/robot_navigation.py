"""Existing nav3d_native core in a bounded geometric desktop simulation.

No ROS, sensor localization, physical brake or hardware driver is implied.
The kernel is unchanged; this adapter closes its permissive endpoint/corner
behaviour and confirms simulated zero motion before submitting manipulation.
"""
import copy
import math
import threading
import time
import uuid

import numpy as np
from navigation_kernel import Pose2D, astar, pure_pursuit, normalize_angle
from navigation_guard import PlatformGuard, PlatformProfile
from robot_kinematics import base_position, pose
from robot_parameter_api import number
from robot_planning import ProfileRobot, default_mapping
from workbench_state import MotionLocked

KERNEL_SOURCE = 'Omind-Robotics/OmindOS-Navigation@3293dfc83e32b8e56f1f185b80e19e56a2fe600d:src/nav3d_native/nav3d_native/algorithms.py'
ACTIVE = {'planning', 'navigating', 'aligning', 'parking', 'manipulation_planning'}


def default_config():
    return {'drive_type': 'differential', 'max_linear_m_s': .25,
            'max_angular_rad_s': .6, 'safety_margin_m': .08,
            'map': {'size_m': [10, 10], 'origin_m': [-5, -5], 'resolution_m': .1}}


def finite(value, name, low, high):
    value = number(value, name)
    if not low <= value <= high:
        raise ValueError(f'{name} must be between {low} and {high}')
    return value


class NavigationSimulation:
    def __init__(self, payload, check):
        allowed = {'profile', 'start', 'base_pose', 'goal', 'config', 'mapping', 'obstacles', 'manipulation'}
        if not isinstance(payload, dict) or set(payload) - allowed or not {'profile', 'start', 'goal'} <= set(payload):
            raise ValueError('导航需要 profile、start、goal，以及可选 config/mapping/obstacles/manipulation。')
        self.check = check
        manipulation=payload.get('manipulation')
        if manipulation is not None:
            if not isinstance(manipulation,dict) or set(manipulation) != {'translation_m'} or not isinstance(manipulation['translation_m'],list) or len(manipulation['translation_m'])!=3:
                raise ValueError('导航后操作需指定三维共同 translation_m。')
            for value in manipulation['translation_m']: finite(value,'translation_m',-.25,.25)
        self.config = copy.deepcopy(payload.get('config', default_config()))
        if not isinstance(self.config, dict) or set(self.config) - set(default_config()) or self.config.get('drive_type') not in {'differential', 'omnidirectional'}:
            raise ValueError('请选择 differential 差速或 omnidirectional 全向运动学配置。')
        self.speed = finite(self.config.get('max_linear_m_s'), 'max_linear_m_s', .02, .5)
        self.angular = finite(self.config.get('max_angular_rad_s'), 'max_angular_rad_s', .05, 1.2)
        self.kind = self.config['drive_type']
        self.guard = PlatformGuard(PlatformProfile('customer',self.kind,self.speed,self.speed if self.kind=='omnidirectional' else 0,self.angular),enabled=True)
        margin = finite(self.config.get('safety_margin_m'), 'safety_margin_m', .02, .5)
        self.mapping = payload.get('mapping', default_mapping())
        self.obstacles = payload.get('obstacles', [])
        self.robot = ProfileRobot(payload['profile'], payload['start'], payload.get('base_pose'), self.mapping, self.obstacles, check)
        q = np.array([self.robot.start[n] for n in self.robot.names])
        if not self.robot.clear(q):
            raise ValueError('当前整机姿态碰撞：' + ' / '.join(self.robot.last_collision))
        self.q = dict(self.robot.start)
        self.profile = copy.deepcopy(payload['profile'])
        self.base = dict(self.robot.base)
        self.p = Pose2D(self.base['x_m'], self.base['y_m'], self.base['yaw_rad'])
        self.goal = base_position(payload['goal'])
        world = self.robot.world(q)
        # All corners, including extended arms: never use only the chassis size.
        corners = []
        for name, local, half in self.robot.shapes:
            t = world[name] @ local
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        corner = t[:3, 3] + t[:3, :3] @ (half * [sx, sy, sz])
                        corners.append(math.hypot(corner[0]-self.p.x, corner[1]-self.p.y))
        self.radius = max(corners) + margin
        if self.radius > 5:
            raise ValueError('整机导航包络超过五米，请检查模型尺寸。')
        spec = self.config.get('map')
        if not isinstance(spec, dict) or set(spec) - {'size_m', 'origin_m', 'resolution_m', 'data'}:
            raise ValueError('map 需要 size_m/origin_m/resolution_m，可选 OccupancyGrid data。')
        self.res = finite(spec.get('resolution_m'), 'resolution_m', .05, .25)
        size, origin = spec.get('size_m'), spec.get('origin_m')
        if not isinstance(size, list) or not isinstance(origin, list) or len(size) != 2 or len(origin) != 2:
            raise ValueError('地图 size_m 和 origin_m 需为二维数组。')
        self.width, self.height = [math.ceil(finite(v, 'map size_m', 1, 40)/self.res) for v in size]
        if self.width*self.height > 40000:
            raise ValueError('导航地图最多 40000 格。')
        self.ox, self.oy = [finite(v, 'map origin_m', -100, 100) for v in origin]
        raw = spec.get('data', [0]*(self.width*self.height))
        if not isinstance(raw, list) or len(raw) != self.width*self.height or any(type(v) is not int or v < -1 or v > 100 for v in raw):
            raise ValueError('地图 data 需逐格提供 -1..100 的整数。')
        self.synthetic_map = 'data' not in spec
        # Unknown and uncertain occupancy fail closed. Known obstacle cells and
        # 3D planning boxes share one world map, projected conservatively to XY.
        self.rects = []
        for i, value in enumerate(raw):
            if value != 0:
                self.rects.append((self.ox+(i%self.width)*self.res, self.oy+(i//self.width)*self.res, self.res, self.res))
        for center, _, half in self.robot.obstacles:
            self.rects.append((float(center[0]-half[0]), float(center[1]-half[1]), float(half[0]*2), float(half[1]*2)))
        self.rect_array = np.array(self.rects,float).reshape(-1,4)
        # Vector rasterization avoids quadratic Python loops for imported maps.
        grid = np.zeros((self.height,self.width),dtype=np.uint8)
        cell_margin = self.res*math.sqrt(2)/2
        r = self.radius+cell_margin
        yy,xx = np.mgrid[:self.height,:self.width]
        xx=self.ox+(xx+.5)*self.res; yy=self.oy+(yy+.5)*self.res
        grid[(xx<=self.ox+r)|(xx>=self.ox+self.width*self.res-r)|(yy<=self.oy+r)|(yy>=self.oy+self.height*self.res-r)] = 100
        grid[(np.abs(xx)>100-cell_margin)|(np.abs(yy)>100-cell_margin)] = 100
        from scipy.ndimage import distance_transform_edt
        raw_blocked=np.array(raw).reshape(self.height,self.width)!=0
        if raw_blocked.any():
            grid[distance_transform_edt(~raw_blocked)*self.res <= r+self.res*math.sqrt(2)/2] = 100
        for center,_,half in self.robot.obstacles:
            check()
            dx=np.maximum(np.abs(xx-center[0])-half[0],0)
            dy=np.maximum(np.abs(yy-center[1])-half[1],0)
            grid[dx*dx+dy*dy<=r*r]=100
        grid=grid.ravel().tolist()
        start, goal = self.cell(self.p.x, self.p.y), self.cell(self.goal['x_m'], self.goal['y_m'])
        def valid(c):
            return 0 <= c[0] < self.width and 0 <= c[1] < self.height and not grid[c[1]*self.width+c[0]]
        if not valid(start) or not valid(goal):
            raise ValueError('起点或目标在整机安全包络、未知区域或地图边界内，导航拒绝。')
        cells = astar(grid, self.width, self.height, start, goal, inflation_radius=0)
        if not cells:
            raise ValueError('未找到整机可通过的导航路径。')
        points = [(self.p.x, self.p.y)]
        for a, b in zip(cells, cells[1:]):
            if a[0] != b[0] and a[1] != b[1]:
                # The inherited A* permits corner cutting. Insert a certified
                # orthogonal cell or reject; do not alter the reused kernel.
                corners = [(a[0], b[1]), (b[0], a[1])]
                corner = next((c for c in corners if valid(c)), None)
                if corner is None:
                    raise ValueError('导航路径切角，安全检查拒绝。')
                points.append(self.world(*corner))
            points.append(self.world(*b))
        points.append((self.goal['x_m'], self.goal['y_m']))
        self.path = []
        for point in points:
            if not self.path or math.dist(self.path[-1], point) > 1e-8:
                if self.path and not self.edge_clear(self.path[-1], point):
                    raise ValueError('导航路径包络检查未通过。')
                self.path.append(point)
        self.status = 'navigating'
        self.elapsed = 0.
        self.v = self.vy = self.w = 0.
        self.stable = 0.
        self.parking_confirmed = False
        self.waypoint = 1 if len(self.path) > 1 else 0
        self.timeout = min(180, max(20, sum(math.dist(a,b) for a,b in zip(self.path,self.path[1:]))/self.speed*4+30))

    def world(self, x, y):
        return self.ox+(x+.5)*self.res, self.oy+(y+.5)*self.res

    def cell(self, x, y):
        return math.floor((x-self.ox)/self.res), math.floor((y-self.oy)/self.res)

    def clear(self, x, y, radius=None):
        r = self.radius if radius is None else radius
        if abs(x)>100 or abs(y)>100: return False
        if not (self.ox+r < x < self.ox+self.width*self.res-r and self.oy+r < y < self.oy+self.height*self.res-r):
            return False
        if len(self.rect_array):
            a=self.rect_array
            dx=np.maximum(np.maximum(a[:,0]-x,x-a[:,0]-a[:,2]),0)
            dy=np.maximum(np.maximum(a[:,1]-y,y-a[:,1]-a[:,3]),0)
            if np.any(dx*dx+dy*dy<=r*r): return False
        return True

    def edge_clear(self, a, b):
        count = max(1, math.ceil(math.dist(a,b)/min(.02,self.res/4)))
        return all(self.clear(a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count) for i in range(count+1))

    def arm_environment_clear(self, boxes):
        self.check()
        for center,rotation,half in boxes:
            # Conservative XY disks enclosing each oriented box. OccupancyGrid
            # has no height: occupied/unknown cells are treated as full columns.
            horizontal_half=np.abs(rotation[:2,:])@half
            radius=float(np.linalg.norm(horizontal_half))
            if not self.clear(float(center[0]),float(center[1]),radius):
                return False
        return True

    def step(self, dt):
        self.check()
        if not 0 < dt <= .1:
            raise ValueError('navigation simulation dt must be in (0,.1]')
        if self.status not in {'navigating', 'aligning', 'parking'}:
            return
        self.elapsed += dt
        if self.elapsed > self.timeout:
            raise ValueError('导航超时；双臂操作未启动。')
        old = self.p
        self.vy=0.
        distance = math.hypot(self.goal['x_m']-old.x, self.goal['y_m']-old.y)
        error = normalize_angle(self.goal['yaw_rad']-old.yaw)
        if self.status == 'parking' and (distance > .045 or abs(error) > math.radians(3)):
            raise ValueError('停车位姿偏移；双臂操作未启动。')
        if distance <= .035:
            self.status = 'aligning' if abs(error) > math.radians(2) else 'parking'
            self.v = 0.
            self.w = max(-self.angular,min(self.angular, 2*error)) if self.status == 'aligning' else 0.
        else:
            # Feed short forward route sections to the original pursuit kernel.
            # Its 22 cm terminal tolerance is followed by precise final approach.
            while self.waypoint < len(self.path)-1 and math.dist((old.x,old.y),self.path[self.waypoint]) < .06:
                self.waypoint += 1
            remaining = self.path[self.waypoint:]
            self.v, self.w, reached = pure_pursuit(old, remaining, lookahead=.12, max_linear=self.speed, max_angular=self.angular)
            if reached:
                heading = math.atan2(self.goal['y_m']-old.y,self.goal['x_m']-old.x)
                delta = normalize_angle(heading-old.yaw)
                self.v = min(self.speed, distance*.8)*max(0,math.cos(delta))**2
                self.w = max(-self.angular,min(self.angular,2.2*delta))
            if self.kind=='omnidirectional':
                # The shared A* route is tracked with a holonomic body twist;
                # lateral travel does not require first rotating the chassis.
                tx,ty=self.path[self.waypoint]
                dx,dy=tx-old.x,ty-old.y
                scale=min(self.speed/max(math.hypot(dx,dy),1e-9),1.5)
                self.v=(math.cos(old.yaw)*dx+math.sin(old.yaw)*dy)*scale
                self.vy=(-math.sin(old.yaw)*dx+math.cos(old.yaw)*dy)*scale
                self.w=max(-self.angular,min(self.angular,2*error))
            self.status = 'navigating'
        now=self.elapsed+1
        for source in ('pose','odom','plan'): self.guard.observe(source,True,now)
        self.guard.receive_command((self.v,self.vy,0,0,0,self.w),now)
        guard_state,cmd=self.guard.evaluate(now)
        if guard_state not in {'READY','LIMITED'}: raise MotionLocked('导航命令联锁：'+guard_state)
        self.v,self.vy,self.w=cmd[0],cmd[1],cmd[5]
        # Exact SE(2) integration of the differential command; geometry only.
        yaw = normalize_angle(old.yaw+self.w*dt)
        if abs(self.w) < 1e-9:
            x=old.x+(self.v*math.cos(old.yaw)-self.vy*math.sin(old.yaw))*dt
            y=old.y+(self.v*math.sin(old.yaw)+self.vy*math.cos(old.yaw))*dt
        else:
            x = old.x+self.v/self.w*(math.sin(yaw)-math.sin(old.yaw))
            y = old.y-self.v/self.w*(math.cos(yaw)-math.cos(old.yaw))
            x += self.vy/self.w*(math.cos(yaw)-math.cos(old.yaw))
            y += self.vy/self.w*(math.sin(yaw)-math.sin(old.yaw))
        if not self.edge_clear((old.x,old.y),(x,y)):
            raise ValueError('路径跟踪将进入整机安全包络，已停止；双臂操作未启动。')
        self.p = Pose2D(x,y,yaw)
        # Derived wheel animation only when the current URDF explicitly declares
        # the supported reference wheels. Navigation never assumes motor IDs.
        for name, wheel in ((self.robot.model.get('wheel_base') or {}).get('wheels', {}) if self.kind=='differential' else {}).items():
            rate = wheel['axis_sign']*(self.v-self.w*wheel['y_m'])/wheel['radius_m']
            self.q[name] += rate*dt
        observed_v, observed_w = math.dist((x,y),(old.x,old.y))/dt, abs(normalize_angle(yaw-old.yaw))/dt
        if self.status == 'parking' and observed_v < .001 and observed_w < .001 and self.v == self.vy == self.w == 0:
            self.stable += dt
            if self.stable >= .6:
                self.parking_confirmed = True
                self.status = 'parked'
        else:
            self.stable = 0.
        self.base = {'x_m':x,'y_m':y,'yaw_rad':yaw}

    def snapshot(self):
        return {'status':self.status, 'elapsed_s':self.elapsed, 'goal':self.goal,
                'base_pose':self.base, 'pose':pose(self.robot.model,self.q,self.base),
                'cmd_vel':{'linear_m_s':self.v,'lateral_m_s':self.vy,'angular_rad_s':self.w}, 'drive_type':self.kind,
                'parking_confirmed':self.parking_confirmed,'stable_stop_s':self.stable,
                'footprint_radius_m':self.radius,'path':self.path,
                'obstacles_xy':self.rects,'obstacles':self.obstacles, 'map_bounds':{'origin_m':[self.ox,self.oy],'size_m':[self.width*self.res,self.height*self.res]},'synthetic_map':self.synthetic_map,
                'simulation_only':True,'localization_source':'geometric_simulator',
                'kernel_source':KERNEL_SOURCE,'hardware_executed':False}


class NavigationJobs:
    def __init__(self, state, planning):
        self.state, self.planning = state, planning
        self.job = None
        self.lock = threading.RLock()

    def active(self):
        return self.job is not None and self.job['status'] in ACTIVE

    def ensure_idle(self):
        with self.lock:
            if self.active():
                raise MotionLocked('底盘导航或任务衔接中，暂时锁定其他运动。请先取消导航。')

    def submit(self, payload):
        with self.lock:
            self.ensure_idle()
            with self.planning.lock:
                if self.planning.job and self.planning.job['status'] == 'running':
                    raise MotionLocked('请先取消当前双臂规划，再启动导航。')
            epoch = self.state.check_motion()
            job = {'task_id':uuid.uuid4().hex,'status':'planning','motion_epoch':epoch,
                   'error':None,'result':None,'cancel':threading.Event(),'simulation':None,
                   'updated_at':time.monotonic(), 'client_seen_at':time.monotonic()}
            self.job = job
            payload = copy.deepcopy(payload)
        def check():
            self.state.check_motion(epoch)
            if job['cancel'].is_set():
                raise MotionLocked('导航已取消；未继续双臂操作。')
            if time.monotonic()-job['client_seen_at'] > 3:
                raise MotionLocked('工作台连接中断，导航已停止；未继续双臂操作。')
        def run():
            try:
                sim = NavigationSimulation(payload,check)
                with self.lock:
                    check(); job['simulation']=sim
                last = time.monotonic()
                while sim.status != 'parked':
                    # Fixed-step geometric time; a suspended process cannot jump
                    # to arrival or fabricate the required parking interval.
                    if job['cancel'].wait(.05):
                        check()
                    now = time.monotonic()
                    if now-last > .5:
                        raise MotionLocked('导航反馈中断，已停止；请重新启动任务。')
                    with self.lock:
                        sim.step(.05); job.update(status=sim.status,updated_at=now)
                    last = now
                with self.lock:
                    check()
                    job.update(status='parked',updated_at=time.monotonic())
                    manipulation = payload.get('manipulation')
                    if manipulation is not None:
                        if not isinstance(manipulation,dict) or set(manipulation) != {'translation_m'}:
                            raise ValueError('导航后操作需指定共同 translation_m。')
                        # Arrival and independently observed simulated zero
                        # displacement, rather than end of planned path, gate it.
                        if not sim.parking_confirmed:
                            raise MotionLocked('停车尚未确认。')
                        job['status'] = 'manipulation_planning'
                        task = self.planning.submit({'profile':sim.profile,'start':sim.q,
                            'base_pose':sim.base,'mapping':sim.mapping,'obstacles':sim.obstacles,
                            'mode':'object','translation_m':manipulation['translation_m']},sim.arm_environment_clear)
                        job['arm_task_id'] = task['task_id']
                if manipulation is not None:
                    while True:
                        if job['cancel'].wait(.05): check()
                        check()
                        task = self.planning.snapshot(job['arm_task_id'])
                        if task['status'] != 'running': break
                    with self.lock:
                        check()
                        if task['status'] != 'succeeded': raise ValueError(task['error'] or '停车后双臂规划未通过。')
                        job.update(status='succeeded',result=task['result'],updated_at=time.monotonic())
            except Exception as e:
                with self.lock:
                    if job.get('arm_task_id'):
                        self.planning.cancel(job['arm_task_id'])
                    sim=job['simulation']
                    if sim is not None:
                        sim.v=sim.vy=sim.w=0.; sim.parking_confirmed=False
                    job.update(status='canceled' if job['cancel'].is_set() or self.state.epoch!=epoch else 'failed',result=None,error=str(e),updated_at=time.monotonic())
        threading.Thread(target=run,daemon=True).start()
        return self.snapshot(job['task_id'])

    def snapshot(self, task_id):
        with self.lock:
            if not self.job or task_id != self.job['task_id']:
                raise ValueError('Unknown navigation task')
            job=self.job
            job['client_seen_at']=time.monotonic()
            if self.active() and (self.state.epoch != job['motion_epoch'] or time.monotonic()-job['updated_at'] > (10 if job['status']=='planning' else 1) and job['status']!='manipulation_planning'):
                self.cancel(task_id)
            result={k:v for k,v in job.items() if k not in {'cancel','simulation','updated_at','client_seen_at'}}
            if self.state.epoch!=job['motion_epoch'] and result['status'] in {'succeeded','parked'}:
                result.update(status='canceled',result=None,error='参数或急停状态已改变，导航结果作废。')
            if job['simulation'] is not None: result['navigation']=job['simulation'].snapshot()
            return copy.deepcopy(result)

    def cancel(self, task_id=None):
        with self.lock:
            if not self.job or task_id is not None and task_id != self.job['task_id']:
                raise ValueError('Unknown navigation task')
            self.job['cancel'].set()
            if self.job.get('arm_task_id'): self.planning.cancel(self.job['arm_task_id'])
            if self.job['simulation'] is not None:
                self.job['simulation'].v=self.job['simulation'].vy=self.job['simulation'].w=0.
                self.job['simulation'].parking_confirmed=False
            self.job.update(status='canceled',result=None,error='导航已取消；未继续双臂操作。')
            return self.snapshot(self.job['task_id'])
