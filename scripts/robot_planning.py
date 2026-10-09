"""Workbench adapter for the unchanged runtime.2 coupled planning kernel.

Collision geometries are enclosed in local oriented boxes. This is deliberately
conservative: a rejected path may be clear in the exact mesh model. Only sampled
configurations are certified, with explicit structural self-collision exclusions.
"""
import copy
import json
import math
from pathlib import Path
import struct
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from dual_task_planner import DualTaskPlanner, PlanningFailure, TrackingSettings
from robot_assets import MESH_ASSETS, approved_mesh_url
from robot_kinematics import build_model, pose, positions, base_position, origin, vec
from robot_parameter_api import number

ROOT = Path(__file__).resolve().parents[1]
KERNEL_COMMIT = 'cbe8caf13921002e463355c6a2982e900640c7e0'


def default_mapping():
    return {'waist': 'waist_pitch',
            'left': [f'left_joint_{i}' for i in range(1, 8)],
            'right': [f'right_joint_{i}' for i in range(1, 8)],
            'tcp_links': ['left_tip', 'right_tip']}


def mesh_bounds(filename, scale):
    url = approved_mesh_url(filename)
    if url not in MESH_ASSETS:
        raise ValueError('碰撞网格未打包，请提供本地已支持的网格或明确的 box 碰撞几何：' + filename)
    raw = (ROOT / MESH_ASSETS[url]).read_bytes()
    if len(raw) < 84:
        raise ValueError('Expected a binary STL collision mesh')
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + 50 * count or count > 200000:
        raise ValueError('Invalid or oversized binary STL')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    points = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1, 3) * scale
    if not len(points) or not np.isfinite(points).all():
        raise ValueError('Invalid mesh vertices')
    return points.min(axis=0), points.max(axis=0)


def overlap(a, b):
    """Separating-axis test for two enclosing OBBs; touching is collision."""
    ca, ra, ha = a
    cb, rb, hb = b
    if np.any(np.abs(ca-cb) > np.abs(ra) @ ha + np.abs(rb) @ hb + 1e-10):
        return False
    axes = [*ra.T, *rb.T] + [np.cross(x, y) for x in ra.T for y in rb.T]
    delta = cb - ca
    for axis in axes:
        if axis @ axis < 1e-16:
            continue
        radius = np.abs(axis @ ra) @ ha + np.abs(axis @ rb) @ hb
        if abs(delta @ axis) > radius + 1e-10:
            return False
    return True


class ProfileRobot:
    def __init__(self, profile, start, base, mapping, obstacles, check):
        if (not isinstance(profile,dict) or not isinstance(profile.get('kinematics'),dict)
                or not isinstance(profile['kinematics'].get('urdf_xml'),str)):
            raise ValueError('规划需要当前客户 URDF 运动学模型。')
        self.model = build_model(profile['kinematics']['urdf_xml'])
        self.start = positions(self.model, start)
        self.base = base_position(base)
        self.check = check
        if (not isinstance(mapping, dict) or set(mapping) != {'waist', 'left', 'right', 'tcp_links'}
                or not isinstance(mapping['left'], list) or not isinstance(mapping['right'], list)
                or len(mapping['left']) != 7 or len(mapping['right']) != 7
                or not isinstance(mapping['tcp_links'], list) or len(mapping['tcp_links']) != 2):
            raise ValueError('配置需指定 waist、左右各七个关节和两个 tcp_links。')
        self.names = [mapping['waist'], *mapping['left'], *mapping['right']]
        if any(not isinstance(n, str) for n in self.names + mapping['tcp_links']) or len(set(self.names)) != 15:
            raise ValueError('十五个规划关节必须互不重复。')
        joints = {j['name']: j for j in self.model['joints']}
        links = {l['name'] for l in self.model['links']}
        if any(n not in joints or joints[n]['type'] != 'revolute' for n in self.names):
            raise ValueError('规划关节必须是当前 URDF 中独立且有限位的 revolute 关节。')
        if any(n not in links for n in mapping['tcp_links']):
            raise ValueError('TCP 连杆不在当前 URDF 中。')
        self.tcp_links = mapping['tcp_links']
        self.limits = np.array([[joints[n]['lower'], joints[n]['upper']] for n in self.names])
        self.index = {n: i for i, n in enumerate(self.names)}
        self.chain = []
        ancestry = {self.model['root']: []}
        groups = {self.model['root']: ()}
        for j in self.model['joints']:
            active = j['name'] in self.index
            ancestry[j['child']] = ancestry[j['parent']] + ([j['name']] if active else [])
            groups[j['child']] = tuple(ancestry[j['child']])
            axis = np.array(j['axis'], float); axis /= np.linalg.norm(axis)
            self.chain.append((j['parent'], j['child'], np.array(j['origin']), axis,
                               self.index.get(j['name']), self.start.get(j['name'], 0)))
        for side, tcp in zip(('left', 'right'), self.tcp_links):
            if ancestry[tcp] != [mapping['waist'], *mapping[side]]:
                raise ValueError('TCP 链必须经过共享腰部和对应七轴机械臂，其他活动关节在规划中保持固定。')
        root = ET.fromstring(profile['kinematics']['urdf_xml'])
        self.shapes = []
        for link in root.findall('link'):
            check()
            name = link.get('name')
            geometries = link.findall('collision') or link.findall('visual')
            if not geometries:
                raise ValueError(name + '：缺少碰撞几何或可用外形，不能验收碰撞检查。')
            for geometry in geometries:
                shape = geometry.find('geometry')
                if shape is None or len(shape) != 1:
                    raise ValueError('Invalid collision geometry')
                g = shape[0]
                if g.tag == 'box':
                    half = np.array(vec(g.get('size', ''))) / 2; center = np.zeros(3)
                elif g.tag == 'sphere':
                    half = np.full(3, number(g.get('radius'), 'radius', positive=True)); center = np.zeros(3)
                elif g.tag == 'cylinder':
                    r = number(g.get('radius'), 'radius', positive=True)
                    half = np.array([r, r, number(g.get('length'), 'length', positive=True)/2]); center = np.zeros(3)
                elif g.tag == 'mesh':
                    scale = np.array(vec(g.get('scale', '1 1 1')))
                    if np.any(scale <= 0):
                        raise ValueError('Mesh scale must be positive')
                    low, high = mesh_bounds(g.get('filename', ''), scale)
                    half = (high-low)/2; center = (high+low)/2
                else:
                    raise ValueError('Unsupported collision geometry: ' + g.tag)
                if np.any(half <= 0) or not np.isfinite(half).all():
                    raise ValueError('Collision dimensions must be positive')
                local = np.array(origin(geometry)); local[:3, 3] += local[:3, :3] @ center
                self.shapes.append((name, local, half))
                if len(self.shapes) > 256:
                    raise ValueError('规划碰撞模型最多支持 256 个几何包围盒。')
        self.pairs, self.exclusions = [], set()
        for i, (a, _, _) in enumerate(self.shapes):
            check()
            for k in range(i+1, len(self.shapes)):
                b = self.shapes[k][0]
                ga, gb = groups[a], groups[b]
                common = min(len(ga), len(gb))
                ancestor = ga[:common] == gb[:common]
                # Rigid assemblies and up to two neighbouring planning joints.
                # Branches (the two arms) always retain inter-arm checks.
                if ancestor and abs(len(ga)-len(gb)) <= 2:
                    if a != b:
                        self.exclusions.add(tuple(sorted((a, b))))
                else:
                    self.pairs.append((i, k))
        self.obstacles = []
        if not isinstance(obstacles, list) or len(obstacles) > 32:
            raise ValueError('障碍物需为最多 32 个 box 的数组。')
        for box in obstacles:
            if not isinstance(box, dict) or set(box) != {'size_m', 'position_m'}:
                raise ValueError('障碍物需包含 size_m 和 position_m。')
            size = np.array(box['size_m'], float); pos = np.array(box['position_m'], float)
            if size.shape != (3,) or pos.shape != (3,) or not np.isfinite(np.r_[size,pos]).all() or np.any(size <= 0):
                raise ValueError('障碍物尺寸需为正，位置需为有限三维向量。')
            self.obstacles.append((pos, np.eye(3), size/2))
        self.last_collision = None
        self.cache = None
        self.environment_guard = None

    def world(self, q):
        self.check()
        q = np.asarray(q)
        if self.cache is not None and np.array_equal(q, self.cache[0]):
            return self.cache[1]
        frame = np.eye(4); frame[:3,:3] = Rotation.from_euler('z', self.base['yaw_rad']).as_matrix()
        frame[:2,3] = [self.base['x_m'], self.base['y_m']]
        world = {self.model['root']: frame}
        for parent, child, fixed, axis, index, fixed_q in self.chain:
            angle = fixed_q if index is None else q[index]
            rotation = np.eye(4)
            if angle:
                rotation[:3,:3] = Rotation.from_rotvec(axis * angle).as_matrix()
            world[child] = world[parent] @ fixed @ rotation
        self.cache = (q.copy(), world)
        return world

    def tcps(self, q):
        world = self.world(q)
        return tuple(world[n] for n in self.tcp_links)

    def clear(self, q):
        world = self.world(q)
        boxes = []
        for name, local, half in self.shapes:
            t = world[name] @ local
            boxes.append((t[:3,3], t[:3,:3], half))
        for index,(i, k) in enumerate(self.pairs):
            if index % 128 == 0:
                self.check()
            if overlap(boxes[i], boxes[k]):
                self.last_collision = [self.shapes[i][0], self.shapes[k][0]]
                return False
        for i, box in enumerate(boxes):
            for k, obstacle in enumerate(self.obstacles):
                if overlap(box, obstacle):
                    self.last_collision = [self.shapes[i][0], f'obstacle_{k+1}']
                    return False
        if self.environment_guard is not None and not self.environment_guard(boxes):
            self.last_collision = ['robot', 'navigation_map']
            return False
        return True


def joint_path(robot, start, goal, seed=7):
    """Bounded deterministic bidirectional RRT-Connect, with sampled edges."""
    start, goal = np.array(start), np.array(goal)
    step = .01
    def edge(a, b):
        count = max(1, math.ceil(float(np.max(np.abs(b-a))) / step))
        if count > 1000:
            raise PlanningFailure('关节边超过 1000 样本预算。')
        return [a + (b-a)*u/count for u in range(1, count+1)]
    def checked(a, b):
        return all(robot.clear(q) for q in edge(a, b))
    if not robot.clear(start) or not robot.clear(goal):
        raise PlanningFailure('起点或目标碰撞：' + ' / '.join(robot.last_collision))
    if checked(start, goal):
        return [start.tolist(), *[q.tolist() for q in edge(start, goal)]]
    rng = np.random.default_rng(seed)
    trees = [([start], [-1]), ([goal], [-1])]
    def extend(tree, target):
        nodes, parents = tree
        near = int(np.argmin(np.linalg.norm(np.asarray(nodes)-target, axis=1)))
        delta = target-nodes[near]; norm = np.linalg.norm(delta)
        new = target if norm <= .15 else nodes[near] + .15*delta/norm
        if not checked(nodes[near], new):
            return None
        nodes.append(new); parents.append(near)
        return len(nodes)-1
    def branch(tree, index):
        nodes, parents = tree; path = []
        while index >= 0:
            path.append(nodes[index]); index = parents[index]
        return path[::-1]
    for iteration in range(1600):
        robot.check()
        side = iteration % 2; first, second = trees[side], trees[1-side]
        target = rng.uniform(robot.limits[:,0], robot.limits[:,1])
        i = extend(first, target)
        if i is None:
            continue
        for _ in range(160):
            j = extend(second, first[0][i])
            if j is None:
                break
            if np.linalg.norm(second[0][j]-first[0][i]) <= 1e-10:
                a, b = branch(first,i), branch(second,j)
                waypoints = a + b[-2::-1]
                if side:
                    waypoints.reverse()
                samples = [waypoints[0]]
                for x,y in zip(waypoints,waypoints[1:]):
                    samples.extend(edge(x,y))
                    if len(samples) > 1000:
                        raise PlanningFailure('规划超过 1000 样本预算。')
                if not all(robot.clear(q) for q in samples):
                    raise PlanningFailure('Final collision validation failed')
                return [q.tolist() for q in samples]
    raise PlanningFailure('在当前计算预算内未找到完整无碰撞路径。')


def plan(payload, check, environment_guard=None):
    allowed = {'profile','start','base_pose','mapping','obstacles','mode','target','translation_m','rotation_deg'}
    if not isinstance(payload,dict) or set(payload)-allowed or payload.get('mode') not in ('joint','object'):
        raise ValueError('expected joint or object planning request')
    robot = ProfileRobot(payload['profile'], payload.get('start', {}), payload.get('base_pose'),
                         payload.get('mapping',default_mapping()), payload.get('obstacles',[]), check)
    robot.environment_guard = environment_guard
    start = [robot.start[n] for n in robot.names]
    details = {}
    if payload['mode'] == 'joint':
        target = positions(robot.model,payload.get('target',{}))
        if any(abs(target[n]-robot.start[n]) > 1e-10 for n in target if n not in robot.names):
            raise ValueError('规划时夹爪、轮轴及其他非主体关节必须保持固定。')
        samples = joint_path(robot,start,[target[n] for n in robot.names])
    else:
        translation = np.array(payload.get('translation_m',[0,0,0]),float)
        angles = np.array(payload.get('rotation_deg',[0,0,0]),float)
        if translation.shape != (3,) or angles.shape != (3,) or not np.isfinite(np.r_[translation,angles]).all():
            raise ValueError('位移和旋转需要有限三维向量。')
        if np.max(np.abs(translation)) > .25 or np.max(np.abs(angles)) > 45:
            raise ValueError('单次协同规划位移限 ±0.25 m，旋转限 ±45°。')
        tcps = robot.tcps(np.array(start))
        obj = np.eye(4); obj[:3,3] = (tcps[0][:3,3]+tcps[1][:3,3])/2
        target = obj.copy(); target[:3,3] += translation
        target[:3,:3] = Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
        inverse = np.linalg.inv(obj)
        planner = DualTaskPlanner(robot.tcps, robot.limits, robot.clear,
                                  TrackingSettings(max_samples=1000, max_nfev=100))
        try:
            details = planner.plan_object_motion(start,[obj,target],[inverse@t for t in tcps])
        except PlanningFailure as e:
            if robot.last_collision:
                raise PlanningFailure(str(e)+'：'+' / '.join(robot.last_collision)) from e
            raise
        samples = details.pop('positions_rad')
    check()
    frames = []
    for i,q in enumerate(samples):
        check()
        frame = pose(robot.model,{**robot.start,**dict(zip(robot.names,q))},robot.base)
        frame['time_s'] = 6*i/max(1,len(samples)-1)
        frames.append(frame)
    return {'model':robot.model,'frames':frames,'duration_s':6,'motion_type':'planned',
            'joint_names':robot.names,'positions_rad':samples,'validated_samples':len(samples),
            'collision_geometry':'conservative_oriented_boxes','collision_check':'sampled',
            'navigation_map_checked':environment_guard is not None,
            'excluded_self_pairs':[list(p) for p in sorted(robot.exclusions)],
            'scope':'fixed_base_kinematics_planning_sampled_collision','timed':False,
            'playback_duration_s':6,'hardware_validated':False,'hardware_executed':False,
            'dynamics_validated':False,'runtime_applied':False,'simulation_only':True,
            'kernel_source_commit':KERNEL_COMMIT,'gravity_limit_exceeded':[],
            'peak_holding_torque_nm':{},'peak_joint_velocity_rad_s':{},
            'limitations':['包围盒碰撞检查可能保守拒绝；使用显式碰撞几何，缺少时采用外形包围盒。',
                           '相邻两个主体关节及同一固定装配内部碰撞被结构性排除，跨臂仍检查。',
                           '离散采样；未验证连续碰撞、载荷碰撞、接触力或动力学。',
                           '六秒仅为播放速度，没有生成可执行控制时序。'],**details}


class PlanningJobs:
    def __init__(self,state):
        self.state=state; self.lock=threading.RLock(); self.job=None

    def submit(self,payload,environment_guard=None):
        epoch=self.state.check_motion()
        with self.lock:
            if self.job and self.job['status']=='running':
                raise ValueError('已有规划正在计算，请先取消。')
            job={'task_id':uuid.uuid4().hex,'status':'running','motion_epoch':epoch,
                 'result':None,'error':None,'cancel':threading.Event()}
            self.job=job
        deadline=time.monotonic()+60
        def check():
            self.state.check_motion(epoch)
            if job['cancel'].is_set():
                raise PlanningFailure('规划已取消。')
            if time.monotonic()>deadline:
                raise PlanningFailure('规划超过 60 秒计算预算。')
        def run():
            try:
                result=plan(copy.deepcopy(payload),check,environment_guard); check()
                with self.lock:
                    # Linearize completion against stop/config mutations.
                    with self.state.lock:
                        check(); result['motion_epoch']=epoch
                        job.update(status='succeeded',result=result)
            except Exception as e:
                with self.lock:
                    job.update(status='canceled' if job['cancel'].is_set() or self.state.epoch!=epoch else 'failed',
                               result=None,error=str(e))
        threading.Thread(target=run,daemon=True).start()
        return self.snapshot(job['task_id'])

    def snapshot(self,task_id):
        with self.lock:
            if not self.job or self.job['task_id']!=task_id:
                raise ValueError('Unknown planning task')
            result={k:v for k,v in self.job.items() if k!='cancel'}
            if result['status']=='succeeded':
                try:
                    self.state.check_motion(result['motion_epoch'])
                except ValueError:
                    result.update(status='canceled',result=None,error='参数或急停状态已改变，规划作废。')
            return copy.deepcopy(result)

    def cancel(self,task_id):
        with self.lock:
            if not self.job or self.job['task_id']!=task_id:
                raise ValueError('Unknown planning task')
            self.job['cancel'].set()
            self.job.update(status='canceled',result=None,error='规划已取消。')
            return self.snapshot(task_id)
