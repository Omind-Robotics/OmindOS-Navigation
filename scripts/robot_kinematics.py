"""URDF forward kinematics and static gravity loads, in SI units.

No controller, contact solver, collision detection or hardware commands.
"""
import math
import xml.etree.ElementTree as ET

from robot_parameter_api import MAX_BYTES, number, validate_parameters
from robot_assets import approved_mesh_url


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
                item["filename"] = g.get("filename", "")
                item["scale"] = vec(g.get("scale", "1 1 1"))
                if any(x <= 0 or x > 100 for x in item["scale"]):
                    raise ValueError("mesh scale must be positive and at most 100")
                item["url"] = approved_mesh_url(item["filename"])
                if item["url"] is None:
                    warnings.append(name + ": mesh 未列入本地资源，以关节骨架显示")
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
    model = {"name": root.get("name", "robot"), "root": base, "links": list(links.values()),
             "joints": ordered, "warnings": list(dict.fromkeys(warnings)),
             "scope": "kinematics_and_static_gravity"}
    model['wheel_base'] = wheel_base_model(root, model)
    return model


def wheel_base_model(root, model):
    """Opt-in four-wheel differential reference; never infer a customer's drive type.

    Wheel radius comes from the first (tire) cylinder visual. Track comes from
    joint origins. Joint-axis sign is distinct from motor/gearbox direction.
    """
    declarations = root.findall('omindos_base')
    if not declarations:
        return None
    roles = ('left_front', 'right_front', 'left_rear', 'right_rear')
    config = declarations[0]
    if (len(declarations) != 1 or len(config) or set(config.attrib) != {'type', *roles}
            or config.get('type') != 'four_wheel_differential'):
        raise ValueError('底盘联动仅支持显式配置的四轮差速参考模型。')
    names = [config.get(role) for role in roles]
    if len(set(names)) != 4:
        raise ValueError('底盘需要四个不同的轮轴。')
    joints = {j['name']: j for j in model['joints']}
    links = {link['name']: link for link in model['links']}
    wheels = {}
    for role, name in zip(roles, names):
        j = joints.get(name)
        if j is None or j['type'] != 'continuous' or j['parent'] != model['root']:
            raise ValueError('底盘轮轴必须是直接连接根连杆的 continuous 关节。')
        axis = transform(j['origin'], j['axis'], True)
        norm = math.hypot(*axis)
        axis = [v/norm for v in axis]
        if abs(axis[0]) > 1e-7 or abs(axis[2]) > 1e-7:
            raise ValueError('四轮差速参考模型的轮轴必须平行于底盘 Y 轴。')
        visuals = links[j['child']]['visuals']
        tire = visuals[0] if visuals else None
        if tire is None or tire['type'] != 'cylinder':
            raise ValueError('轮连杆的第一个外形必须是轮胎圆柱。')
        tire_axis = transform(j['origin'], transform(tire['transform'], [0, 0, 1], True), True)
        if abs(abs(sum(a*b for a, b in zip(axis, tire_axis)))-1) > 1e-7:
            raise ValueError('轮胎圆柱必须与轮轴同轴。')
        if any(abs(tire['transform'][i][3]) > 1e-7 for i in range(3)):
            raise ValueError('轮胎圆柱中心必须位于轮轴原点。')
        y = j['origin'][1][3]
        if (role.startswith('left') and y <= 0) or (role.startswith('right') and y >= 0):
            raise ValueError('左轮须位于底盘 +Y，右轮须位于 -Y。')
        wheels[name] = {'role': role, 'radius_m': tire['radius'], 'y_m': y,
                        'axis_sign': 1 if axis[1] > 0 else -1}
    left_y = wheels[names[0]]['y_m']
    right_y = wheels[names[1]]['y_m']
    if (abs(left_y + right_y) > 1e-7 or abs(wheels[names[2]]['y_m']-left_y) > 1e-7
            or abs(wheels[names[3]]['y_m']-right_y) > 1e-7):
        raise ValueError('四轮差速参考模型要求左右对称，且同侧前后轮的 Y 坐标相同。')
    return {'type': 'four_wheel_differential', 'track_width_m': left_y-right_y,
            'wheels': wheels, 'reference_only': True}


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


def jog_trial(payload):
    if not isinstance(payload, dict) or set(payload) - {'profile', 'start', 'joint', 'direction', 'base_pose'}:
        raise ValueError('expected profile, start, joint, direction and base_pose')
    profile = payload.get('profile')
    validate_parameters(profile)
    model = build_model(profile['kinematics']['urdf_xml'])
    start = positions(model, payload.get('start', {}))
    name, direction = payload.get('joint'), payload.get('direction')
    if not isinstance(name, str) or name not in start or type(direction) is not int or direction not in (-1, 1):
        raise ValueError('choose an active joint and direction -1 or +1')
    target = {**start, name: start[name] + direction*math.radians(.1)}
    positions(model, target)
    duration = max(.15, 1.5*math.radians(.1)/profile['dynamics']['joints'][name]['velocity_limit_rad_s'])
    result = trial({'profile': profile, 'start': start, 'target': target, 'duration_s': duration,
                    'base_start': payload.get('base_pose'), 'base_target': payload.get('base_pose')})
    return {**result, 'motion_type': 'jog', 'jog_joint': name, 'step_deg': direction*.1}


def sine_trial(payload):
    if not isinstance(payload, dict) or set(payload) - {'profile', 'start', 'joints', 'amplitude_deg', 'period_s', 'cycles', 'base_pose'}:
        raise ValueError('expected profile, start, joints, amplitude_deg, period_s, cycles and base_pose')
    profile = payload.get('profile')
    validate_parameters(profile)
    model = build_model(profile['kinematics']['urdf_xml'])
    start = positions(model, payload.get('start', {}))
    selected = payload.get('joints')
    if not isinstance(selected, list) or not selected or any(not isinstance(n, str) or n not in start for n in selected) or len(set(selected)) != len(selected):
        raise ValueError('joints must be a nonempty list of unique active joints')
    amplitude = number(payload.get('amplitude_deg', 5), 'amplitude_deg', positive=True)
    period = number(payload.get('period_s', 4), 'period_s', positive=True)
    cycles = payload.get('cycles', 2)
    if amplitude > 90 or not .5 <= period <= 30 or type(cycles) is not int or not 1 <= cycles <= 10 or period*cycles > 60:
        raise ValueError('振幅应为 (0,90]°，周期 0.5–30 秒，1–10 个周期，总时长不超过 60 秒。')
    joints = {j['name']: j for j in model['joints']}
    requested = math.radians(amplitude)
    actual = {}
    for name in selected:
        j = joints[name]
        room = min(start[name]-j['lower'], j['upper']-start[name]) if j['type'] == 'revolute' else requested
        actual[name] = min(requested, max(0, room))
    if not any(a > 1e-12 for a in actual.values()):
        raise ValueError('所选关节均位于限位，无法围绕当前角度做正弦运动；请先点动离开限位。')
    drives = profile['dynamics']['joints']
    minimum_period = max(2*math.pi*a/drives[n]['velocity_limit_rad_s'] for n, a in actual.items())
    if period + 1e-10 < minimum_period:
        raise ValueError(f'period_s must be at least {minimum_period:.3f} seconds to respect velocity limits')
    base = base_position(payload.get('base_pose'))
    duration = period*cycles
    # Whole cycles have samples at extrema and exact start/end positions.
    per_cycle = max(24, min(120, math.ceil(period*30/4)*4, (360//cycles//4)*4))
    count = per_cycle*cycles + 1
    frames, peaks = [], {n: 0 for n in start}
    for i in range(count):
        t = duration*i/(count-1)
        wave = math.sin(2*math.pi*i/per_cycle)
        q = {n: q0 + actual.get(n, 0)*wave for n, q0 in start.items()}
        if i in (0, count-1):
            q = start.copy()
        frame = pose(model, q, base)
        frame['time_s'] = t
        frame['motor_positions_rad'] = {n: (q[n]-drives[n]['zero_offset_rad'])*drives[n]['reduction_ratio']/drives[n]['direction'] for n in q}
        for n, value in frame['holding_torque_nm'].items():
            peaks[n] = max(peaks[n], abs(value))
        frames.append(frame)
    return {'model': model, 'frames': frames, 'duration_s': duration, 'motion_type': 'sine',
            'selected_joints': selected, 'period_s': period, 'cycles': cycles,
            'actual_amplitude_deg': {n: math.degrees(a) for n, a in actual.items()},
            'amplitude_limited_joints': [n for n, a in actual.items() if a < requested-1e-12],
            'minimum_period_s': minimum_period, 'peak_holding_torque_nm': peaks,
            'gravity_limit_exceeded': [n for n in peaks if peaks[n] > drives[n]['torque_limit_nm']],
            'peak_joint_velocity_rad_s': {n: 2*math.pi*actual.get(n, 0)/period for n in start},
            'scope': 'joint_sine_kinematics_and_static_gravity', 'runtime_applied': False, 'hardware_validated': False,
            'limitations': ['正弦仅作用于选定旋转关节，底盘平面位姿保持不变。', '没有碰撞、接触或动力学求解；起止速度未做真机控制整形。']}


def wheel_trial(payload):
    """A finite, smoothly started/stopped wheel-speed pulse and exact SE(2) odometry.

    Differential-drive relations follow ros2_control diff_drive_controller:
    left=(v-w*track/2)/r, right=(v+w*track/2)/r. No contact/friction solver.
    """
    if not isinstance(payload, dict) or set(payload) - {'profile', 'start', 'base_pose', 'linear_m_s', 'angular_rad_s', 'duration_s'}:
        raise ValueError('expected profile, start, base_pose, linear_m_s, angular_rad_s and duration_s')
    profile = payload.get('profile')
    validate_parameters(profile)
    model = build_model(profile['kinematics']['urdf_xml'])
    chassis = model['wheel_base']
    if chassis is None:
        raise ValueError('当前模型未配置底盘轮型与四轮映射，不能试跑。')
    start = positions(model, payload.get('start', {}))
    base = base_position(payload.get('base_pose'))
    linear = number(payload.get('linear_m_s', 0), 'linear_m_s')
    angular = number(payload.get('angular_rad_s', 0), 'angular_rad_s')
    duration = number(payload.get('duration_s', 3), 'duration_s')
    if not .5 <= duration <= 10 or abs(linear) > .5 or abs(angular) > .8:
        raise ValueError('底盘预览限定 0.5–10 秒、线速度 ±0.5 m/s、角速度 ±0.8 rad/s。')
    drives = profile['dynamics']['joints']
    rates = {n: (linear-angular*w['y_m'])/w['radius_m']*w['axis_sign']
             for n, w in chassis['wheels'].items()}
    for name, rate in rates.items():
        if abs(rate) > drives[name]['velocity_limit_rad_s'] + 1e-10:
            raise ValueError(name + ': wheel velocity exceeds drive limit；请降低底盘速度。')
    # Recover body velocity from the four wheel rates, averaging each side.
    side_speed = {side: sum(rates[n]*w['radius_m']*w['axis_sign']
                           for n, w in chassis['wheels'].items() if w['role'].startswith(side))/2
                  for side in ('left', 'right')}
    v = (side_speed['right']+side_speed['left'])/2
    omega = (side_speed['right']-side_speed['left'])/chassis['track_width_m']
    count = math.ceil(duration*30/2)*2 + 1  # include the exact peak-speed sample
    frames, peaks = [], {n: 0.0 for n in start}
    for i in range(count):
        u = i/(count-1)
        gain = (1-math.cos(2*math.pi*u))/2
        elapsed = duration*(u/2-math.sin(2*math.pi*u)/(4*math.pi))
        if i in (0, count-1):
            gain, elapsed = 0.0, duration*u/2
        q = {n: q0+rates.get(n, 0)*elapsed for n, q0 in start.items()}
        theta = omega*elapsed
        half = theta/2
        length = v*elapsed*(math.sin(half)/half if abs(half) > 1e-10 else 1.0)
        current_base = {'x_m': base['x_m']+length*math.cos(base['yaw_rad']+half),
                        'y_m': base['y_m']+length*math.sin(base['yaw_rad']+half),
                        'yaw_rad': (base['yaw_rad']+theta+math.pi) % (2*math.pi)-math.pi}
        if i == 0:
            current_base = base.copy()
        frame = pose(model, q, current_base)
        frame.update(time_s=duration*u, wheel_velocity_rad_s={n: rate*gain for n, rate in rates.items()},
                     base_velocity={'linear_m_s': v*gain, 'angular_rad_s': omega*gain},
                     motor_positions_rad={n: (q[n]-drives[n]['zero_offset_rad'])*drives[n]['reduction_ratio']/drives[n]['direction'] for n in q})
        for name, torque in frame['holding_torque_nm'].items():
            peaks[name] = max(peaks[name], abs(torque))
        frames.append(frame)
    return {'model': model, 'frames': frames, 'duration_s': duration, 'motion_type': 'wheel_base',
            'peak_joint_velocity_rad_s': {n: abs(rates.get(n, 0)) for n in start},
            'peak_base_velocity': {'linear_m_s': abs(v), 'angular_rad_s': abs(omega)},
            'peak_holding_torque_nm': peaks,
            'gravity_limit_exceeded': [n for n in peaks if peaks[n] > drives[n]['torque_limit_nm']],
            'scope': 'four_wheel_differential_reference_kinematics',
            'runtime_applied': False, 'hardware_validated': False,
            'limitations': ['四轮差速参考模型；不代表实机轮型，无侧移。',
                            '不计算侧向滑移、轮地接触、牵引力、碰撞、平衡或导航。']}

