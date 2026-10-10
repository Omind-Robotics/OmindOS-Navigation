"""Pure joint-coordinate previews for explicit kinematic profiles.

No motor-space mapping, mass assumptions, torque calculation, dynamics or
hardware transport. The existing workbench route owns stop/epoch protection.
"""
import math
import xml.etree.ElementTree as ET

from robot_kinematics import build_model, pose, positions, base_position
from robot_parameter_api import number
from workbench_state import _validate_workbench_profile, KINEMATIC_MODE


def setup(payload, allowed):
    if not isinstance(payload, dict) or set(payload)-set(allowed):
        raise ValueError('unexpected kinematic preview fields')
    profile=payload.get('profile')
    if not isinstance(profile,dict) or profile.get('profile_mode')!=KINEMATIC_MODE:
        raise ValueError('explicit kinematic_preview profile is required')
    _validate_workbench_profile(profile)
    model=build_model(profile['kinematics']['urdf_xml'])
    model['scope']='kinematic_preview_no_dynamics'
    root=ET.fromstring(profile['kinematics']['urdf_xml']);speeds={}
    for joint in root.findall('joint'):
        if joint.get('type')=='fixed':continue
        limit=joint.find('limit')
        if limit is None or 'velocity' not in limit.attrib:
            raise ValueError(joint.get('name')+': explicitly set a positive URDF preview velocity limit')
        speeds[joint.get('name')]=number(limit.get('velocity'),'preview joint velocity',positive=True)
    return model,positions(model,payload.get('start',{})),speeds


def frame(model,q,base,t):
    out=pose(model,q,base,calculate_gravity=False)
    out.update(time_s=t,holding_torque_nm=None,motor_positions_rad=None,centers_of_mass={})
    return out


def result(model,frames,duration,**details):
    return {'model':model,'frames':frames,'duration_s':duration,
            'scope':'fixed_base_kinematic_preview_no_dynamics',
            'peak_holding_torque_nm':{},'gravity_limit_exceeded':[],
            'motor_mapping_evaluated':False,'gravity_evaluated':False,
            'collision_validated':False,'dynamics_validated':False,
            'hardware_validated':False,'runtime_applied':False,
            'limitations':['仅按URDF关节坐标与速度上限播放，未输出电机目标。',
                           '普通点动/正弦未验收碰撞、载荷、动力学或真机停止。'],**details}


def trial(payload):
    model,start,speeds=setup(payload,{'profile','start','target','duration_s','base_start','base_target'})
    target=positions(model,payload.get('target',{}))
    base=base_position(payload.get('base_start'));end=base_position(payload.get('base_target'))
    if base!=end:
        raise ValueError('kinematic arm preview requires a stationary base; navigate and park separately')
    minimum=max((1.5*abs(target[n]-start[n])/speeds[n]for n in start),default=0)
    duration=number(payload.get('duration_s',max(2.,minimum)),'duration_s',positive=True)
    if duration>60 or duration+1e-10<minimum:
        raise ValueError(f'duration_s must be between {minimum:.3f} and 60 seconds')
    count=min(181,max(2,math.ceil(duration*30)+1));frames=[]
    for i in range(count):
        u=i/(count-1);s=u*u*(3-2*u)
        q={n:start[n]+s*(target[n]-start[n])for n in start}
        if i==0:q=start.copy()
        if i==count-1:q=target.copy()
        frames.append(frame(model,q,base,u*duration))
    return result(model,frames,duration,minimum_duration_s=minimum,
                  peak_joint_velocity_rad_s={n:1.5*abs(target[n]-start[n])/duration for n in start},
                  peak_base_velocity={'linear_m_s':0.,'angular_rad_s':0.})


def jog(payload):
    model,start,speeds=setup(payload,{'profile','start','joint','direction','base_pose'})
    name,direction=payload.get('joint'),payload.get('direction')
    if not isinstance(name,str) or name not in start or type(direction)is not int or direction not in(-1,1):
        raise ValueError('choose an active joint and direction -1 or 1')
    target={**start,name:start[name]+direction*math.radians(.1)}
    positions(model,target)
    duration=max(.15,1.5*math.radians(.1)/speeds[name])
    out=trial({'profile':payload['profile'],'start':start,'target':target,'duration_s':duration,
               'base_start':payload.get('base_pose'),'base_target':payload.get('base_pose')})
    out.update(motion_type='jog',jog_joint=name,step_deg=.1*direction)
    return out


def sine(payload):
    model,start,speeds=setup(payload,{'profile','start','joints','amplitude_deg','period_s','cycles','base_pose'})
    selected=payload.get('joints')
    if not isinstance(selected,list) or not selected or any(not isinstance(n,str)or n not in start for n in selected)or len(set(selected))!=len(selected):
        raise ValueError('select unique active joint names')
    amplitude=number(payload.get('amplitude_deg',5),'amplitude_deg',positive=True)
    period=number(payload.get('period_s',4),'period_s',positive=True);cycles=payload.get('cycles',2)
    if amplitude>90 or not .5<=period<=30 or type(cycles)is not int or not 1<=cycles<=10 or period*cycles>60:
        raise ValueError('invalid sine amplitude, period, cycles or total duration')
    joints={j['name']:j for j in model['joints']};actual={}
    for n in selected:
        j=joints[n];room=min(start[n]-j['lower'],j['upper']-start[n])if j['type']=='revolute'else math.radians(amplitude)
        actual[n]=min(math.radians(amplitude),max(0,room))
    if not any(a>1e-12 for a in actual.values()):raise ValueError('selected joints are at their limits')
    minimum=max(2*math.pi*a/speeds[n]for n,a in actual.items())
    if period+1e-10<minimum:raise ValueError(f'period_s must be at least {minimum:.3f} seconds')
    per=max(24,min(120,math.ceil(period*30/4)*4,(360//cycles//4)*4));count=per*cycles+1
    base=base_position(payload.get('base_pose'));duration=period*cycles;frames=[]
    for i in range(count):
        wave=math.sin(2*math.pi*i/per);q={n:q0+actual.get(n,0)*wave for n,q0 in start.items()}
        if i in(0,count-1):q=start.copy()
        frames.append(frame(model,q,base,duration*i/(count-1)))
    return result(model,frames,duration,motion_type='sine',selected_joints=selected,period_s=period,cycles=cycles,
                  actual_amplitude_deg={n:math.degrees(a)for n,a in actual.items()},minimum_period_s=minimum,
                  amplitude_limited_joints=[n for n,a in actual.items()if a+1e-12<math.radians(amplitude)],
                  peak_joint_velocity_rad_s={n:2*math.pi*actual.get(n,0)/period for n in start})


def motion(path,payload):
    functions={'/v1/robot-preview':trial,'/v1/robot-jog':jog,'/v1/robot-sine':sine}
    if path not in functions:
        raise ValueError('该运动学整机配置不执行旧参考底盘/编舞，请使用关节预览或独立导航规划。')
    return functions[path](payload)
