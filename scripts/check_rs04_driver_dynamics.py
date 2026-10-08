#!/usr/bin/env python3
"""Drive MuJoCo through real RS04 command/feedback codecs and driver state.

No physical devices. Mass/inertia cases are sensitivity assumptions, not a
verified RS04 CAD model. Every failure is recorded instead of omitted.
"""
import argparse
import json
import math
from pathlib import Path

from quadruped_controller import Controller, PinnedPolicy, COMMIT, POLICY_SHA256, simulation_control_profile
from quadruped_backends import MujocoBackend
from rs04_simulated_bus import DriverInLoop, modify_mass_sensitivity

CASES=[('reference',0.,'proximal',1.),
       ('rs04_nominal_proximal',.54,'proximal',1.),
       ('rs04_high_mass_proximal',.58,'proximal',2.),
       ('rs04_distal_stress',.58,'distal_stress',2.)]
MOVEMENTS=[('forward',[.2,0.,0.],False),('lateral',[0.,.1,0.],False),
           ('turn',[0.,0.,.3],False),('push',[0.,0.,0.],True)]


def run_case(source,policy,case,movement,kd=2.,kp=50.):
    case_name,delta,distribution,armature=case
    name,command,push=movement
    sim=MujocoBackend(source)
    assumptions=modify_mass_sensitivity(sim,delta,distribution,armature)
    loop=DriverInLoop(sim)
    controller=Controller(policy,sim.effort_limits,kp=[kp]*12,kd=[kd]*12)
    phases=[]; trace=[]; failure=None
    try:
        controller.action('arm',loop.sensors(),loop.clock())
        for phase,duration in [('standby',1.),('standup',3.1),('hold',3.),('move',6.),('stop',5.)]:
            if phase=='standup': controller.action('stand',loop.sensors(),loop.clock())
            if phase=='move': controller.action('walk',loop.sensors(),loop.clock())
            if phase=='stop': controller.action('stop',loop.sensors(),loop.clock())
            start=sim.telemetry()['base_xyz']
            q=sim.sensors(loop.clock()).quaternion_xyzw
            yaw=lambda q:math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
            yaw_start=yaw(q)
            heights=[]; uprights=[]; speeds=[]; peak_effort=0.; torque_square=[0.]*12
            for i in range(round(duration/.005)):
                now=loop.clock()
                if phase=='move': controller.velocity(command,now)
                sim.data.xfrc_applied[sim.base_id,:]=0.
                if push and phase=='move' and 400<=i<440: sim.data.xfrc_applied[sim.base_id,1]=40.
                effort=controller.tick(loop.sensors(),now)
                if controller.state=='FAULT': raise RuntimeError(controller.reason)
                loop.step(effort)
                state=sim.telemetry(); heights.append(state['base_xyz'][2]); uprights.append(state['upright'])
                speeds.append(math.hypot(*state['body_velocity'][:2]))
                peak_effort=max(peak_effort,max(abs(x) for x in effort))
                torque_square=[a+b*b for a,b in zip(torque_square,effort)]
                if i%40==0:
                    trace.append({'time':round(loop.clock(),3),'phase':phase,'xyz':state['base_xyz'],
                                  'upright':state['upright'],'speed':speeds[-1]})
            yaw_end=yaw(sim.sensors(loop.clock()).quaternion_xyzw)
            phases.append({'phase':phase,'displacement_m':[a-b for a,b in zip(state['base_xyz'],start)],
                'yaw_change_rad':math.atan2(math.sin(yaw_end-yaw_start),math.cos(yaw_end-yaw_start)),
                'minimum_height_m':min(heights),'minimum_upright':min(uprights),
                'maximum_command_effort_nm':peak_effort,
                'maximum_joint_RMS_command_effort_nm':max(math.sqrt(x/len(heights)) for x in torque_square),
                'final_second_mean_speed_m_s':sum(speeds[-200:])/len(speeds[-200:])})
    except Exception as error:
        failure=str(error)
    finally:
        loop.close()
    completed=len(phases)==5
    checks={'completed_without_driver_or_controller_fault':completed and failure is None,
            'velocity_within_RS04_encoding_range':loop.bus.raw_peak_velocity<=15.,
            'all_motors_stopped_at_end':len(loop.bus.stopped_addresses)==12 and not any(loop.bus.mode.values())}
    if completed:
        checks.update({'upright':all(p['minimum_upright']>.8 for p in phases[2:]),
                       'clearance':all(p['minimum_height_m']>.2 for p in phases[2:]),
                       'stop_speed':phases[-1]['final_second_mean_speed_m_s']<.15})
        if name=='forward': checks['forward_progress']=phases[3]['displacement_m'][0]>.3
        if name=='lateral': checks['lateral_progress']=phases[3]['displacement_m'][1]>.1
        if name=='turn': checks['turn_progress']=phases[3]['yaw_change_rad']>.3
    return {'case':case_name,'movement':name,'status':'PASS' if all(checks.values()) else 'FAIL',
            'host_kp':kp,'host_kd':kd,
            'checks':checks,'error':failure,'assumptions':assumptions,'phases':phases,'trace':trace,
            'command_frames':loop.bus.frames_sent,'feedback_frames':loop.bus.frames_feedback,
            'maximum_unquantized_motor_velocity_rad_s':loop.bus.raw_peak_velocity,
            'simulation_seconds':loop.clock()}


def fault_cases(source):
    results=[]
    for name in ('feedback_loss','uncalibrated_fault','host_freeze_watchdog','one_bus_stop_failure'):
        sim=MujocoBackend(source); loop=DriverInLoop(sim)
        # Use measured position hold to avoid exciting the folded robot.
        reason=None; start=loop.clock(); key=next(iter(loop.bus.joints))
        if name=='feedback_loss': loop.bus.drop_feedback.add(key)
        if name=='uncalibrated_fault': loop.bus.faults[key]=32
        if name=='one_bus_stop_failure': loop.bus.stop_failures.add(key)
        if name=='host_freeze_watchdog':
            for _ in range(25): loop.bus.advance()
            passed=loop.bus.watchdog_trips==12 and not any(loop.bus.mode.values())
        elif name=='one_bus_stop_failure':
            loop.bus.stopped_addresses.clear()
            with_error=loop.driver._send_all_stop()
            for _ in range(25): loop.bus.advance()
            passed=len(with_error)==1 and len(loop.bus.stopped_addresses)==11 and not any(loop.bus.mode.values())
        else:
            try:
                for _ in range(30):
                    loop.step([0.]*12)
            except RuntimeError as error:
                reason=str(error)
            passed=bool(reason) and not loop.driver.active and not any(loop.bus.mode.values())
        elapsed=loop.clock()-start
        loop.bus.stop_failures.clear(); loop.close()
        results.append({'case':name,'passed':passed,'reason':reason,
                        'simulated_elapsed_s':elapsed,'firmware_watchdog_trips':loop.bus.watchdog_trips})
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--control-profile',type=Path,required=True)
    args=parser.parse_args(); policy=PinnedPolicy(args.source)
    gains=simulation_control_profile(args.control_profile)
    if len(set(gains['kp']))!=1 or len(set(gains['kd']))!=1:
        raise ValueError('this sensitivity matrix expects uniform gains; use per-joint test for nonuniform gains')
    results=[run_case(policy.source,policy,case,movement,
                     kd=2. if case[0]=='reference' else gains['kd'][0],
                     kp=50. if case[0]=='reference' else gains['kp'][0]) for case in CASES for movement in MOVEMENTS]
    faults=fault_cases(policy.source)
    passed=all(r['status']=='PASS' for r in results) and all(f['passed'] for f in faults)
    report={'status':'PASS' if passed else 'FAIL','hardware_tested':False,'customer_model_validated':False,
            'transport':'in_process_RS04_CAN_frame_emulator','upstream_commit':COMMIT,'policy_sha256':POLICY_SHA256,
            'candidate_control_profile_sha256':gains['sha256'],
            'mass_sources_kg':{'RS03':.880,'RS04':1.420,'each_tolerance':.020},
            'not_modeled':['actual_RS04_CAD_inertia','mounting_changes','motor_thermal_dynamics',
                           'USB_or_CAN_latency','torque_speed_curve','independent_hardware_emergency_stop'],
            'total_simulation_seconds':sum(r['simulation_seconds'] for r in results),
            'scenarios':results,'fault_tests':faults}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({**report,'scenarios':[{k:v for k,v in r.items() if k not in ('trace','phases')}
                                         for r in results]},indent=2))
    return 0 if passed else 1


if __name__=='__main__': raise SystemExit(main())
