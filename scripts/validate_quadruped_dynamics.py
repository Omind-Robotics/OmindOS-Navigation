#!/usr/bin/env python3
"""Closed-loop contact dynamics validation; never opens a CAN interface."""
import argparse
import json
import math
from pathlib import Path

from quadruped_controller import Controller, PinnedPolicy, COMMIT, POLICY_SHA256, NAMES, LIMITS
from quadruped_backends import MujocoBackend, SimulatedPositionBackend
from joint_interface import PositionCommand


def scenario(source, policy, name, command, push=False, control_mode='joint-position'):
    sim = MujocoBackend(source)
    controller = Controller(policy, sim.effort_limits)
    dt, now = .005, 0.
    adapter = None
    sequence = 0
    if control_mode=='joint-position':
        adapter=SimulatedPositionBackend(sim,NAMES,[(lo-.08,hi+.08) for lo,hi in LIMITS],
                                         [50.]*12,[2.]*12,clock=lambda:now)
        adapter.arm()
    controller.action('arm', sim.sensors(now), now)
    trace, phase_stats = [], []
    phases = [('standby',1.), ('standup',3.1), ('hold',3.), ('move',6.), ('stop',5.)]
    for phase, duration in phases:
        if phase == 'standup': controller.action('stand', sim.sensors(now), now)
        if phase == 'move': controller.action('walk', sim.sensors(now), now)
        if phase == 'stop': controller.action('stop', sim.sensors(now), now)
        start = sim.telemetry()['base_xyz']
        start_quat = sim.sensors(now).quaternion_xyzw
        min_height, min_up, max_torque, speeds = math.inf, math.inf, 0., []
        for k in range(round(duration/dt)):
            if phase == 'move':
                controller.velocity(command, now)
            sim.data.xfrc_applied[sim.base_id,:] = 0
            if push and phase == 'move' and 400 <= k < 440:
                sim.data.xfrc_applied[sim.base_id,1] = 40.  # 8 N.s lateral impulse
            if adapter:
                position=controller.tick_targets(sim.sensors(now),now)
                if position is None: raise RuntimeError(controller.reason)
                adapter.apply_positions(PositionCommand(adapter.command_session,sequence,now,.05,
                                                        tuple(NAMES),tuple(position)),now)
                sequence+=1
                effort=adapter.last_effort
            else:
                effort = controller.tick(sim.sensors(now), now)
                sim.apply(effort)
            now += dt
            if controller.state == 'FAULT':
                raise RuntimeError(f'{name}/{phase}: {controller.reason}')
            status = sim.telemetry()
            min_height = min(min_height,status['base_xyz'][2])
            min_up = min(min_up,status['upright'])
            max_torque = max(max_torque,max(abs(x) for x in effort))
            speeds.append(math.hypot(*status['body_velocity'][:2]))
            if k % 20 == 0:
                trace.append({'t':round(now,3),'phase':phase, 'xyz':status['base_xyz'],
                              'upright':status['upright'], 'speed':speeds[-1]})
        quat = sim.sensors(now).quaternion_xyzw
        yaw = lambda q: math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
        phase_stats.append({'phase':phase, 'minimum_height_m':min_height, 'minimum_upright':min_up,
            'max_effort_nm':max_torque, 'displacement':[a-b for a,b in zip(status['base_xyz'],start)],
            'yaw_change_rad':math.atan2(math.sin(yaw(quat)-yaw(start_quat)),math.cos(yaw(quat)-yaw(start_quat))),
            'final_second_mean_speed':sum(speeds[-200:])/200})
    checks = {'upright':all(s['minimum_upright'] > .8 for s in phase_stats[2:]),
              'clearance':all(s['minimum_height_m'] > .2 for s in phase_stats[2:]),
              'stop_speed':phase_stats[-1]['final_second_mean_speed'] < .15,
              'policy_executed':controller.inferences > 500}
    move = phase_stats[3]
    if name == 'forward': checks['forward_progress'] = move['displacement'][0] > .3
    if name == 'lateral': checks['lateral_progress'] = move['displacement'][1] > .1
    if name == 'turn': checks['turn_progress'] = move['yaw_change_rad'] > .3
    if adapter: adapter.close()
    else: sim.close()
    return {'name':name,'checks':checks,'phases':phase_stats,'trace':trace,'position_commands':sequence,
            'push_force_n':40 if push else 0, 'push_duration_s':.2 if push else 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--control-mode',choices=['joint-position','host-effort-test'],default='joint-position')
    args = parser.parse_args()
    policy = PinnedPolicy(args.source)
    results = [scenario(policy.source, policy, name, command, push,args.control_mode) for name,command,push in
               [('forward',[.2,0,0],False), ('lateral',[0,.1,0],False),
                ('turn',[0,0,.3],False), ('standing_push',[0,0,0],True)]]
    import torch, mujoco, numpy
    passed = all(all(r['checks'].values()) for r in results)
    report = {'status':'PASS' if passed else 'FAIL', 'hardware_tested':False,
              'customer_RS04_dynamics_tested':False, 'upstream_commit':COMMIT,
              'policy_sha256':POLICY_SHA256, 'physics_hz':200, 'policy_hz':50,
              'control_mode':args.control_mode,'joint_loop':'simulated_position_pd' if args.control_mode=='joint-position' else 'host_pd',
              'versions':{'torch':torch.__version__,'mujoco':mujoco.__version__,'numpy':numpy.__version__},
              'scenarios':results}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({**report,'scenarios':[{k:v for k,v in r.items() if k!='trace'} for r in results]},indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
