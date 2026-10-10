#!/usr/bin/env python3
"""Validate a local prepared RM75 bundle against the scalar collision oracle.

Reports source/API timings only, never installed-binary or hardware acceptance.
"""
import argparse
import json
from pathlib import Path
import statistics
import sys
import tempfile
import threading
import time
import urllib.request

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'tests')]
from robot_model_bundle import register_bundle, model
from robot_planning import ProfileRobot, plan, PlanningFailure
from robot_workbench import make_handler, ThreadingHTTPServer
from test_workbench_collision_fast import exhaustive_clear


def scalar_clear(robot, q):
    result, robot.last_collision = exhaustive_clear(robot, q)
    return result


def median_ms(function, repeats=7):
    results = []
    for _ in range(repeats):
        start = time.perf_counter()
        function()
        results.append((time.perf_counter()-start)*1000)
    return statistics.median(results)


def run(bundle, output):
    identity = register_bundle(bundle)
    report = {'scope': 'source_runtime_and_real_HTTP_API', 'bundle': identity,
              'hardware_validated': False, 'dynamics_validated': False,
              'compiled_installer_tested': False, 'variants': {}}
    fast_clear = ProfileRobot.clear
    for variant in ('b', '6fb', '6f'):
        record = model(variant)
        p, mapping, poses = record['profile'], record['mapping'], record['poses']
        start = poses['preview']
        r = ProfileRobot(p, start, None, mapping, [], lambda: None)
        rng = np.random.default_rng(20261010)
        samples = [[q[n] for n in r.names] for q in poses.values()]
        samples += rng.uniform(r.limits[:, 0], r.limits[:, 1], size=(256, 15)).tolist()
        results = []
        for q in samples:
            baseline = scalar_clear(r, q), r.last_collision
            actual = r.clear(q), r.last_collision
            if actual != baseline:
                raise AssertionError(f'{variant}: changed collision decision')
            results.append(actual[0])
        q = [start[n] for n in r.names]
        if not r.clear(q):
            raise AssertionError('preview collides: '+str(r.last_collision))
        baseline_ms = median_ms(lambda: [scalar_clear(r, q) for _ in range(100)], 5)/100
        fast_ms = median_ms(lambda: [r.clear(q) for _ in range(100)], 5)/100
        target = {**start, 'left_joint_7': start['left_joint_7']+.2}
        payload = {'profile': p, 'mapping': mapping, 'start': start,
                   'target': target, 'mode': 'joint'}
        fast_result = plan(payload, lambda: None)
        fast_plan_ms = median_ms(lambda: plan(payload, lambda: None))
        try:
            ProfileRobot.clear = scalar_clear
            old_result = plan(payload, lambda: None)
            old_plan_ms = median_ms(lambda: plan(payload, lambda: None))
        finally:
            ProfileRobot.clear = fast_clear
        if fast_result['positions_rad'] != old_result['positions_rad']:
            raise AssertionError('changed deterministic path')
        # An obstacle in the wrist must continue to block the path.
        name, local, half = next(s for s in r.shapes if s[0] == 'left_link_7')
        center = (r.world(q)[name] @ local)[:3, 3].tolist()
        try:
            plan({**payload, 'obstacles': [{'size_m': [.03]*3, 'position_m': center}]}, lambda: None)
        except PlanningFailure:
            pass
        else:
            raise AssertionError('wrist obstacle was not rejected')
        report['variants'][variant] = {
            'boxes': len(r.shapes), 'self_pairs': len(r.pairs),
            'compared_poses': len(samples), 'clear_poses': sum(results),
            'same_collision_decisions_and_first_pair': True,
            'same_joint_path': True, 'wrist_obstacle_rejected': True,
            'validated_path_samples': fast_result['validated_samples'],
            'scalar_collision_ms': baseline_ms, 'batched_collision_ms': fast_ms,
            'collision_speedup': baseline_ms/fast_ms,
            'scalar_plan_ms': old_plan_ms, 'batched_plan_ms': fast_plan_ms,
            'plan_speedup': old_plan_ms/fast_plan_ms}
    with tempfile.TemporaryDirectory() as folder:
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(folder))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        def call(path, data=None):
            request = urllib.request.Request(base+path, data=None if data is None else json.dumps(data).encode(),
                                             headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=15) as response:
                return json.load(response)
        try:
            inventory = call('/v1/workbench-models')
            assert len(inventory['models']) == 3
            record = call('/v1/workbench-model/b')
            p = record['profile']
            saved = call('/v1/workbench-profiles', p)
            assert call('/v1/workbench-profiles/'+saved['profile_id']) == p
            call('/v1/workbench-active-profile', p)
            task = call('/v1/planning-start', {'mode': 'joint', 'profile': p,
                'mapping': record['mapping'], 'start': record['poses']['preview'],
                'target': {**record['poses']['preview'], 'left_joint_7': .2}})
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                state = call('/v1/planning-task/'+task['task_id'])
                if state['status'] != 'running': break
                time.sleep(.02)
            assert state['status'] == 'succeeded', state
            call('/v1/workbench-stop', {'engaged': True})
            report['HTTP'] = {'models': 3, 'saved_and_reloaded': True,
                              'applied': True, 'joint_plan_status': state['status'],
                              'stop_engaged': True}
        finally:
            server.shutdown();thread.join(timeout=5);server.server_close()
    report['status'] = 'passed'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.bundle, args.output)
