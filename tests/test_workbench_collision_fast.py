"""Compare batched rejection to the previous exhaustive OBB implementation."""
import unittest
import numpy as np
from test_workbench_rm75_integration import profile, robot, FIXTURE
from robot_planning import overlap


def exhaustive_clear(r, q):
    world = r.world(q)
    boxes = []
    for name, local, half in r.shapes:
        transform = world[name] @ local
        boxes.append((transform[:3, 3], transform[:3, :3], half))
    for i, k in r.pairs:
        if overlap(boxes[i], boxes[k]):
            return False, [r.shapes[i][0], r.shapes[k][0]]
    for i, box in enumerate(boxes):
        for k, obstacle in enumerate(r.obstacles):
            if overlap(box, obstacle):
                return False, [r.shapes[i][0], f'obstacle_{k+1}']
    return True, None


class FastCollisionTests(unittest.TestCase):
    def assert_same(self, r, q):
        expected, collision = exhaustive_clear(r, q)
        self.assertEqual(r.clear(q), expected)
        self.assertEqual(r.last_collision, collision)

    def test_actual_three_variants_and_random_poses_match_exhaustive(self):
        rng = np.random.default_rng(20261010)
        for variant in ('b', '6fb', '6f'):
            r = robot(profile(variant))
            qs = [[q[n] for n in r.names] for q in FIXTURE['poses'].values()]
            qs.extend(rng.uniform(r.limits[:, 0], r.limits[:, 1], size=(128, 15)))
            for q in qs:
                self.assert_same(r, q)

    def test_thin_obstacles_and_near_touching_boundaries_match(self):
        start = FIXTURE['poses']['preview']
        r = robot(start=start)
        q = [start[n] for n in r.names]
        name, local, half = next(s for s in r.shapes if s[0] == 'left_link_7')
        t = r.world(q)[name] @ local
        extents = np.abs(t[:3, :3]) @ half
        for gap in (-1e-5, 0, 0.5e-10, 1e-10, 2e-10, 1e-5):
            center = t[:3, 3].copy()
            center[0] += extents[0] + .001 + gap
            obstacles = [{'size_m': [.002, 2, 2], 'position_m': center.tolist()}]
            self.assert_same(robot(start=start, obstacles=obstacles), q)

    def test_full_obstacle_budget_and_base_changes(self):
        rng = np.random.default_rng(17)
        obstacles = [{'size_m': [.025, .015, .01], 'position_m': p.tolist()}
                     for p in rng.uniform(-1.5, 1.5, size=(32, 3))]
        r = robot(obstacles=obstacles)
        for _ in range(64):
            r.base = dict(x_m=float(rng.uniform(-1, 1)), y_m=.2, yaw_rad=.7)
            r.cache = None
            self.assert_same(r, rng.uniform(r.limits[:, 0], r.limits[:, 1]))

    def test_navigation_guard_sees_every_original_box(self):
        r = robot()
        q = [FIXTURE['poses']['preview'][n] for n in r.names]
        received = []
        r.environment_guard = lambda boxes: received.append(boxes) or False
        self.assertFalse(r.clear(q))
        self.assertEqual(len(received[0]), len(r.shapes))
        self.assertEqual(r.last_collision, ['robot', 'navigation_map'])

    def test_cancel_is_checked_even_after_all_pairs_are_rejected(self):
        r = robot()
        class Canceled(Exception):
            pass
        calls = []
        def check():
            calls.append(1)
            if len(calls) == 2:
                raise Canceled()
        r.check = check
        with self.assertRaises(Canceled):
            r.clear([FIXTURE['poses']['preview'][n] for n in r.names])


if __name__ == '__main__':
    unittest.main()
