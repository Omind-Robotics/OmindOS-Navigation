import json
import math
from pathlib import Path
import tempfile
import unittest

from nav3d_native.platform_guard import (
    PlatformGuard, TIMEOUTS, ZERO, load_profile, stamp_is_fresh)


class TestPlatformGuard(unittest.TestCase):
    def ready(self, profile='wheeled_diff', command=(0.2, 0, 0, 0, 0, 0.1)):
        guard = PlatformGuard(load_profile(profile), enabled=True)
        guard.receive_command(command, 10.0)
        for source in ('pose', 'odom', 'plan'):
            guard.observe(source, True, 10.0)
        return guard

    def test_all_platforms_use_same_navigation_contract(self):
        for name in ('wheeled_diff', 'wheeled_omni', 'quadruped'):
            with self.subTest(name=name):
                status, command = self.ready(name).evaluate(10.1)
                self.assertEqual(status, 'READY')
                self.assertEqual(command, (0.2, 0, 0, 0, 0, 0.1))

    def test_disabled_and_incomplete_startup_never_move(self):
        guard = PlatformGuard(load_profile('quadruped'))
        self.assertEqual(guard.evaluate(10), ('DISARMED', ZERO))
        guard.enabled = True
        self.assertEqual(guard.evaluate(10), ('COMMAND_MISSING', ZERO))

    def test_every_missing_invalid_and_stale_source_stops(self):
        for source, timeout in TIMEOUTS.items():
            for failure in ('missing', 'invalid', 'stale'):
                with self.subTest(source=source, failure=failure):
                    guard = self.ready()
                    if failure == 'missing':
                        del guard.receipts[source]
                    elif failure == 'invalid':
                        guard.valid[source] = False
                    else:
                        guard.receipts[source] = 10.0 - timeout - 0.01
                    status, command = guard.evaluate(10)
                    self.assertEqual(status, source.upper() + '_' + (
                        'TIMEOUT' if failure == 'stale' else failure.upper()))
                    self.assertEqual(command, ZERO)

    def test_lateral_is_platform_specific(self):
        for name in ('wheeled_diff', 'wheeled_omni', 'quadruped'):
            with self.subTest(name=name):
                status, command = self.ready(name, (0, 0.1, 0, 0, 0, 0)).evaluate(10)
                self.assertEqual(command[1], 0 if name == 'wheeled_diff' else 0.1)
                self.assertEqual(status, 'COMMAND_INVALID' if name == 'wheeled_diff' else 'READY')

    def test_limits_preserve_command_ratios(self):
        guard = self.ready('quadruped', (0.9, 0.4, 0, 0, 0, 2.0))
        self.assertEqual(guard.evaluate(10), ('LIMITED', (0.45, 0.2, 0, 0, 0, 1.0)))

    def test_invalid_and_unsupported_commands_stop(self):
        for value in (math.nan, math.inf, -math.inf):
            for axis in range(6):
                with self.subTest(value=value, axis=axis):
                    values = [0.0] * 6
                    values[axis] = value
                    self.assertEqual(self.ready(command=values).evaluate(10),
                                     ('COMMAND_INVALID', ZERO))
        for axis in (2, 3, 4):
            values = [0.0] * 6
            values[axis] = 0.2
            self.assertEqual(self.ready('quadruped', values).evaluate(10),
                             ('COMMAND_INVALID', ZERO))
        for values in ([], [1] * 7, ['bad'] * 6, None):
            self.assertEqual(self.ready(command=values).evaluate(10),
                             ('COMMAND_INVALID', ZERO))

    def test_estop_latches_across_false_and_new_commands(self):
        guard = self.ready()
        guard.emergency_stop(True)
        guard.emergency_stop(False)
        guard.receive_command((0.2, 0, 0, 0, 0, 0), 10.1)
        self.assertEqual(guard.evaluate(10.1), ('EMERGENCY_STOP', ZERO))

    def test_stale_sources_can_recover_with_new_data(self):
        guard = self.ready()
        self.assertEqual(guard.evaluate(20)[1], ZERO)
        guard.receive_command((0.2, 0, 0, 0, 0, 0), 20)
        for source in ('pose', 'odom', 'plan'):
            guard.observe(source, True, 20)
        self.assertEqual(guard.evaluate(20)[0], 'READY')

    def test_clock_and_message_stamps_fail_closed(self):
        for now in (9.0, math.inf, math.nan):
            self.assertEqual(self.ready().evaluate(now)[1], ZERO)
        for stamp in (0, -1, math.inf, math.nan, 12, 1):
            self.assertFalse(stamp_is_fresh(stamp, 10, 0.5))
        self.assertTrue(stamp_is_fresh(9.9, 10, 0.5))

    def test_profile_schema_and_limits(self):
        with self.assertRaises(ValueError):
            load_profile('go2_not_validated')
        for change in ({'max_vx': -1}, {'max_vy': 1}, {'max_wz': math.nan},
                       {'max_vx': True}, {'kind': 'unknown'}):
            data = {'kind': 'differential', 'max_vx': 0.5, 'max_vy': 0, 'max_wz': 1}
            data.update(change)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'profile.json'
                path.write_text(json.dumps({'schema_version': 1, 'profiles': {'test': data}}))
                with self.assertRaises(ValueError):
                    load_profile('test', path)
