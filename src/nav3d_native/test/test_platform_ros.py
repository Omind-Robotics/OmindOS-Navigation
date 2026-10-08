"""Real ROS topic checks, isolated to the preview namespace; no robot connection."""
import copy
import importlib.util
import math
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 runtime required')
class TestPlatformRos(unittest.TestCase):
    def setUp(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry, Path
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import Bool
        from nav3d_native.platform_adapter_node import PlatformAdapter, ROOT, PATH_QOS
        rclpy.init(args=['--ros-args', '-p', 'enable_preview_motion:=true'])
        self.rclpy = rclpy
        self.adapter = PlatformAdapter()
        self.source = Node('platform_test_source')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.adapter)
        self.executor.add_node(self.source)
        self.types = (PoseStamped, Twist, Odometry, Path, Bool)
        self.pubs = {
            'command': self.source.create_publisher(Twist, ROOT + '/navigation/cmd_vel', 1),
            'pose': self.source.create_publisher(PoseStamped, ROOT + '/localization/pose', 1),
            'odom': self.source.create_publisher(Odometry, ROOT + '/odom', 1),
            'plan': self.source.create_publisher(Path, ROOT + '/plan', PATH_QOS),
            'estop': self.source.create_publisher(Bool, ROOT + '/emergency_stop', 1),
        }
        self.received = []
        self.source.create_subscription(Twist, ROOT + '/robot/cmd_vel',
                                        self.received.append, 10)

    def tearDown(self):
        self.executor.shutdown()
        self.adapter.destroy_node()
        self.source.destroy_node()
        self.rclpy.shutdown()

    def exercise(self, seconds=0.5, omit=None, malformed=None):
        PoseStamped, Twist, Odometry, Path, _ = self.types
        deadline = time.monotonic() + seconds
        next_publish = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                stamp = self.source.get_clock().now().to_msg()
                pose = PoseStamped()
                pose.header.stamp, pose.header.frame_id = stamp, 'map'
                pose.pose.orientation.w = 1.0
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = stamp, 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.orientation.w = 1.0
                path = Path()
                path.header = copy.deepcopy(pose.header)
                path.poses = [copy.deepcopy(pose)]
                command = Twist()
                command.linear.x = 0.2
                messages = {'command': command, 'pose': pose, 'odom': odom, 'plan': path}
                if malformed == 'nan':
                    command.linear.x = math.nan
                elif malformed == 'empty_path':
                    path.poses = []
                elif malformed == 'old_pose':
                    # Use a separate header; the path still has a fresh source stamp.
                    pose.header.stamp.sec -= 10
                elif malformed == 'wrong_frame':
                    odom.child_frame_id = 'camera'
                for source, msg in messages.items():
                    if source != omit:
                        self.pubs[source].publish(msg)
                next_publish = time.monotonic() + 0.05
            self.executor.spin_once(timeout_sec=0.005)

    def assert_stopped(self):
        self.assertGreaterEqual(len(self.received), 3)
        for msg in self.received[-3:]:
            self.assertEqual((msg.linear.x, msg.linear.y, msg.angular.z), (0, 0, 0))

    def test_source_loss_recovery_and_bad_messages(self):
        self.exercise(0.8)
        self.assertGreater(self.received[-1].linear.x, 0)
        for source, wait in (('command', 0.5), ('pose', 1.0), ('odom', 0.8), ('plan', 2.3)):
            with self.subTest(source=source):
                self.exercise(wait, omit=source)
                self.assert_stopped()
                self.exercise()
                self.assertGreater(self.received[-1].linear.x, 0)
        for malformed in ('nan', 'empty_path', 'old_pose', 'wrong_frame'):
            with self.subTest(malformed=malformed):
                self.exercise(malformed=malformed)
                self.assert_stopped()
                self.exercise()
                self.assertGreater(self.received[-1].linear.x, 0)

    def test_emergency_stop_does_not_reset_from_false(self):
        self.exercise(0.8)
        self.assertGreater(self.received[-1].linear.x, 0)
        Bool = self.types[-1]
        msg = Bool()
        msg.data = True
        self.pubs['estop'].publish(msg)
        self.exercise()
        self.assert_stopped()
        msg.data = False
        self.pubs['estop'].publish(msg)
        self.exercise()
        self.assert_stopped()


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 runtime required')
class TestSimulatorWatchdog(unittest.TestCase):
    def scenario(self, lateral_enabled):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from nav3d_native.simulator_node import NativeSimulator
        rclpy.init(args=['--ros-args', '-p', 'mapping_phase_seconds:=0.0',
                        '-p', f'lateral_velocity_enabled:={str(lateral_enabled).lower()}'])
        simulator = NativeSimulator()
        source = Node('simulator_watchdog_test')
        publisher = source.create_publisher(Twist, '/cmd_vel', 1)
        executor = SingleThreadedExecutor()
        executor.add_node(simulator)
        executor.add_node(source)

        def run(seconds, publish=False):
            deadline = time.monotonic() + seconds
            next_publish = 0
            while time.monotonic() < deadline:
                if publish and time.monotonic() >= next_publish:
                    msg = Twist()
                    msg.linear.x, msg.linear.y = 0.15, 0.2
                    publisher.publish(msg)
                    next_publish = time.monotonic() + 0.05
                executor.spin_once(timeout_sec=0.005)

        try:
            run(0.5)
            start = (simulator.pose.x, simulator.pose.y)
            run(0.8, publish=True)
            self.assertGreater(simulator.pose.x - start[0], 0.05)
            if lateral_enabled:
                self.assertGreater(simulator.pose.y - start[1], 0.08)
            else:
                self.assertAlmostEqual(simulator.pose.y, start[1], places=5)
            # Simulate adapter disappearance: no command publisher activity at all.
            run(0.7)
            stopped = (simulator.pose.x, simulator.pose.y)
            run(0.4)
            self.assertAlmostEqual(simulator.pose.x, stopped[0], places=5)
            self.assertAlmostEqual(simulator.pose.y, stopped[1], places=5)
        finally:
            executor.shutdown()
            simulator.destroy_node()
            source.destroy_node()
            rclpy.shutdown()

    def test_diff_simulator_rejects_lateral_and_stops_after_publisher_loss(self):
        self.scenario(False)

    def test_omni_body_simulator_moves_laterally_and_stops_after_publisher_loss(self):
        self.scenario(True)
