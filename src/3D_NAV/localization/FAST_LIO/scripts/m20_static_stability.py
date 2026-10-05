#!/usr/bin/env python3
import argparse
import math
import time

import rospy
from nav_msgs.msg import Odometry


def euler(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def angle_delta(value, reference):
    return math.atan2(math.sin(value - reference), math.cos(value - reference))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--warmup", type=float, default=15.0)
    parser.add_argument("--max-position-drift", type=float, default=0.05)
    parser.add_argument("--max-attitude-drift-deg", type=float, default=0.5)
    args = parser.parse_args(rospy.myargv()[1:])

    poses = []
    rospy.init_node("m20_static_stability", anonymous=True)

    def callback(message):
        position = message.pose.pose.position
        poses.append((position.x, position.y, position.z, *euler(message.pose.pose.orientation)))

    rospy.Subscriber("/Odometry_loc", Odometry, callback, queue_size=1000)
    print(f"Warmup for {args.warmup:.1f}s; keep M20 completely stationary.")
    time.sleep(args.warmup)
    poses.clear()
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.02)
    if len(poses) < 2:
        raise RuntimeError("Insufficient /Odometry_loc samples")

    reference = poses[0]
    position_drifts = []
    attitude_drifts = []
    axis_drifts = [[], [], []]
    for pose in poses:
        position_drifts.append(math.sqrt(sum((pose[i] - reference[i]) ** 2 for i in range(3))))
        sample_axis_drifts = [
            abs(math.degrees(angle_delta(pose[i], reference[i]))) for i in range(3, 6)
        ]
        for axis, drift in zip(axis_drifts, sample_axis_drifts):
            axis.append(drift)
        attitude_drifts.append(max(sample_axis_drifts))
    final_position = position_drifts[-1]
    final_attitude = attitude_drifts[-1]
    print(
        f"samples={len(poses)} final_position_drift={final_position:.4f}m "
        f"max_position_drift={max(position_drifts):.4f}m"
    )
    print(
        f"final_attitude_drift={final_attitude:.3f}deg "
        f"max_attitude_drift={max(attitude_drifts):.3f}deg"
    )
    print(
        "attitude_axes_deg "
        f"final_roll={axis_drifts[0][-1]:.3f} max_roll={max(axis_drifts[0]):.3f} "
        f"final_pitch={axis_drifts[1][-1]:.3f} max_pitch={max(axis_drifts[1]):.3f} "
        f"final_yaw={axis_drifts[2][-1]:.3f} max_yaw={max(axis_drifts[2]):.3f}"
    )
    passed = (
        max(position_drifts) <= args.max_position_drift
        and max(attitude_drifts) <= args.max_attitude_drift_deg
    )
    print("PASS" if passed else "FAIL")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
