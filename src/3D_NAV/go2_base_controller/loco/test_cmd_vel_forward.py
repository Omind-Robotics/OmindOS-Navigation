#!/usr/bin/env python3
import argparse

import rospy
from geometry_msgs.msg import Twist


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish /cmd_vel to move the robot forward by a fixed distance."
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=1.0,
        help="Forward distance in meters. Default: 1.0",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.2,
        help="Forward speed in m/s. Default: 0.2",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz while moving. Default: 20",
    )
    parser.add_argument(
        "--wait-for-subscriber",
        type=float,
        default=5.0,
        help="Seconds to wait for a /cmd_vel subscriber before starting. Default: 5",
    )
    parser.add_argument(
        "--stop-publish-duration",
        type=float,
        default=0.5,
        help="Seconds to keep publishing zero velocity after motion. Default: 0.5",
    )
    return parser.parse_args(rospy.myargv()[1:])


def wait_for_subscriber(pub, timeout_sec):
    start = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            return True
        if timeout_sec > 0 and (rospy.Time.now() - start).to_sec() >= timeout_sec:
            return False
        rate.sleep()
    return False


def publish_for_duration(pub, msg, duration_sec, rate_hz):
    end_time = rospy.Time.now() + rospy.Duration.from_sec(duration_sec)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        pub.publish(msg)
        rate.sleep()


def main():
    rospy.init_node("test_cmd_vel_forward", anonymous=True)
    args = parse_args()

    if args.distance <= 0:
        raise ValueError("--distance must be > 0")
    if args.speed <= 0:
        raise ValueError("--speed must be > 0")
    if args.rate <= 0:
        raise ValueError("--rate must be > 0")
    if args.stop_publish_duration < 0:
        raise ValueError("--stop-publish-duration must be >= 0")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.sleep(0.2)

    if not wait_for_subscriber(pub, args.wait_for_subscriber):
        rospy.logwarn("No /cmd_vel subscriber detected before timeout. Publishing anyway.")

    move_duration = args.distance / args.speed
    move_cmd = Twist()
    move_cmd.linear.x = args.speed

    stop_cmd = Twist()

    rospy.logwarn(
        "Moving forward %.2f m at %.2f m/s for %.2f s. Ensure the path is clear.",
        args.distance,
        args.speed,
        move_duration,
    )
    publish_for_duration(pub, move_cmd, move_duration, args.rate)

    rospy.loginfo("Publishing zero velocity to stop.")
    publish_for_duration(pub, stop_cmd, args.stop_publish_duration, args.rate)
    pub.publish(stop_cmd)
    rospy.loginfo("Forward cmd_vel test complete.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
