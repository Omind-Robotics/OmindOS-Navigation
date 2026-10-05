#!/usr/bin/env python3
import argparse
import math
import statistics
import struct
import time

import rospy
from sensor_msgs.msg import Imu, PointCloud2


def rate(samples):
    if len(samples) < 2:
        return 0.0
    return (len(samples) - 1) / (samples[-1][0] - samples[0][0])


def mean_std(values):
    return statistics.mean(values), statistics.pstdev(values)


def point_time_range(message):
    field = next(field for field in message.fields if field.name == "timestamp")
    endian = ">" if message.is_bigendian else "<"
    values = []
    for index in range(message.width * message.height):
        value = struct.unpack_from(
            endian + "d", message.data, index * message.point_step + field.offset
        )[0]
        if math.isfinite(value):
            values.append(value)
    stamp = message.header.stamp.to_sec()
    return (min(values) - stamp) * 1000.0, (max(values) - stamp) * 1000.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args(rospy.myargv()[1:])

    imus = []
    front = []
    rear = []
    last_cloud = {}
    rospy.init_node("m20_sensor_audit", anonymous=True)

    def imu_callback(message):
        imus.append(
            (
                message.header.stamp.to_sec(),
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
                message.header.frame_id,
            )
        )

    def cloud_callback(message, name, samples):
        samples.append((message.header.stamp.to_sec(), len(message.data)))
        last_cloud[name] = message

    rospy.Subscriber("/IMU", Imu, imu_callback, queue_size=2000)
    rospy.Subscriber(
        "/m20/lidar/front",
        PointCloud2,
        lambda msg: cloud_callback(msg, "front", front),
        queue_size=100,
    )
    rospy.Subscriber(
        "/m20/lidar/rear",
        PointCloud2,
        lambda msg: cloud_callback(msg, "rear", rear),
        queue_size=100,
    )

    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.01)

    if not imus or not front or not rear:
        raise RuntimeError("Missing IMU, front LiDAR, or rear LiDAR data")

    print("vendor_contract imu=/IMU axes aligned to base_link, units=rad/s,m/s^2")
    print("vendor_contract imu_position_base_link_m=[0.0632,-0.0268,-0.0435]")
    print("vendor_contract front_lidar_position_base_link_m=[0.32028,0,-0.013]")
    print("vendor_contract rear_lidar_position_base_link_m=[-0.32028,0,-0.013]")
    print(f"samples imu={len(imus)} front={len(front)} rear={len(rear)}")
    print(f"rates_hz imu={rate(imus):.3f} front={rate(front):.3f} rear={rate(rear):.3f}")
    for name, index, unit in (
        ("gyro_x", 1, "rad/s"),
        ("gyro_y", 2, "rad/s"),
        ("gyro_z", 3, "rad/s"),
        ("acc_x", 4, "m/s^2"),
        ("acc_y", 5, "m/s^2"),
        ("acc_z", 6, "m/s^2"),
    ):
        average, deviation = mean_std([sample[index] for sample in imus])
        print(f"{name} mean={average:.6f} std={deviation:.6f} {unit}")

    acceleration_norm = [
        math.sqrt(sample[4] ** 2 + sample[5] ** 2 + sample[6] ** 2)
        for sample in imus
    ]
    average, deviation = mean_std(acceleration_norm)
    print(f"acc_norm mean={average:.6f} std={deviation:.6f} m/s^2")
    differences = [min(abs(item[0] - other[0]) for other in rear) * 1000.0 for item in front]
    print(
        "front_rear_sync_ms "
        f"mean={statistics.mean(differences):.4f} max={max(differences):.4f}"
    )
    imu_stamps = [sample[0] for sample in imus]
    for name, clouds in (("front", front), ("rear", rear)):
        covered_clouds = [
            cloud for cloud in clouds if imu_stamps[0] <= cloud[0] <= imu_stamps[-1]
        ]
        imu_differences = [
            min(abs(cloud[0] - imu_stamp) for imu_stamp in imu_stamps) * 1000.0
            for cloud in covered_clouds
        ]
        print(
            f"{name}_imu_header_sync_ms "
            f"mean={statistics.mean(imu_differences):.4f} max={max(imu_differences):.4f}"
        )
    for name in ("front", "rear"):
        minimum, maximum = point_time_range(last_cloud[name])
        message = last_cloud[name]
        print(
            f"{name} frame={message.header.frame_id!r} points={message.width * message.height} "
            f"point_time_ms=[{minimum:.3f},{maximum:.3f}]"
        )
    print(f"imu_frame={imus[-1][7]!r}")
    print(f"nuc_minus_sensor_clock={rospy.Time.now().to_sec() - imus[-1][0]:.3f}s")


if __name__ == "__main__":
    main()
