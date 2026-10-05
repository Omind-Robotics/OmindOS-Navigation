# Upstream provenance

- Repository: `https://github.com/hku-mars/FAST_LIO.git`
- Branch: `ROS2`
- FAST-LIO commit: `a4743b095409588842a5b30ddfa27e29d2f99164`
- `include/ikd-Tree` commit: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- Imported: 2026-08-29

The runtime frame literals were namespaced from `camera_init -> body` to
`lio_odom -> lio_imu`. This prevents the estimator's raw IMU pose from
conflicting with the robot's accepted `base_link -> body -> livox_frame ->
imu_link` static transform chain. A separate adapter publishes the composed
base pose.

The unused upstream `pcl_ros` build dependency was removed. The source uses
native PCL and `pcl_conversions`; both are already present in the validated
hardware image.

The missing upstream `tf2_ros` target dependency was added so its existing
`TransformBroadcaster` include is exported to the compiler on ROS 2 Humble.

MID360 CustomMsg points are stable-sorted by `offset_time` before IMU
undistortion. The change and the dense 0.20 m runtime profile mirror the
same-installation settings already accepted in `/home/nuc/Nav_test`.
