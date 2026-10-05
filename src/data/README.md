# Runtime data

This directory is a local convenience location. Generated maps, bags, point
clouds, and trajectories are not source code and are excluded from Git.

Recommended layout:

```text
point_cloud/scans.pcd
trace_data/mapping_trajectory.txt
traversable/traversable_areas.pcd
bags/
```

The retained hardware map is `traversable_cloud/traversable_areas.pcd`.
`traversable_cloud/traversable_areas_start_patch.pcd` preserves that source and
adds the manually approved horizontal start patch used by the ROS 2 hardware
one-command launch.

Set `NAV3D_DATA_ROOT` to this directory or to external storage. Large datasets
should be published as versioned release assets, object storage, or a separate
Git LFS dataset with checksums and collection metadata.

Do not commit credentials, device serial numbers, private network topology, or
recordings containing sensitive environment data.
