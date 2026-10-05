# OmindOS Navigation v0.1

This release freezes the current ROS 2 Humble Nav_3d-derived stack as the first OmindOS Navigation baseline.

## Preserved capabilities
FAST-LIO, Livox integration, 3D planning, Bezier optimization, NMPC, RViz2 3D goal tooling, and the native ROS 2 validation path are retained.

## Platform separation
Go2-specific controllers, odometry bridges, command gates, safety supervision, launch files, and extrinsics are legacy/platform-adapter material. OmindOS Navigation core must not depend on Go2.

## Validation boundary
The inherited 2026-09-08 codebase includes offline ROS 2 validation and hardware-bringup tests. Historical validation showed the 3D offline loop reaching COMPLETED; CasADi QRSQP could fall back to the existing tracker in some cycles. Go2 real-motion validation was not claimed.

## Next work
1. Formalize OmindOS ROS 2 interface contracts.
2. Move robot-specific integrations into explicit adapters.
3. Add multi-goal inspection mission management.
4. Add whole-robot collision checking.
5. Run acceptance tests on the intended OmindOS hardware platform.
