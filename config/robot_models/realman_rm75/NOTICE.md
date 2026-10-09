# RM75-B URDF attribution

Copyright 2024 realman-robotics. Apache-2.0; see LICENSE and UPSTREAM_LICENSE.

Source: RealManRobot/rm_models, commit b63dddb20b7620d14cfff32678de60bb3a201ebc,
path RM75/urdf/RM75-B/urdf/RM75-B.urdf. Retrieved via the existing OmindOS
Manipulation model; source Git blob 92b8823ba425eff3cf7b353cf8158d09ced23632.
RM75-B.urdf is unmodified. The eight RM75-B STL assets are included unmodified
from the same commit; see meshes/PROVENANCE.json.

scripts/build_workbench_demo.py builds a derived composite model: two prefixed
copies of the seven-axis chain, fixed installation transforms, one waist pitch,
fixed head, reference grippers and wheel joints. Arm joint origins, axes, limits,
link masses and inertias come from this URDF. Arm visual and collision meshes
are retained. Body, mounts, grippers, wheels and motor declarations
remain uncalibrated reference values. No hardware actuation is implemented.

