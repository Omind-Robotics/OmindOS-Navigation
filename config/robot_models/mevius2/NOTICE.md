# MEVIUS2 workbench reference geometry

Source: https://github.com/haraduka/mevius2/tree/4f09680bb575574377b903bbc971bdb2695d507b

The immutable STL URDF, 13 visual meshes and upstream MIT license are fetched by
`scripts/fetch_mevius2_workbench.py`. Each file is checked against its Git blob
hash and length in UPSTREAM.json before packaging. The original files remain
unchanged. The workbench profile changes only the displayed robot name.

Geometry, masses, inertias, joint limits, effort and velocity are upstream
reference values. The displayed standing pose is 0 / 0.7 / -1.2 rad per leg.
The software profile uses identity output-to-joint coordinates for preview;
these are not measured customer motor direction, reduction or zero offsets.
Customer ROBSTRIDE04 calibration remains unknown in the customer record.
No motor driver is connected and no physical hardware has been validated.
