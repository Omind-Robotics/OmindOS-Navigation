# Renderer dependencies

The checked-in `robot-scene.js` bundle is generated from `robot-scene.source.js`.
It includes Three.js **0.180.0**, `RoundedBoxGeometry` and `RoomEnvironment`
from the same npm package. Three.js is MIT licensed; its complete license is
preserved in `../../../LICENSES/Three-MIT.txt` and its license comment is retained
in the bundle. Sources: https://github.com/mrdoob/three.js/tree/r180 and
https://www.npmjs.com/package/three/v/0.180.0.

The build-only tool is esbuild **0.25.10**. It is not embedded in the runtime bundle.

To reproduce after source edits:

```sh
cd web/robot-workbench
npm ci
npm run build
```

The Python server serves the prebuilt bundle locally. Customer operation does not
require npm, a CDN, cloud textures, or an external model service. The studio
environment and lights are constructed procedurally. An unavailable WebGL context
falls back to the previous 2D-canvas wireframe preview.

Shading, small box-edge bevels and selectable joint handles are visualization only.
URDF geometry dimensions, joint transforms, masses, saved profiles and the
kinematics/static-gravity calculations are not changed by presentation styles.


## RM75-B meshes and photo

Eight unmodified STL assets: RealManRobot/rm_models commit b63dddb20b7620d14cfff32678de60bb3a201ebc, Apache-2.0. License, notice and per-file Git blob hashes live in config/robot_models/realman_rm75/. Both arms share these assets.

mda1-reference.webp is the user-supplied robot appearance image already approved for the website in maganrobotics-boop/omindos PR #44, path apps/website/assets/mda1-robot-cutout-20261009.webp, Git blob 687794685dd32f209732e6d0b2ffeed8a8ad1aa4. It guides surface finish only; it is not a dimensional or dynamics source.
