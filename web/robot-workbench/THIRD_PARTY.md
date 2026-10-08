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
