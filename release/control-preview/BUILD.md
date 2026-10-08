# Private build record

The release is built from private OmindOS source. Never upload the source checkout, native-c, native-objects, native-tests, Python modules, or generated C files to the public release repository.

Build base: official ros:humble-ros-base-jammy, Python 3.10.12 with python3-dev/gcc. Install python3-pip, python3-venv, libgl1, libegl1, libglfw3. Create a --system-site-packages virtualenv at /build/venv310. Build uses Cython 3.1.6, setuptools 80.9.0, wheel 0.45.1. Runtime pins: torch 2.5.1+cpu (PyTorch CPU index), numpy 1.26.4, mujoco 3.3.7. ROS Humble extensions retain the NumPy 1.x ABI.

Mount the build directory at /build. Put the private checkout in /build/source. Check out haraduka/mevius2 at 4f09680bb575574377b903bbc971bdb2695d507b and verify a clean checkout. Copy its models/ directory to /build/reference-models. Generate reference-manifest.json as canonical JSON (sort_keys=True, separators=(',', ':')) containing commit and a sorted files map of all model-relative paths to SHA-256. The manifest SHA-256 must equal 4f222335e987683f8fb46fc8e1966c1d7b6b85e1d7ebc98ecfe1c29e7617fd3e.

Run compile_native.py with the virtualenv. It creates /build/native-app containing 19 shared modules, public config, models and notices. It rejects business Python/C sources and excludes *.customer-record.json. Build Dockerfile with a dependency-only local base tag omindos/control-builddeps:preview5. All application compilation takes place in the mounted build directory; private sources are never copied into an image layer.

Validate the compiled modules using prepare_validation.py and the existing regression assertions, plus real ROS profile and quadruped checks. Run the packaged dynamics scenarios. Test the workbench in a browser against the final image. Audit model-manifest and asset tampering and ensure no backend source is present. Export the image only after these gates pass.

The host launcher loads and checks the exact image identity. Keep published versioned archives immutable. Source code stays in the private repository; the public repository receives notes, checksums and validation metadata.
