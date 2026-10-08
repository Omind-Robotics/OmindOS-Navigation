# Third-party notices

OmindOS application modules are compiled binaries. Third-party components retain their own licenses.

- MEVIUS2 model and policy: Kento Kawaharazuka, MIT; upstream https://github.com/haraduka/mevius2 at commit 4f09680bb575574377b903bbc971bdb2695d507b. License: /opt/omindos/LICENSES/MEVIUS2-MIT.txt.
- RealMan RM75 model: attribution and upstream license in /opt/omindos/config/robot_models/realman_rm75/.
- Three.js: MIT, /opt/omindos/LICENSES/Three-MIT.txt.
- Ubuntu 22.04 / ROS 2 Humble base: https://hub.docker.com/_/ros and https://github.com/osrf/docker_images. Per-package notices in /usr/share/doc and /opt/ros/humble/share; exact installed versions are recorded in THIRD_PARTY_PACKAGES.txt.
- Python: https://www.python.org/psf/license/ ; the runtime is bundled.
- PyTorch, MuJoCo, NumPy and their Python dependencies: installed distribution metadata and license files are preserved in /opt/omindos-runtime/lib/python3.10/site-packages. Exact installed versions are recorded in THIRD_PARTY_PYTHON.json.
- Cython is used to compile the private application modules. No generated application C files or build directories are shipped.

Models, parameters, browser assets and third-party runtime files are included. No claim is made that third-party runtimes contain no source files; the no-source delivery boundary applies to OmindOS backend business logic.
