# OmindOS Navigation Runtime delivery

## Objective
Keep the development repository private while providing a reproducible runtime and a small public-facing ROS 2 integration boundary.

## Build
From the repository root:

    docker build -f docker/Dockerfile.runtime -t omindos/navigation:0.1 .

The Dockerfile uses a multi-stage build. Source is present in the builder stage; the final stage receives the colcon install tree rather than the complete repository.

## Run
Use runtime/compose.runtime.yaml or run the image with host ROS networking and the required sensor/device mappings.

## Security boundary
A runtime image is not a cryptographic source-protection mechanism. Compiled C++ is harder to inspect than source, while installed Python packages may remain inspectable. Do not place Git history, credentials, private keys, customer secrets or development-only configuration in the runtime image.

## Product boundary
Keep proprietary OmindOS application/orchestration code in private packages. Treat GPL and other third-party components according to their own licenses. Publish or provide only the source that the relevant license requires; do not expose unrelated proprietary repositories.

## v0.1 next step
Validate the runtime image on the target ROS 2 Humble machine, then add target-specific platform adapters and hardware acceptance tests.
