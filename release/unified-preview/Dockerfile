# Clean source allowlist: never use the historical v0.1 image as a base.
FROM ros@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19
SHELL ["/bin/bash", "-c"]
WORKDIR /opt/omindos_preview
COPY src/nav3d_native/ src/nav3d_native/
COPY scripts/validate_platform_preview.sh scripts/validate_platform_preview.sh
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select nav3d_native && \
    source install/setup.bash && \
    python3 -c "from nav3d_native.platform_guard import load_profile; load_profile('quadruped')"
COPY entrypoint.sh /omindos_preview_entrypoint.sh
RUN chmod 755 /omindos_preview_entrypoint.sh && \
    mkdir -p /opt/omindos_preview/distribution-evidence && \
    dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' \
      > /opt/omindos_preview/distribution-evidence/os-packages.tsv
ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=171 PYTHONUNBUFFERED=1
LABEL org.opencontainers.image.title="OmindOS Navigation Unified Preview" \
      org.opencontainers.image.version="0.2.0-preview.1" \
      ai.omindos.scope="kinematic-preview-only" \
      ai.omindos.hardware-validated="false" \
      ai.omindos.publication-status="approved-preview"
ENTRYPOINT ["/omindos_preview_entrypoint.sh"]
CMD ["profiles"]
