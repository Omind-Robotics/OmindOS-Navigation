FROM omindos/control-builddeps:preview5
COPY venv310 /opt/omindos-runtime
COPY native-app /opt/omindos
COPY source/release/control-preview/omindos-runtime /usr/local/bin/omindos-runtime
RUN chmod 755 /usr/local/bin/omindos-runtime
ENV PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=1
WORKDIR /opt/omindos
ENTRYPOINT ["/ros_entrypoint.sh", "/usr/local/bin/omindos-runtime"]
CMD ["--help"]
