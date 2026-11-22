ARG ZEPHYR_IMAGE=ghcr.io/jaster-prj/devcontainer-zephyr:latest
FROM ${ZEPHYR_IMAGE}

USER root
WORKDIR /workspace

COPY --from=wasserverteiler-src . /workspace/application
COPY --from=native-udp-src . /workspace/zephyr-native-linux-udp-can
# This hardware-only timeout has no value when TASK_WDT is disabled on native.
RUN sed -i '/^CONFIG_TASK_WDT_MIN_TIMEOUT=/d' /workspace/application/prj.conf
RUN /opt/venv/bin/west build -b native_sim /workspace/application \
      --build-dir /workspace/application/build/native_sim \
      -- -DZEPHYR_EXTRA_MODULES="/workspace/canopennodezephyr;/workspace/zephyr-native-linux-udp-can" \
          -DEXTRA_CONF_FILE=/workspace/application/boards/native_sim.conf \
          -DEXTRA_DTC_OVERLAY_FILE=/workspace/application/boards/native_sim.overlay
CMD ["/workspace/application/build/native_sim/zephyr/zephyr.exe"]
