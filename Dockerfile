FROM zephyrprojectrtos/zephyr-build:v0.26.17
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip git cmake build-essential pkg-config libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install colcon-common-extensions vcstool catkin_pkg lark \
      empy==3.3.4 numpy setuptools

ENV ZEPHYR_BASE=/workspace/zephyrproject/zephyr
RUN git config --system --add safe.directory '*'

WORKDIR /workspace/zephyrproject
CMD ["bash"]
