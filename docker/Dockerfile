FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ARDUPILOT_REF=92b0cd788ec29406f26c6f9c31d5ceedbd1cc538

ENV ARDUPILOT_REF=${ARDUPILOT_REF} \
    MUJOCO_GL=osmesa \
    ROS_DOMAIN_ID=42 \
    PYTHONUNBUFFERED=1

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        git \
        build-essential \
        cmake \
        ninja-build \
        ccache \
        gawk \
        wget \
        curl \
        jq \
        sudo \
        lsb-release \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-nav-msgs \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-visualization-msgs \
        ros-humble-tf2-ros \
        ros-humble-rosgraph-msgs \
        ros-humble-foxglove-bridge \
        libgl1 \
        libglfw3 \
        libosmesa6 \
        libegl1 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
        mujoco \
        numpy \
        pymavlink \
        pytest

RUN useradd --create-home --shell /bin/bash ardupilot \
    && printf 'ardupilot ALL=(ALL) NOPASSWD:ALL\n' > /etc/sudoers.d/ardupilot \
    && chmod 0440 /etc/sudoers.d/ardupilot \
    && install -d -o ardupilot -g ardupilot /opt/ardupilot

USER ardupilot
WORKDIR /opt

RUN [[ "${ARDUPILOT_REF}" =~ ^[0-9a-f]{40}$ ]] \
    && git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot \
    && git checkout --detach "${ARDUPILOT_REF}" \
    && git submodule update --init --recursive \
    && git rev-parse HEAD | tee /opt/ardupilot/ARDUPILOT_BUILD_REF \
    && test "$(git rev-parse HEAD)" = "${ARDUPILOT_REF}"

RUN export USER=ardupilot HOME=/home/ardupilot \
    && cd /opt/ardupilot \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && ./waf configure --board sitl \
    && ./waf copter

USER root

RUN git config --system --add safe.directory /opt/ardupilot \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["bash", "-lc", "sleep infinity"]
