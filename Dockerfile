ARG FROM_IMAGE=ros:foxy
FROM ${FROM_IMAGE}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install udev
RUN sudo apt-get update && \
    sudo apt-get install \
    udev \
    python3-colcon-common-extensions \
    python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# Install Driver
WORKDIR /opt/
RUN git clone https://github.com/naoki-mizuno/ds4drv --branch devel && \
    cd ds4drv && \
    python3 setup.py install && \
    sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/ && \
    sudo udevadm trigger

# Build workspace
WORKDIR /opt/underlay_ws/src
COPY . .
WORKDIR /opt/underlay_ws
RUN source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source /opt/underlay_ws/install/setup.bash
