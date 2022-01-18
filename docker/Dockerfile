ARG ROS_DISTRO=foxy

FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt update \
    && apt install \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install ds4drv
WORKDIR /opt
RUN git clone https://github.com/naoki-mizuno/ds4drv --branch devel \
    && cd ds4drv \
    && python3 setup.py install

# Build workspace
WORKDIR /opt/underlay_ws
RUN mkdir src \
    && cd src \
    && git clone https://github.com/naoki-mizuno/ds4_driver \
        --branch ${ROS_DISTRO}-devel
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

COPY ./entrypoint.bash /
ENTRYPOINT ["/entrypoint.bash"]
