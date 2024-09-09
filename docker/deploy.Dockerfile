# syntax=docker/dockerfile:1

ARG ROS_DISTRO=rolling

FROM --platform=${BUILDPLATFORM} ros:${ROS_DISTRO} AS build

ADD ../pico_interface /pico_interface
WORKDIR /pico_interface
RUN mkdir -p build \
    && rm -r ./build/* \
    && cd build \
    && cmake .. \
    && make \
    && make install

ADD ../workspace /workspace
WORKDIR /workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    apt update \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && rm -r build install log \
    && colcon build

# RUN chmod +x /ros_entrypoint.sh

# get rid of the entrypoint to see if that makes the pi happy...
ENTRYPOINT [ ]