ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO} AS ros

ARG USER
ARG UID

ENV USER=${USER}
ENV UID=${UID}
ENV GUID=${UID}

WORKDIR /home/${USER}/workspace
RUN colcon build

# For some reason, new Ubuntu containers create this user by default
# and it's goofing all of my stuff up...
RUN userdel -r ubuntu

# Add user w/ sudo privileges; helps to resolve annoyances
# with building/modifying files inside and outside of container
RUN groupadd --gid ${GUID} ${USER} \
    && useradd --uid ${UID} --gid ${GUID} -ms /bin/bash -m ${USER} \
    && adduser ${USER} sudo \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Add user to `dialout` group so we can test serial communications
RUN gpasswd --add ${USER} dialout

# update our bashrc
COPY docker/bashrc_additions /home/${USER}/bashrc_additions
RUN cat /home/${USER}/bashrc_additions >> /home/${USER}/.bashrc

RUN chown -R ${USER}:${USER} /home/${USER}

# Prerequisites for cross-compiling the rookie_pico code
RUN sudo apt update && \
    sudo apt install --no-install-recommends -y \
        cmake \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib

# Other useful things for development
RUN sudo apt update && \
    sudo apt install --no-install-recommends -y \
        minicom \
        cppcheck \
        python3-serial \
        python3-pip \
        pipx \
        gdb \
        clangd

# Build the pico_interface lib, because doing it manually is annoying
COPY pico_interface /home/${USER}/pico_interface
WORKDIR /home/${USER}/pico_interface
RUN mkdir -p build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install

# ROS dependencies
# TODO: just use rosdep for this...
RUN sudo apt update \
    && sudo apt install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-slam-toolbox

# We need this for nanopb
RUN pip install grpcio-tools --break-system-packages
