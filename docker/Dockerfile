ARG USER="ubuntu"
ARG UID=1000

FROM ubuntu:noble AS ubuntu

ARG USER
ARG UID

ENV USER=${USER}
ENV UID=${UID}
ENV GUID=${UID}

# RUN apt update && \
#     apt install -y \
#         git \
#         openssh-server

# RUN userdel -r ubuntu

# Add user w/ sudo privileges; helps to resolve annoyances
# with building/modifying files inside and outside of container
# RUN groupadd --gid ${GUID} ${USER} \
#     && useradd --uid ${UID} --gid ${GUID} -ms /bin/bash -m ${USER} \
#     && adduser ${USER} sudo \
#     && echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
# RUN useradd -u ${UID} -ms /bin/bash ${USER} \
#     && echo "${USER}:${USER}" | chpasswd && adduser ${USER} sudo \
#     && echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

FROM ros:rolling AS ros

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
COPY bashrc_additions /home/${USER}/bashrc_additions
RUN cat /home/${USER}/bashrc_additions >> /home/${USER}/.bashrc

RUN chown -R ${USER}:${USER} /home/${USER}

# Prerequisites for building the rookie_pico code
RUN sudo apt update && \
    sudo apt install --no-install-recommends -y \
        cmake \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib