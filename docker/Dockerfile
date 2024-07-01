ARG USER="ubuntu"

FROM ubuntu:noble as ubuntu

ARG USER
ENV USER=${USER}
ARG UID=1000
ARG GUID=${UID}

# Add user w/ sudo privileges; helps to resolve annoyances
# with building/modifying files inside and outside of container
RUN groupadd --gid $GUID $USER \
    && useradd --uid $UID --gid $GUID -m $USER \
    && adduser $USER sudo \
    && echo "$USER ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers


FROM ros:rolling

ARG USER
ENV USER=${USER}

WORKDIR /home/${USER}/workspace
RUN colcon build

# update our bashrc
COPY bashrc_additions /home/${USER}/bashrc_additions
RUN cat /home/${USER}/bashrc_additions >> /home/${USER}/.bashrc

