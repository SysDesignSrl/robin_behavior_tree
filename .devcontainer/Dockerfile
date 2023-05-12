FROM ros:noetic-robot
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 #
 # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
 && apt-get update \
 && apt-get install -y sudo \
 && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update \
 && apt-get install -y git git-lfs build-essential gdb \
 && apt-get install -y python3-catkin-tools python3-osrf-pycommon python3-pip \
 && apt-get install -y python3-vcstool \
 && apt-get install -y iproute2 iputils-ping \
 && apt-get install -y vim

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# Copy
ENV WORKSPACE_ROOT=/home/$USERNAME/workspace
RUN mkdir -p ${WORKSPACE_ROOT}/src
WORKDIR ${WORKSPACE_ROOT}
