# Get official base image
FROM osrf/ros:jazzy-desktop-full

# Switch to bash (necessary for source to work)
SHELL ["/bin/bash", "-c"]

# Avoid interactive prompts during package installs
ARG DEBIAN_FRONTEND=noninteractive

# Install basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rmw-cyclonedds-cpp \
    nano \
    tmux \
    git \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Delete base image user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
    userdel -r ubuntu; \
    fi

# Create a new non-root user with custom name
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Source ROS 2 in every shell session
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

CMD ["/bin/bash"]

# Install additional requirements defined in requirements.txt
#COPY ../requirements.txt .
#RUN pip install --break-system-packages -r requirements.txt
