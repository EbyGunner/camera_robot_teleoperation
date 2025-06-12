FROM ros:humble

ARG USERNAME=gunner
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN rm -f /etc/apt/sources.list.d/ros*.list

RUN rm -rf /etc/apt/sources.list.d/ros* /etc/apt/keyrings/ros* /usr/share/keyrings/ros* || true

RUN rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y curl gnupg lsb-release

# Install tools and fetch the latest valid ROS GPG key
RUN apt-get update && apt-get install -y \
    sudo \
    lsb-release \
    gnupg2 \
    curl \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list


# Create a non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update && apt-get install -y sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Install GUI and webcam dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-image-tools \
    v4l-utils \
    udev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    python3-pip \
    && apt-get clean

# Set up environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# Allow GUI display forwarding
ENV DISPLAY=:0

# Switch to the new user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

CMD ["bash"]
