# Set the base image to Ubuntu 22.04
FROM ubuntu:22.04

# Set environment variable to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install apt-utils first to avoid debconf warnings
RUN apt-get update && apt-get install -y apt-utils

# Set locale to UTF-8
RUN apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

# Preconfigure tzdata package to set timezone to America/Chicago
RUN ln -fs /usr/share/zoneinfo/America/Chicago /etc/localtime \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata

# Install other required dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl

# Add ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index and install ROS 2 Humble and additional packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y ros-humble-desktop \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-kinematics \
#    ros-humble-gazebo-ros \
#    ros-humble-gazebo-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit-ros-perception \
    ros-humble-rviz-visual-tools \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-resources \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    rospack-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize and update rosdep
RUN rosdep init && rosdep update

# Setup colcon mixins
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default

# Install neuromeka Python package
RUN pip3 install neuromeka

# Install git for clone the ROS2 source code
RUN apt-get update && apt-get install -y git

# Add ROS 2 environment setup to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /home/user/indy-ros2/install/setup.bash" >> /root/.bashrc \
    && echo "cd /home/user/indy-ros2" >> /root/.bashrc

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Download IndyROS2 source code on the indy-ros2 directory
RUN cd /home \
    && mkdir user && cd user \
    && mkdir indy-ros2 && cd indy-ros2 \
    && git clone https://github.com/neuromeka-robotics/indy-ros2.git src \
    && cd src \
    && git switch humble-indyDCP3 \
    && cd .. \
    && . /opt/ros/humble/setup.bash \
    && colcon build

# Set the default command to run when starting the container
CMD ["bash"]

