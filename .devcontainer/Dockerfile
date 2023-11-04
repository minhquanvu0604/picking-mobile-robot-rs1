FROM osrf/ros:noetic-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Customise the installation
USER root

RUN apt-get update -y && apt-get install -y \
    python3 \
    python3-yaml \
    python3-rospkg

RUN apt-get update -y && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-slam-toolbox \
    ros-noetic-slam-gmapping 

RUN apt-get update -y && apt-get install -y \
    ros-noetic-robot-controllers \
    ros-noetic-moveit-core \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-topic-tools

RUN apt-get update -y && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-gazebo 
    
RUN apt-get update && apt-get install -y \
    python3-pip

RUN ln -s /usr/bin/python3 /usr/bin/python

USER $USERNAME

RUN pip install defusedxml

# Rosdep update
RUN rosdep update

RUN mkdir -p /home/$USERNAME/catkin_ws/src

RUN git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git \
    /home/$USERNAME/catkin_ws/src/aws-robomaker-small-warehouse-world

RUN git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git \
    /home/$USERNAME/catkin_ws/src/gazebo_models_worlds_collection

RUN git clone https://github.com/nilseuropa/gazebo_ros_battery.git \
    /home/$USERNAME/catkin_ws/src/gazebo_ros_battery

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# RUN cd /home/$USERNAME/catkin_ws && /opt/ros/noetic/bin/catkin_make 