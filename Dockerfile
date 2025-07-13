# Base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Update & install system tools, ROS packages, and Gazebo dependencies
RUN apt-get update && apt upgrade -y && apt-get install -y \
    python3-pip \
    python3-tk \
    git \
    wget \
    curl \
    iputils-ping \
    lsb-release \
    sudo \
    vim \
    nano \
    unzip \
    terminator \
    subversion \
    gedit \
    # Core ROS 2 Humble packages
    python3-colcon-common-extensions \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-gazebo-ros \
    # Optional but useful for visualization and launching
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-turtlebot3-* \
 && rm -rf /var/lib/apt/lists/*


# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy==1.24.4\
    matplotlib \
    opencv-python \
    pandas \
    PyYAML \
    torch \
    ultralytics

# Create catkin workspace
# RUN apt-get install terminator -y
ENV ROS2_WS=/root/ros2_ws
RUN mkdir -p ${ROS2_WS}/src

# Set up ROS environment on shell startup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
    
# Default working directory
WORKDIR ${ROS2_WS}/src

# Copy source archive into workspace root
COPY src.zip ${ROS2_WS}/

# Unzip into workspace root and remove zip
RUN unzip ${ROS2_WS}/src.zip -d ${ROS2_WS}/ && rm ${ROS2_WS}/src.zip

# Return to the workspace root
WORKDIR ${ROS2_WS}

# Install ROS package dependencies using rosdep
RUN apt-get update && \
    apt-get install -y python3-rosdep && \
    rosdep update

RUN rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source workspace on container start
RUN /bin/bash -c  "source ${ROS2_WS}/install/setup.bash" 

# Enable Gazebo GUI from container
ENV DISPLAY=host.docker.internal:0
ENV QT_X11_NO_MITSHM=1

# Install X11 apps (optional)
RUN apt-get update && apt-get install -y x11-apps && rm -rf /var/lib/apt/lists/*

# Expose ROS master port
EXPOSE 11311

# Start a bash shell by default
CMD ["/bin/bash"]