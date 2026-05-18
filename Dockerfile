FROM osrf/ros:noetic-desktop-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

# Install basic utilities
RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

# Create a new user with specified UID and GID
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} docker
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} docker
RUN usermod -a -G dialout docker

# Configure sudoers file for the new user
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget



# ---------------------------------------------------------------------------
# Block 1 - Install ROS dependencies, editors, and Practice 2 system packages.
# This block keeps the original dependencies from Practice 1 and adds the
# packages required by Practice 2: pip and Tk support for Python/OpenCV windows.
# ---------------------------------------------------------------------------
RUN apt-get update &&\
    apt-get install -y \
    ros-noetic-effort-controllers* \
    ros-noetic-ackermann-msgs ros-noetic-hector-gazebo \
    ros-noetic-moveit* \
    ros-noetic-soem \
    ros-noetic-socketcan-interface \
    ros-noetic-joint-trajectory-controller \
    python3-pip \
    python3.8-tk \
    nano vim \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Block 2 - Install the Deep Learning / Computer Vision dependency required
# for the gesture-recognition part of Practice 2.
# MediaPipe is required later by pose_estimation.py.
# ---------------------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir mediapipe==0.10.9





# Switch to the new user
USER docker

# Set HOME environment variable
ENV HOME /home/docker
RUN mkdir -p ${HOME}/catkin_ws/src

# Copy the source code into the Docker workspace
COPY ./src ${HOME}/catkin_ws/src


# Compile the ROS workspace
RUN . /opt/ros/noetic/setup.sh && \
    cd ${HOME}/catkin_ws && \
    catkin_make 

# ---------------------------------------------------------------------------
# Block 3 - Give execution permissions to the Python scripts used by ROS.
# ROS can only execute the scripts with rosrun/roslaunch if they are executable.
# The "|| true" prevents the Docker build from failing if a future script is not
# present yet during an intermediate development stage.
# ---------------------------------------------------------------------------
USER root
RUN cd ${HOME}/catkin_ws/src/intelligent_robotics/scripts/ && \
    chmod +x spawn_random_position.py || true && \
    chmod +x show_camera_robot.py || true && \
    chmod +x capture_video.py || true && \
    chmod +x pose_estimation.py || true && \
    chmod +x obstacle_avoidance.py || true && \
    chmod +x obstacle_detection.py || true && \
    chmod +x blue_ackermann_low_level_bridge.py || true

RUN cd ${HOME}/catkin_ws/src/ackermann_vehicle/ackermann_vehicle_gazebo/scripts/ && \
    chmod +x ackermann_controller

    
# Update bash configuration
RUN echo "TERM=xterm-256color" >> ~/.bashrc
RUN echo "# COLOR Text" >> ~/.bashrc
RUN echo "PS1='\[\033[01;33m\]\u\[\033[01;33m\]@\[\033[01;33m\]\h\[\033[01;34m\]:\[\033[00m\]\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc
RUN echo "CLICOLOR=1" >> ~/.bashrc
RUN echo "LSCOLORS=GxFxCxDxBxegedabagaced" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "## ROS" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Switch back to the docker user
USER docker

# Set environment variables for graphical display and NVIDIA GPU
ENV DISPLAY=:0    
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility
ENV ROS_HOSTNAME=localhost
# Set the working directory to the ROS workspace
WORKDIR ${HOME}/catkin_ws
