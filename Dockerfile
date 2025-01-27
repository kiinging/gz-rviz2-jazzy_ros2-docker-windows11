# Use official ROS2 Jazzy image
FROM osrf/ros:jazzy-desktop


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    && rm -rf /var/lib/apt/lists/*


# Install dependencies
RUN apt-get update \
    && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joy* \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-twist-mux \
    && rm -rf /var/lib/apt/lists/*

# Optionally, add any additional setup for the user
RUN sudo apt-get update && sudo apt-get install -y \
    python3-pip \
    && sudo rm -rf /var/lib/apt/lists/*

# the user is created as ubunut (from osrf/ros image)
# Set the working directory for the new user
USER ubuntu
WORKDIR /home/ubuntu
# Append required setup commands to .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc 

# Ensure the user owns the .bashrc file
RUN chown ubuntu:ubuntu /home/ubuntu/.bashrc

CMD ["bash"]

