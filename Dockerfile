# Use official ROS2 Jazzy image
FROM osrf/ros:jazzy-desktop

# Set working directory
WORKDIR /app

# Copy files
COPY generic_robot_package /app/

# Install dependencies
RUN apt-get update && apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Build and install package
RUN colcon build --symlink-install --packages-select generic_robot_package

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV ROS_MASTER_URI=http://localhost:11311

# Run launch file
CMD ["ros2", "launch", "generic_robot_package", "launch.py"]