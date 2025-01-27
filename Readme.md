# RViz Jazzy ROS Docker for Windows 11

This repository provides a setup for running RViz2 using ROS 2 Jazzy on Windows 11 via Docker. It focuses on the specific configurations required to enable a smooth experience for Windows users.

## Why use this setup?
Using `docker-compose.yaml` simplifies the management of containers by defining all configurations, networks, and volumes in a single file. This approach makes it easier to build, start, and manage the containerized environment for running RViz2 on Windows.

## Requirements

1. **Windows 11** with Docker Desktop installed.
2. **VcXsrv Windows X Server** for X11 forwarding:
   - Download and install from: [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/).
3. **Docker and Docker Compose** installed on your system.

## Setup Instructions

### 1. Install VcXsrv Windows X Server

- Download and install VcXsrv from the link above.
- Start the X server using default configurations (or ensure it listens on `host.docker.internal:0`).

### 2. Clone this repository

```bash
git clone <repository-url>
cd rviz2-jazzy_ros2-docker-windows11
```

### 3. Build and Start the Docker Container

Use the following command to build and start the container:

```bash
docker-compose up --build
```

### 4. Access the Container

To interact with the container, use:

```bash
docker exec -it ros2_jazzy_container bash
```

### 5. Run RViz2

Once inside the container, run RViz2 using the command:

```bash
rviz2
```

## Key Configuration Details

### `docker-compose.yaml`
```yaml
version: "3.8"
services:
  ros2-jazzy:
    image: ros2_dev  # Or your custom image
    container_name: ros2_jazzy_container
    build:
      context: .  # Use your custom Dockerfile here
      dockerfile: Dockerfile
    hostname: deck  # Custom hostname
    environment:
      - DISPLAY=host.docker.internal:0
    volumes:
      - ./ros2-project:/home/ubuntu/ros2-project
    network_mode: host  # Use host network to enable X11 forwarding
    restart: "no"
    runtime: runc
    tty: true
    command: bash
```

This file simplifies container management by consolidating settings like image name, container name, and volume mappings. It also enables X11 forwarding through the `DISPLAY` environment variable.

### `Dockerfile`
```dockerfile
# Use official ROS2 Jazzy image
FROM osrf/ros:jazzy-desktop

# Install basic utilities
RUN apt-get update \
    && apt-get install -y \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 dependencies
RUN apt-get update \
    && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Set working directory for user
WORKDIR /home/ubuntu

# Install Python dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    python3-pip \
    && sudo rm -rf /var/lib/apt/lists/*

# Copy entrypoint and bash configuration
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/ubuntu/.bashrc

# Set entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
```

This Dockerfile builds a ROS 2 Jazzy environment with pre-installed dependencies for RViz2 and ROS 2 controllers. It also configures the container for the user `ubuntu`.

## Notes

1. Ensure the X server is running on your host machine before starting the container.
2. For troubleshooting, verify that `DISPLAY` is correctly set to `host.docker.internal:0` in the container.
3. Use `network_mode: host` to enable X11 forwarding seamlessly.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Useful commands
docker-compose up --build
docker exec -it ros2_jazzy_container bash


---
Happy coding and exploring ROS 2 on Windows!

