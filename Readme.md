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

Open a **Command Prompt** (`cmd.exe`) and run the following commands:

```bash
cd C:\Users\YourUsername\Documents  # Navigate to a suitable directory
git clone https://github.com/kiinging/gz-rviz2-jazzy_ros2-docker-windows11.git
cd rviz2-jazzy_ros2-docker-windows11
```

### 3. Build and Start the Docker Container

In the same **Command Prompt**, execute:

```bash
docker-compose up --build
```

- This command will **build** and start the container.
- To **exit** the container while stopping and deleting it, press `Ctrl+C` and then run:
  ```bash
  docker-compose down
  ```
  This ensures the container is removed after exiting.

### 4. Access the Container

Open a **new Command Prompt** (`cmd.exe`) and run:

```bash
docker-compose up -d  # Start the container in detached mode
docker exec -it ros2_jazzy_container bash
```

Now, you are inside the container’s interactive shell.

### 5. Build and Launch the Robot Model

Once inside the container, navigate to the ROS 2 workspace and build the project:

```bash
cd ros2-project/diff_drive_ws
colcon build
source install/setup.bash
```

To launch the URDF model of the robot, use:

```bash
ros2 launch rsp.launch.py
```

You can also launch other launch files to study control, navigation, and additional functionalities of the robot.

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

```bash
docker-compose up --build  
docker exec -it ros2_jazzy_container bash  

docker container prune  
ros2 pkg list | grep joint_state_publisher
```

---

Happy coding and exploring ROS 2 on Windows!

