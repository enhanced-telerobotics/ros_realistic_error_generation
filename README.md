# Realistic robot error injection - ROS Package

# ROS2 installation

Create your ROS 2 workspace and clone all repositories using `vcs`:

```bash
source /opt/ros/galactic/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
vcs import --input https://raw.githubusercontent.com/surgical-robotics-ai/realistic_robot_error_injection/refs/heads/main/ros2-error-injection-devel.repos.yaml  
```

Compile using `colcon`
```
cd ~/ros2_ws
colcon build 
source ~/ros2_ws/install/setup.bash
```
