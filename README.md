# Robotic Arm Workspace

This workspace contains packages for a robotic arm simularion using ROS 2, including:

- **robotic_arm_bringup**: Launch files and startup configurations
- **robotic_arm_camera**: Camera integration and vision processing
- **robotic_arm_controllers**: Control algorithms for the robotic arm
- **robotic_arm_custom_nodes**: Custom ROS 2 nodes for specific functionalities
- **robotic_arm_description**: URDF models and mesh files
- **robotic_arm_firmware**: Firmware for embedded controllers
- **robotic_arm_kinematics**: Forward and inverse kinematics solvers
- **robotic_arm_msgs**: Custom message and service definitions
- **robotic_arm_recognition**: Object recognition modules
- **robotic_arm_transforms**: Coordinate transformations

## Getting Started

### Prerequisites
- ROS 2 (tested with Humble)
- Colcon build tools
- Dependencies listed in package.xml files

### Building
```bash
cd robot_arm_ws
colcon build
```

### Usage
1.launch the simuilation:
```bash
ros2 launch robotic_arm_description robot_arm_gazebo.launch.py
```
2.launch the controllers:
```bash
ros2 launch robotic_arm_controllers robot_arm_controllers.launch.py
```
3.Recognition using simulation camera:
```bash
ros2 launch robotic_arm_recognition robotic_arm_recognition_model.launch.py
```
4.transformation of the dynamic links (detected objects)
```bash
ros2 launch robotic_arm_transforms transforms.launch.xml
```
5.Solving the IK problems and computing the forward kinematics
```bash
ros2 launch robotic_arm_kinematics robotic_arm_kinematics.launch.py
```
### All the packages can be launch using this single command
```bash
source install/setup.bash
ros2 launch robotic_arm_bringup sim_robotic_arm.launchlaunch.py
```

## License

[Insert license information here]

## Contributors

[Your name/organization]
