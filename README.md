# TSID Framework

The **TSID Framework** is a robotics software package built on top of the [Task-Space Inverse Dynamics (TSID)](https://github.com/stack-of-tasks/tsid) library. This framework leverages the TSID library to implement controllers for robot manipulation and motion planning. 

In this package, we provide a set of ROS2 controllers, including the **Cartesian Space Controller**, which allows precise control of the robot's end-effector frame in Cartesian space.

## Features

- **Integration with the TSID Library**: Leverages advanced inverse dynamics techniques for task-space control.
- **ROS2 Controllers**: Implements controllers compatible with the ROS2 control stack.
- **Interactive Control**: Supports `rviz2` for intuitive interaction using an interactive marker.

## Cartesian Space Controller

The **Cartesian Space Controller** controls the robot's end-effector by directly specifying Cartesian positions, velocities, and orientations, enabling precise and intuitive manipulation.

### How to Launch the Controller

1. **Launch the Cartesian Space Controller**:
   Run the following command to start the controller:
   ```bash
   ros2 launch tsid_controllers cartesian_space_controller.launch.py

   Open Rviz2 and spawn the interactive marker:
   ```bash
   ros2 run rviz2 rviz2

   Switch controllers (for example run the controller on right side): 
   ```bash
   ros2 control switch_controllers --activate cartesian_space_controller --deactivate arm_right_controller

⚠ **Warning**: This controller is specifically customized for the **INRIA robot** and its simulation environment. If you wish to use it, ensure you are using the INRIA robot simulation setup (work in progress for the rest of the platform).
