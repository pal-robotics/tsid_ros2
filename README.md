# TSID Framework: Generic ROS 2 Controllers

A powerful, optimization-based control framework for ROS 2, powered by the [Task-Space Inverse Dynamics (TSID)](https://github.com/stack-of-tasks/tsid) library. 

This repository provides a suite of `ros2_controllers` that allow robots to perform complex manipulation and motion planning. By using a Quadratic Programming (QP) solver, the framework ensures that all motions respect the robot's physical dynamics, joint limits, and torque constraints.

---

## Architecture & Theory

The framework is robot-agnostic. It works by formulating a control task (e.g., reaching a coordinate or a joint state) as an optimization problem.

### The Optimization Problem
At every control cycle, the controller solves for the optimal accelerations ($\dot{v}$) and torques ($\tau$):

$$
\begin{aligned}
\min_{\tau, \dot{v}} \quad & \Vert \text{Task Error} \Vert^2 \\
\text{s.t.} \quad & M\dot{v} + h = \tau & \text{(Dynamics)} \\
& \tau^{\min} \leq \tau \leq \tau^{\max} & \text{(Torque Limits)} \\
& v^{\min} \leq v + \Delta t\dot{v} \leq v^{\max} & \text{(Velocity Limits)} \\
& q^{\min} \leq q(t) + \Delta t v(t) + \frac{1}{2}\Delta t^2\dot{v} \leq q^{\max} & \text{(Joint Limits)}
\end{aligned}
$$


---

## Available Controller Types

| Controller Class | Description | Command Type |
| :--- | :--- | :--- |
| `JointSpaceTsidController` | Tracks a desired joint configuration. | `Float64MultiArray` |
| `CartesianSpaceController` | Tracks a pose in operational space. | `EePos.msg` |
| `JointSpaceVelTsidController` | Tracks desired joint velocities. | `Float64MultiArray` |
| `CartesianVelocityController` | Tracks operational space velocities. | `Float64MultiArray` (6-dim) |

---

## Repository structure

```text
.
├── tsid_controllers/          # Main ROS 2 controller plugin implementations
├── tsid_controller_msgs/     # Custom message definitions (EePos, EeWrench)
└── tsid_interactive_marker/  # Tools for RViz-based interactive control
```

---

## Example: Usage with TIAGo Pro

While the **TSID Framework** is designed to be robot-agnostic, the following steps demonstrate a practical implementation using the **TIAGo Pro** robot in a simulated environment. This serves as a template for deploying the controllers on any hardware.

### 1. Start the Simulation
First, bring up the robot in a Gazebo environment to initialize the hardware interfaces and `controller_manager`.

```bash
ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py world_name:=empty
```

### 3. Switch to TSID Control

Replace the default trajectory controller with the TSID Joint Space version:

```bash
ros2 control switch_controllers \
    --deactivate arm_right_controller \
    --activate arm_right_joint_space_controller
``` 

### 4. Send a Command

Command the 7 joints of the TIAGo arm to move to a zero position:

```bash
ros2 topic pub /arm_right_joint_space_controller/joint_position_cmd \
    std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
    --once
```
## Technical Implementation Details

### Safety & Joint Limits
When using **Velocity Interface** controllers, a safety synchronization logic is applied to handle the physical limits of the robot:
* **Global Deceleration:** If any single axis decelerates to stop at its software joint limit, the controller automatically decelerates **all other axes** to a stop simultaneously. This prevents the robot from reaching skewed or dangerous configurations while one joint is restricted.
* **Recovery:** Once an axis has reached its limit, motion in the other joints is only possible if the new command moves the joint that is at its limit back into the valid workspace.

### Reference Frames
For Cartesian controllers, the system can be commanded in different frames via specific controller instances. This allows for intuitive teleoperation or automated planning:
* **End-Effector Frame:** Commands are relative to the tool/gripper (e.g., "move 5cm forward from the current hand position").
* **Robot Frame:** Commands are relative to the robot base (e.g., "move to coordinates X, Y, Z relative to the floor").

---

## Custom Message API

To interact with Cartesian and Torque-aware controllers, use the provided messages in `tsid_controller_msgs`:

### `tsid_controller_msgs/msg/EePos`
Used for operational space positioning.
* **`ee_name`** (`string[]`): The name of the link to be controlled (e.g., `arm_right_7_link`).
* **`desired_pose`** (`geometry_msgs/Pose[]`): The target displacement or pose.

### `tsid_controller_msgs/msg/EeWrench`
Used for force-feedback control tasks.
* **`ee_name`** (`string[]`): The target link.
* **`desired_wrench`** (`geometry_msgs/Wrench[]`): Target force/torque values for hardware equipped with F/T sensors.
