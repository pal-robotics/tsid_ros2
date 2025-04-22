# Msg to use for cartesian controller 

string[] ee_name
geometry_msgs/Wrench[] desired_wrench

# ros2 topic pub /cartesian_force_controller/wrench_cmd tsid_controller_msgs/msg/EeWrench "{ee_name: ['arm_right_7_link'],
# desired_wrench: [{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}]}" --once