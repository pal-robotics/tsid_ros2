cmake_minimum_required(VERSION 3.8)
project(tsid_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(_Boost_PYTHON310_HEADERS "boost/python.hpp")
ament_auto_find_build_dependencies()
find_package(pinocchio REQUIRED)
find_package(tsid REQUIRED)

generate_parameter_library(tsid_controllers_params
  src/tsid_controllers_parameters.yaml
)

ament_auto_add_library(tsid_controllers SHARED
  src/tsid_position_control.cpp
  src/cartesian_space_controller.cpp
  src/joint_space_controller.cpp
  src/tsid_velocity_control.cpp
  src/tasks/task-vel-joint.cpp
  src/joint_space_vel_controller.cpp
  src/tasks/task-cartesian-velocity.cpp
  src/tasks/task-se3-equality.cpp
  src/cartesian_velocity_controller.cpp
  src/sin_joint.cpp
  src/cartesian_force_controller.cpp
)

target_link_libraries(tsid_controllers
                      tsid_controllers_params
                      pinocchio::pinocchio  
                      tsid::tsid
)
target_include_directories(tsid_controllers PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  pinocchio::pinocchio
  tsid::tsid
)


pluginlib_export_plugin_description_file(controller_interface tsid_controllers.xml)

# if(BUILD_TESTING)

# endif()

ament_auto_package(INSTALL_TO_SHARE launch config
USE_SCOPED_HEADER_INSTALL_DIR)
