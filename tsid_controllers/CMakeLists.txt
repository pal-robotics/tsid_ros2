cmake_minimum_required(VERSION 3.8)
project(tsid_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()
find_package(pinocchio REQUIRED)
find_package(tsid_vendor REQUIRED)


generate_parameter_library(tsid_controllers_params
  src/tsid_controllers_parameters.yaml
)

ament_auto_add_library(tsid_controllers SHARED
  src/tsid_position_control.cpp
  src/cartesian_space_controller.cpp
  src/joint_space_controller.cpp
  src/tsid_velocity_control.cpp
  src/tasks/task-vel-joint.cpp
  #src/joint_space_vel_controller.cpp
  src/tasks/task-cartesian-velocity.cpp
  src/cartesian_velocity_controller.cpp
)

target_link_libraries(tsid_controllers
                      tsid_controllers_params
                      pinocchio::pinocchio  
                      ${tsid_vendor_LIBRARIES}
)
target_include_directories(tsid_controllers PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  pinocchio::pinocchio
  ${tsid_vendor_INCLUDE_DIRS}
)


pluginlib_export_plugin_description_file(controller_interface tsid_controllers.xml)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)

#   ament_add_gmock(test_dynamic_tsid_controller
#   test/dynamic_tsid_controller_test.cpp
#   )

#   target_include_directories(test_dynamic_tsid_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   )

#   target_link_libraries(test_dynamic_tsid_controller
#   ${PROJECT_NAME}  ${EIGEN3_LIBRARIES}
#   )

# endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
