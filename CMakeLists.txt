cmake_minimum_required(VERSION 3.8)
project(dynamic_tsid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

generate_parameter_library(dynamic_tsid_controller_params
  src/dynamic_tsid_controller_parameters.yaml
)

ament_auto_add_library(dynamic_tsid_controller SHARED
  src/dynamic_tsid_controller.cpp
)
target_link_libraries(dynamic_tsid_controller dynamic_tsid_controller_params)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gmock(test_dynamic_tsid_controller
  test/dynamic_tsid_controller_test.cpp
  )

  target_include_directories(test_dynamic_tsid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_dynamic_tsid_controller
  ${PROJECT_NAME}  ${EIGEN3_LIBRARIES}
  )

endif()

ament_package()
