cmake_minimum_required(VERSION 3.8)
project(tsid_interactive_marker)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(tsid_interactive_marker
  src/interactive_marker_tsid.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()


ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

