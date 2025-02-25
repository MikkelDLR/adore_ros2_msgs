cmake_minimum_required(VERSION 3.8)
project(adore_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
include(requirements.cmake)

# Collect all *.msg files from msg/ directory as RELATIVE paths
file(GLOB RELATIVE_MSG_FILES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "msg/*.msg"
)

# Pass the relative .msg files to rosidl_generate_interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${RELATIVE_MSG_FILES}
  DEPENDENCIES
    geometry_msgs
    shape_msgs
    nav_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skipping copyright
  set(ament_cmake_copyright_FOUND TRUE)
  # Skipping cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
