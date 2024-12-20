cmake_minimum_required(VERSION 3.8)
project(adore_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
include(requirements.cmake)

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/MapConnection.msg"
  "msg/MapPoint.msg"
  "msg/MapLane.msg"
  "msg/MapRoad.msg"
  "msg/Map.msg"

  "msg/Route.msg"
  "msg/GoalPoint.msg"

  "msg/TrafficSignal.msg"
  "msg/TrafficSignals.msg"

  "msg/VehicleStateDynamic.msg"
  "msg/VehicleStateExtended.msg"
  "msg/IndicatorState.msg"
  "msg/VehicleCommand.msg"
  "msg/GearState.msg"
  "msg/VehicleInfo.msg"
  "msg/StateMonitor.msg"

  "msg/Trajectory.msg"
  "msg/TrajectoryTranspose.msg"

  "msg/SafetyCorridor.msg"
  
  "msg/TrafficParticipant.msg"
  "msg/TrafficClassification.msg"
  "msg/TrafficParticipantDetection.msg"
  "msg/TrafficParticipantSet.msg"

  DEPENDENCIES
  geometry_msgs
  shape_msgs
  nav_msgs
  builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()




