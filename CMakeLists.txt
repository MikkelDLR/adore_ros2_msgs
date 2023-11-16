cmake_minimum_required(VERSION 3.8)
project(adore_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Border.msg"
  "msg/BorderID.msg"
  "msg/BorderInterval.msg"
  "msg/BorderProgress.msg"
  "msg/BorderTypeChangeProfile.msg"
  "msg/GraphValue.msg"
  "msg/CooperativeUser.msg"
  "msg/CooperativePlanning.msg"
  "msg/Num.msg"
  "msg/Path.msg"
  "msg/PathPoint.msg"
  "msg/PlanningRequest.msg"
  "msg/PlanningResult.msg"
  "msg/SchedulerNotification.msg"
  "msg/SetPoint.msg"
  "msg/SetPointRequest.msg"
  "msg/TerminalRequest.msg"
  "msg/TrafficClassification.msg"
  "msg/TrafficParticipant.msg"
  "msg/TrafficParticipantData.msg"
  "msg/TrafficParticipantDetection.msg"
  "msg/TrafficParticipantSet.msg"
  "msg/TrafficParticipantSetSimulation.msg"
  "msg/TrafficParticipantSimulation.msg"
  "msg/View.msg"

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




