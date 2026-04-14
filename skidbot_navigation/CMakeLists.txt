cmake_minimum_required(VERSION 3.8)
project(skidbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  skidbot_navigation/twist_to_twist_stamped.py
  skidbot_navigation/planner_benchmark.py
  skidbot_navigation/gps_waypoint_logger.py
  skidbot_navigation/interactive_waypoint_follower.py
  skidbot_navigation/logged_waypoint_follower.py
  skidbot_navigation/planner_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
