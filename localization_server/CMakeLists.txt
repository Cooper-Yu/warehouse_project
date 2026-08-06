cmake_minimum_required(VERSION 3.8)
project(localization_server)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/auto_initial_pose.py
  scripts/loaded_scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
