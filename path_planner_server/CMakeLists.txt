cmake_minimum_required(VERSION 3.8)
project(path_planner_server)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/collect_navigation_logs
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
