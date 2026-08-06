cmake_minimum_required(VERSION 3.8)
project(map_server)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tools/clear_map_polygon.py
  tools/generate_current_sim_keepout_mask.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
