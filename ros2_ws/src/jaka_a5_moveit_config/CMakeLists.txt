cmake_minimum_required(VERSION 3.8)
project(jaka_a5_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/add_collision_objects.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
