cmake_minimum_required(VERSION 3.8)
project(known_free_validator)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(jaka_planning_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(known_free_validator_node src/known_free_validator_node.cpp)
target_compile_features(known_free_validator_node PUBLIC cxx_std_17)
ament_target_dependencies(known_free_validator_node
  jaka_planning_interfaces
  geometric_shapes
  moveit_core
  moveit_ros_planning
  octomap_msgs
  rclcpp
  sensor_msgs
)
target_include_directories(known_free_validator_node PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(known_free_validator_node ${OCTOMAP_LIBRARIES})

install(TARGETS known_free_validator_node DESTINATION lib/${PROJECT_NAME})
ament_package()
