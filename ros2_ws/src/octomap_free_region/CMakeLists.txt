cmake_minimum_required(VERSION 3.16)
project(octomap_free_region)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(octomap_free_region_node src/octomap_free_region_node.cpp)
target_include_directories(octomap_free_region_node PRIVATE
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(octomap_free_region_node ${OCTOMAP_LIBRARIES})
ament_target_dependencies(octomap_free_region_node
  rclcpp
  octomap_msgs
  visualization_msgs
  std_msgs
  geometry_msgs
)

install(TARGETS octomap_free_region_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
