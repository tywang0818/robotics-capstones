cmake_minimum_required(VERSION 3.8)
project(hppt_boundary_ros)

# ROS2 基本包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)


# 系统库
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# cppraplab 源文件路径（根据实际位置调整）
set(CPPRAPLAB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../source")
set(CPPRAPLAB_INC  "${CMAKE_CURRENT_SOURCE_DIR}/../../include")

set(CPPRAPLAB_SRCS
  ${CPPRAPLAB_ROOT}/graph.cpp
  ${CPPRAPLAB_ROOT}/search.cpp
  ${CPPRAPLAB_ROOT}/search_dijkstra.cpp
  ${CPPRAPLAB_ROOT}/avltree.cpp
  ${CPPRAPLAB_ROOT}/search_emoa.cpp
  ${CPPRAPLAB_ROOT}/search_hpp_pt.cpp
)

add_executable(hppt_boundary_ros
  src/hppt_boundary_ros.cpp
  ${CPPRAPLAB_SRCS}
)

target_include_directories(hppt_boundary_ros PUBLIC
  ${CPPRAPLAB_INC}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(hppt_boundary_ros
  rclcpp
  sensor_msgs
  octomap_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  moveit_msgs
)

target_link_libraries(hppt_boundary_ros
  ${OCTOMAP_LIBRARIES}
  Threads::Threads
)

install(TARGETS hppt_boundary_ros
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Standalone HPP-PT sanity test (star graph)
# 用法： ros2 run hppt_boundary_ros test_hppt_star
# 或直接： /root/ros2_ws/install/hppt_boundary_ros/lib/hppt_boundary_ros/test_hppt_star
# ----------------------------------------------------------------------------
add_executable(test_hppt_star
  tests/test_hppt_star.cpp
  ${CPPRAPLAB_SRCS}
)

target_include_directories(test_hppt_star PUBLIC
  ${CPPRAPLAB_INC}
)

target_link_libraries(test_hppt_star
  Threads::Threads
)

install(TARGETS test_hppt_star
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Stage A perf test — clusterFrontier + farthestPointSample
#   ros2 run hppt_boundary_ros test_stage_a_perf
# ----------------------------------------------------------------------------
add_executable(test_stage_a_perf
  tests/test_stage_a_perf.cpp
)
target_include_directories(test_stage_a_perf PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_stage_a_perf
  ${OCTOMAP_LIBRARIES}
)
install(TARGETS test_stage_a_perf
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Stage B1 perf test — computeInformationGain (视锥 raycast IG)
#   ros2 run hppt_boundary_ros test_stage_b_perf
# ----------------------------------------------------------------------------
add_executable(test_stage_b_perf
  tests/test_stage_b_perf.cpp
)
target_include_directories(test_stage_b_perf PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_stage_b_perf
  ${OCTOMAP_LIBRARIES}
)
install(TARGETS test_stage_b_perf
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# new_method Greedy Baseline 纯算法单测(Phase 1/2/4/5/6/7)
#   ros2 run hppt_boundary_ros test_baseline
# ----------------------------------------------------------------------------
add_executable(test_baseline
  tests/test_baseline.cpp
)
target_include_directories(test_baseline PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_baseline
  ${OCTOMAP_LIBRARIES}
)
install(TARGETS test_baseline
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()