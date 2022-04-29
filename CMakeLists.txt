cmake_minimum_required(VERSION 2.8.12)
project(oaisys_client)

add_definitions(-std=c++14)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  grid_map_msgs
  eigen_catkin
  cv_bridge
  oaisys_client_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy oaisys_client_ros
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/oaisys_planner.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread)

add_executable(oaisys_planner_node
  src/oaisys_planner_node.cpp
)
add_dependencies(oaisys_planner_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(oaisys_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread)

add_executable(oaisys_mapping_node
  src/oaisys_mapping_node.cpp
)
add_dependencies(oaisys_mapping_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(oaisys_mapping_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread)

add_executable(oaisys_replay_node
  src/oaisys_replay_node.cpp
)
add_dependencies(oaisys_replay_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(oaisys_replay_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread)
