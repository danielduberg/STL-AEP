cmake_minimum_required(VERSION 2.8.3)
project(stl_evaluation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  octomap_msgs
  roscpp
  roslib
  stl_aeplanner_msgs
  tf2
)
find_package(octomap REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/stl_evaluation_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)