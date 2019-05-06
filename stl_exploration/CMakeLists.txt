cmake_minimum_required(VERSION 2.8.3)
project(stl_exploration)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  genmsg
  geometry_msgs
  mavros_msgs
  nav_msgs
  nodelet
  octomap_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  stl_aeplanner_msgs
  tf
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(fly_to src/fly_to.cpp)
target_link_libraries( fly_to ${catkin_LIBRARIES})
add_dependencies(fly_to ${${PROJECT_NAME}_EXPORTED_TARGETS} 
stl_exploration_generate_messages_cpp
${PROJECT_NAME}_gencfg)

add_executable(stl_exploration src/stl_exploration.cpp)
target_link_libraries(stl_exploration ${catkin_LIBRARIES})
add_dependencies(stl_exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} 
stl_exploration_generate_messages_cpp
${PROJECT_NAME}_gencfg)