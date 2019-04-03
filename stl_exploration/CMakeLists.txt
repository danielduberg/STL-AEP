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

add_library(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(laserscan_to_pointcloud ${catkin_EXPORTED_LIBRARIES})
endif()

add_executable(fly_to src/fly_to.cpp)
target_link_libraries( fly_to ${catkin_LIBRARIES})
add_dependencies(fly_to stl_exploration_generate_messages_cpp)

add_executable(stl_exploration src/stl_exploration.cpp)
target_link_libraries(stl_exploration ${catkin_LIBRARIES})
add_dependencies(stl_exploration ${catkin_EXPORTED_TARGETS})
add_dependencies(stl_exploration stl_exploration_generate_messages_cpp)