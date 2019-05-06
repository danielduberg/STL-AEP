cmake_minimum_required(VERSION 2.8.3)
project(stl_frontier_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  dd_gazebo_plugins
  dynamic_reconfigure
  genmsg
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  stl_aeplanner_msgs
  visualization_msgs
)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

generate_dynamic_reconfigure_options(
  cfg/STL.cfg
)

catkin_package(
  INCLUDE_DIRS include ${OCTOMAP_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES rrtplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenMP_LIBS}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(stl_frontier_planner src/stl_frontier_planner.cpp src/stl_frontier_planner/rrt_node.cpp)
target_compile_options(stl_frontier_planner PRIVATE ${OpenMP_FLAGS})
target_link_libraries(stl_frontier_planner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenMP_LIBS}
)
add_dependencies(stl_frontier_planner
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  stl_frontier_planner_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

add_executable(stl_frontier_planner_node src/stl_frontier_planner_node.cpp src/stl_frontier_planner.cpp src/stl_frontier_planner/rrt_node.cpp)
target_compile_options(stl_frontier_planner_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(stl_frontier_planner_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenMP_LIBS}
)
add_dependencies(stl_frontier_planner_node
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  stl_frontier_planner_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)