cmake_minimum_required(VERSION 2.8.3)
project(stl_aeplanner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  stl_aeplanner_msgs
  dd_gazebo_plugins
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  tf
  tf2
  genmsg 
  actionlib_msgs 
  actionlib

  dynamic_reconfigure
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

# add_message_files(
#   DIRECTORY msg 
#   FILES
#   Node.msg
#   LTLStats.msg
# )
# add_action_files(
#   DIRECTORY action 
#   FILES stl_aeplanner.action
# )
# add_service_files(
#   DIRECTORY srv
#   FILES Reevaluate.srv
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   visualization_msgs
#   std_msgs
#  actionlib_msgs
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/STL.cfg
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES stl_aeplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenMP_LIBS}
  CATKIN_DEPENDS stl_aeplanner_msgs dd_gazebo_plugins message_runtime roscpp geometry_msgs visualization_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(stl_aeplanner src/stl_aeplanner_nodelet.cpp src/stl_aeplanner.cpp src/stl_aeplanner_viz.cpp src/param.cpp)
target_compile_options(stl_aeplanner PRIVATE ${OpenMP_FLAGS})
target_link_libraries(stl_aeplanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenMP_LIBS}
)
add_dependencies(stl_aeplanner
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  stl_aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

add_executable(stl_aeplanner_node src/stl_aeplanner_node.cpp src/stl_aeplanner.cpp src/stl_aeplanner_viz.cpp src/param.cpp)
target_compile_options(stl_aeplanner_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(stl_aeplanner_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenMP_LIBS}
)
add_dependencies(stl_aeplanner_node
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  stl_aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
