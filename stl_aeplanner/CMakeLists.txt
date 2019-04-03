cmake_minimum_required(VERSION 2.8.3)
project(aeplanner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  aeplanner_msgs
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
#   FILES aeplanner.action
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
  cfg/LTL.cfg
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES aeplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenMP_LIBS}
  CATKIN_DEPENDS aeplanner_msgs dd_gazebo_plugins message_runtime roscpp geometry_msgs visualization_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(aeplanner src/aeplanner_nodelet.cpp src/aeplanner.cpp src/aeplanner_viz.cpp src/param.cpp)
target_compile_options(aeplanner PRIVATE ${OpenMP_FLAGS})
target_link_libraries(aeplanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenMP_LIBS}
)
add_dependencies(aeplanner
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

add_executable(aeplanner_node src/aeplanner_node.cpp src/aeplanner.cpp src/aeplanner_viz.cpp src/param.cpp)
target_compile_options(aeplanner_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(aeplanner_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenMP_LIBS}
)
add_dependencies(aeplanner_node
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
