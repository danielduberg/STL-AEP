cmake_minimum_required(VERSION 2.8.3)
project(stl_aeplanner_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)

add_message_files(
  FILES
  LTLStats.msg
  Node.msg
  Volume.msg
)

add_service_files(
  FILES
  BestNode.srv
  Coverage.srv
  Query.srv
  Reevaluate.srv
)

add_action_files(
  FILES
  aeplanner.action
  FlyTo.action
  rrt.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)