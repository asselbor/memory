cmake_minimum_required(VERSION 2.8.3)
project(memory)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Activity.msg
  Animation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS nodes
#  LIBRARIES memory
   CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

#catkin_package()
