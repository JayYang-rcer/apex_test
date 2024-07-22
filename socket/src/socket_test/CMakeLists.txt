cmake_minimum_required(VERSION 3.0.2)
project(socket_test)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socket_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


add_library(head_udp_src
  include/socket_test/server_pack.h
  src/udp_test/server_pack.cpp
)
add_dependencies(head_udp_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_udp_src
  ${catkin_LIBRARIES}
)

add_executable(server_pack src/udp_test/server_pack.cpp)


add_executable(client src/udp_test/client.cpp)
target_link_libraries(client
  ${catkin_LIBRARIES}
)
