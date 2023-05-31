cmake_minimum_required(VERSION 3.1.2)
project(swarm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
# add_definitions(-Og)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_lib
  mrs_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  dynamic_reconfigure
  )

# messages for MPC_TRACKER
add_message_files(DIRECTORY msg FILES
  UserParams.msg
  Swarming.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/param_server.cfg
  )

set(LIBRARIES
  Swarm
  SwarmAio
  ParamServer
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp nodelet mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs message_runtime visualization_msgs message_generation
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# SwarmAio

add_library(SwarmAio
  src/swarm_aio.cpp
  )

target_link_libraries(SwarmAio
  ${catkin_LIBRARIES}
  )

add_dependencies(SwarmAio
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Swarm

add_library(Swarm
  src/swarm.cpp
  )

target_link_libraries(Swarm
  ${catkin_LIBRARIES}
  )

add_dependencies(Swarm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# ParamServer

add_library(ParamServer
  src/param_server.cpp
  )

target_link_libraries(ParamServer
  ${catkin_LIBRARIES}
  )

add_dependencies(ParamServer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )


