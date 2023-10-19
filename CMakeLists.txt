cmake_minimum_required(VERSION 2.8.3)
project(xbow4x)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  message_generation
  self_test
  diagnostic_updater
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   EulerStamped.msg
)

add_service_files(
  FILES
  broadcast_tf.srv
  send_command.srv
  status.srv
  calibrate.srv
  set_baudrate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS thread system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES xbow4x
 CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    sensor_msgs
    tf2
    self_test
    diagnostic_updater
 # DEPENDS system_lib
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

#file(GLOB_RECURSE HEADERS */*.hpp */*.h)

## Declare a C++ library
add_library(xbow4x
  src/xbow4x.cpp
)

## Declare a C++ executable
add_executable(xbow4x_node src/xbow4x_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(xbow4x_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(xbow4x_node
  xbow4x
  ${catkin_LIBRARIES})

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
