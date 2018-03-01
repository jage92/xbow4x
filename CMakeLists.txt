cmake_minimum_required(VERSION 2.8.3)
project(xbow6x)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)

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
 LIBRARIES xbow6x
 CATKIN_DEPENDS roscpp sensor_msgs serial
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

## Declare a C++ library
add_library(xbow6x
  src/xbow6x.cpp
)

## Declare a C++ executable
add_executable(xbow6x_node src/xbow6x_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(xbow440_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(xbow6x_node
  xbow6x
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
)
