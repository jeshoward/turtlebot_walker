cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_walker)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  nav_msgs 
  roscpp 
  sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(
  turtlebot_walker
  src/turtlebot_walker.cc
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  turtlebot_walker
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
add_dependencies(
  turtlebot_walker
  ${turtlebot_walker_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtlebot_walker.cc)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
