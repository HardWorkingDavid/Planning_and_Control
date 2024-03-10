cmake_minimum_required(VERSION 3.0.2)
project(vehicle_state)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vehicle_state
#  CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_model_node src/vehicle_state.cpp)
target_link_libraries(vehicle_model_node ${catkin_LIBRARIES})