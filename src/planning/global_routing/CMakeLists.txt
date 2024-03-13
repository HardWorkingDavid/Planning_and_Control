cmake_minimum_required(VERSION 3.0.2)
project(global_routing)

add_compile_options(-std=c++14)

file(GLOB_RECURSE SRC_FILE     
  ${PROJECT_SOURCE_DIR}/src/*.cpp 
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  waypoint_msgs
)
find_package(Eigen3 QUIET)
find_package(OsqpEigen QUIET)

link_directories(
  /usr/local/lib 
)
catkin_package(
  INCLUDE_DIRS include
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(global_routing_node ${SRC_FILE})
target_link_libraries(global_routing_node OsqpEigen::OsqpEigen
                            libosqp.so  
                            libOsqpEigen.so 
                            libqdldl.so
                            osqp adolc ipopt glog)