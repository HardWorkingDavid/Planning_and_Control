cmake_minimum_required(VERSION 3.0.2)
project(vehicle_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vehicle_description
#  CATKIN_DEPENDS xacro
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch mesh urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)
