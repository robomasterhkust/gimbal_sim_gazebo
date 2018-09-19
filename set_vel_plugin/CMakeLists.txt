cmake_minimum_required(VERSION 2.6)
project(SetVelPlugin)

find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS ${GAZEBO_CXX_FLAGS})

add_library(GimbalControlPlugin SHARED src/GimbalControlPlugin.cpp)
target_link_libraries(GimbalControlPlugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
