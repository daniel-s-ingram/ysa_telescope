cmake_minimum_required(VERSION 2.8.3)
project(ysa_telescope)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})