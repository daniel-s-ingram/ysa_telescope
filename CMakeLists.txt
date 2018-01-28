cmake_minimum_required(VERSION 2.8.3)
project(ysa_telescope)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mpu_reader src/mpu_reader.cpp)
#add_executable(motor_controller src/motor_controller.cpp)

target_link_libraries(mpu_reader ${catkin_LIBRARIES})
#target_link_libraries(motor_controller ${catkin_LIBRARIES} EposCmd)