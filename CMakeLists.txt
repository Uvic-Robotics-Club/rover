cmake_minimum_required(VERSION 2.8.3)
project(rover)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
    rospy
    roscpp
    serial
)

## Generate messages in the 'msg' folder
 #add_message_files(
#   FILES
#   motor_velocities.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

include_directories(${catkin_INCLUDE_DIRS})
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

# these are all of the nodes that I can call
#add_executable(joy_to_motor_teleop src/joy_to_motor_teleop.cpp)
#target_link_libraries(joy_to_motor_teleop ${catkin_LIBRARIES})



#add_executable(
#  direct_usb_motor_control
#  src/direct_motor_control.cpp
#  src/motor.cpp)
#target_link_libraries(direct_usb_motor_control ${catkin_LIBRARIES})
