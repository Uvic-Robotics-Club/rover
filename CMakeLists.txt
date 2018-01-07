cmake_minimum_required(VERSION 2.8.3)
project(rover)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    roscpp
    rospy
)
catkin_python_setup()
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# these are all of the nodes that I can call
#add_executable(wheel_sending src/wheel_sending.cpp src/MiniPID.cpp)
#target_link_libraries(wheel_sending ${catkin_LIBRARIES})

#add_executable(wheel_receiving src/wheel_receiving.cpp)
#target_link_libraries(wheel_receiving ${catkin_LIBRARIES})

#FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/wiringPi)
#add_executable(blink src/blink.cpp)
#target_link_libraries(blink ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
