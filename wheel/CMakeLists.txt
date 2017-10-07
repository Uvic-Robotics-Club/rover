cmake_minimum_required(VERSION 2.8.3)
project(wheel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  roslaunch
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sending src/sending.cpp src/MiniPID.cpp)
target_link_libraries(sending ${catkin_LIBRARIES})

add_executable(receiving src/receiving.cpp)
target_link_libraries(receiving ${catkin_LIBRARIES})


roslaunch_add_file_check(launch)

