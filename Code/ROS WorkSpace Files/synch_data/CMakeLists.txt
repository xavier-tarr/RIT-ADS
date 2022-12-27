cmake_minimum_required(VERSION 2.8.3)
project(synch_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  message_filters
  sensor_msgs
  nav_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/flycapture
)

add_executable(Dataset_test src/main.cpp)

target_link_libraries(Dataset_test
   ${catkin_LIBRARIES}
   flycapture
   flycapturevideo
)

