cmake_minimum_required(VERSION 2.8.3)
project(radar_mr76)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  radar_msgs
  can_msgs
)

find_package(Boost REQUIRED COMPONENTS system )

catkin_package(
    CATKIN_DEPENDS  roscpp std_msgs radar_msgs can_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(radar_publisher src/main.cpp src/mr76.cpp src/radar.cpp)
add_dependencies(radar_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_publisher
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )
