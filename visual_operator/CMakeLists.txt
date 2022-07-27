cmake_minimum_required(VERSION 3.0.2)
project(visual_operator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs 
  joy 
)

find_package (OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
  LIBRARIES turtlebot_teleop 
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs geometry_msgs joy 
  
# DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(optical_flow src/optical_flow.cpp)
target_link_libraries(optical_flow ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(my_publisher src/my_publisher.cpp) 
target_link_libraries(my_publisher ${catkin_LIBRARIES}) 
