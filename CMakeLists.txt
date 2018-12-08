cmake_minimum_required(VERSION 2.8.3)
project(safe_footstep_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(mask_image src/mask_image.cpp)
# target_link_libraries(mask_image ${catkin_LIBRARIES})
add_executable(rect_publisher src/rect_publisher.cpp)
target_link_libraries(rect_publisher ${catkin_LIBRARIES})
# add_dependencies(mask_image beginner_tutorials_generate_messages_cpp)
