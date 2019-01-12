cmake_minimum_required(VERSION 2.8.3)
project(safe_footstep_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  jsk_recognition_msgs
  cv_bridge
  image_transport
  message_generation
  tf
  roseus
)

add_message_files(
  FILES
  TargetFoothold.msg
)

add_service_files(
  FILES
  CheckFoothold.srv
  GetTargetFoothold.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

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

add_executable(convert_to_map_coords src/convert_to_map_coords.cpp)
target_link_libraries(convert_to_map_coords ${catkin_LIBRARIES})
