cmake_minimum_required(VERSION 3.0.2)
project(gokhas_perception)
set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  message_filters
  message_generation
  message_runtime
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  vision_msgs
  visualization_msgs
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  YoloResult.msg
  Detection2D.msg
  Detection2DArray.msg
)

add_service_files(
  FILES
  PixelTo3D.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  vision_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    image_transport
    message_filters
    message_runtime
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
    vision_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(tracker_with_cloud_node src/tracker_with_cloud_node.cpp)
target_link_libraries(tracker_with_cloud_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(tracker_with_cloud_node gokhas_perception_generate_messages_cpp)

install(PROGRAMS
  script/tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS tracker_with_cloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
