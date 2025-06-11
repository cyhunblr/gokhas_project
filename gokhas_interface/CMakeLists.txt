cmake_minimum_required(VERSION 3.0.2)
project(gokhas_interface)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  gokhas_communication
  gokhas_perception
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs
    gokhas_communication
    gokhas_perception
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts with proper structure
catkin_install_python(PROGRAMS
  scripts/src/ui/main_window.py
  scripts/src/ui/control_handlers.py
  scripts/src/ui/custom_widgets.py
  scripts/src/ros/ros_bridge.py
  scripts/src/com/publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python modules (non-executable)
install(DIRECTORY scripts/src/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/src
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install resource files (icons, images, styles)
install(DIRECTORY scripts/src/ui/resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
  FILES_MATCHING 
    PATTERN "*.png"
    PATTERN "*.jpg"
    PATTERN "*.qss"
    PATTERN "*.css"
)

# Install launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
  )
endif()

# Install config files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
  )
endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add test files here if you have any
  # catkin_add_nosetests(test/test_gokhas_interface.py)
endif()