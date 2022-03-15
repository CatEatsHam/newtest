cmake_minimum_required(VERSION 3.0.2)
project(tf_tree_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tf_tree_publisher
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf tf2
  DEPENDS 
)

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(tf_tree_publisher_node nodes/tf_tree_publisher_node.cpp src/tfTreeClass.cpp)
target_link_libraries(tf_tree_publisher_node ${catkin_LIBRARIES})

set(EXECUTABLES
tf_tree_publisher_node
)

set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

install(TARGETS
  tf_tree_publisher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include
  USE_SOURCE_PERMISSIONS
  )
  install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(DIRECTORY nodes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)


