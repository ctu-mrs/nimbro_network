cmake_minimum_required(VERSION 2.8)

project(tf_throttle)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_msgs
  tf2_ros
  )

catkin_package(CATKIN_DEPENDS roscpp tf2 tf2_msgs tf2_ros)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_throttle
  src/tf_throttle.cpp
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS tf_throttle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
