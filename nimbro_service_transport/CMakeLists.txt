cmake_minimum_required(VERSION 3.1.2)

project(nimbro_service_transport)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  rqt_gui_cpp
  std_msgs
  topic_tools
  mrs_lib
)

add_message_files(FILES ServiceStatus.msg)
add_service_files(FILES AddTwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp roslib rqt_gui_cpp std_msgs topic_tools
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(service_server
  src/tcp/client_handler.cpp
  src/tcp/service_server.cpp
)
target_link_libraries(service_server ${catkin_LIBRARIES})

add_executable(service_client
  src/common.cpp
  src/tcp/service_client.cpp
)
target_link_libraries(service_client ${catkin_LIBRARIES})
add_dependencies(service_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(udp_server
  src/udp/udp_server.cpp
)
target_link_libraries(udp_server ${catkin_LIBRARIES})

add_executable(udp_client
  src/common.cpp
  src/udp/udp_client.cpp
)
target_link_libraries(udp_client ${catkin_LIBRARIES})
add_dependencies(udp_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

# GUI
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(QT_LIBRARIES Qt5::Widgets)

qt5_wrap_cpp(MOC_SRCS
  src/gui/service_widget.h
)

add_library(service_widget
  ${MOC_SRCS}
  src/gui/service_widget.cpp
)
add_dependencies(service_widget ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(service_widget ${QT_LIBRARIES})

install(PROGRAMS
  scripts/get_md5.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS service_client service_server udp_client udp_server
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
 plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(catch_ros REQUIRED)

  include_directories(${catch_ros_INCLUDE_DIRS})

  add_executable(test_server
    test/test_server.cpp
  )
  target_link_libraries(test_server ${catkin_LIBRARIES} ${rostest_LIBRARIES})
  add_dependencies(test_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

  catch_add_rostest_node(test_client
    test/test_client.cpp
  )
  target_link_libraries(test_client ${rostest_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(test_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

  add_rostest(test/test_udp.test)
  add_rostest(test/test_tcp.test)
endif()
