cmake_minimum_required(VERSION 2.8)

project(nimbro_topic_transport)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rqt_gui_cpp
  std_msgs
  std_srvs
  topic_tools
  mrs_lib
)

add_message_files(FILES
  CompressedMsg.msg
  ReceiverStats.msg
  SenderStats.msg
  TopicBandwidth.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rqt_gui_cpp std_msgs std_srvs topic_tools
)
include_directories(${catkin_INCLUDE_DIRS})

# Some optional components of the NimbRo software framework
find_package(plot_msgs QUIET)
if(plot_msgs_FOUND)
  message(STATUS "plot_msgs found, enabling its support in ${PROJECT_NAME}")
  include_directories(${plot_msgs_INCLUDE_DIRS})
  add_definitions(-DWITH_PLOTTING=1)
else()
  message(STATUS "plot_msgs NOT found, not enabling its support in ${PROJECT_NAME}")
endif()

find_package(config_server QUIET)
if(config_server_FOUND)
  message(STATUS "config_server found, enabling its support in ${PROJECT_NAME}")
  include_directories(${config_server_INCLUDE_DIRS})
  add_definitions(-DWITH_CONFIG_SERVER=1)
else()
  message(STATUS "config_server NOT found, not enabling its support in ${PROJECT_NAME}")
endif()

include(ExternalProject)
ExternalProject_Add(
  openfec_lib
  URL ${CMAKE_CURRENT_SOURCE_DIR}/lib/openfec_v1_4_2.tgz
  URL_MD5 c4f8b0aa3e9352f2e713e3db2885ea1c
  BUILD_COMMAND make
  INSTALL_COMMAND cp "<SOURCE_DIR>/bin/Release/libopenfec.so" "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so.1.4.2" && ln -sf "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so.1.4.2" "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so.1" && ln -sf "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so.1" "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so"
  PATCH_COMMAND sed -i.bak "s/FLAGS \"-O4\"/FLAGS \"-O4 -w\"/" CMakeLists.txt
)
ExternalProject_Get_Property(openfec_lib SOURCE_DIR)
set(OPENFEC_PATH "${SOURCE_DIR}")
add_definitions(-DWITH_OPENFEC=1)
include_directories(${OPENFEC_PATH}/src/lib_common)
link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
set(OPENFEC_LIBRARY openfec)
install(FILES
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so.1.4.2
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so.1
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libopenfec.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
message(STATUS "Found and using OpenFEC at: ${OPENFEC_PATH}")

find_library(BZ2_LIBRARY bz2 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD libzstd REQUIRED)
add_definitions(-DWITH_ZSTD=1)
include_directories(${ZSTD_INCLUDE_DIRS})

add_compile_options("-Wall")

add_library(topic_info src/topic_info.cpp)
target_link_libraries(topic_info ${catkin_LIBRARIES})

add_executable(udp_sender
  src/udp/topic_sender.cpp
  src/udp/udp_sender.cpp
)
add_dependencies(udp_sender openfec_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(udp_sender topic_info ${catkin_LIBRARIES} ${BZ2_LIBRARY} ${OPENFEC_LIBRARY} ${ZSTD_LIBRARIES} ${config_server_LIBRARIES})

add_executable(udp_receiver
  src/udp/topic_receiver.cpp
  src/udp/udp_receiver.cpp
)
add_dependencies(udp_receiver openfec_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(udp_receiver topic_info ${catkin_LIBRARIES} ${BZ2_LIBRARY} ${OPENFEC_LIBRARY} ${ZSTD_LIBRARIES})

add_executable(tcp_sender
  src/tcp/tcp_sender.cpp
)
add_dependencies(tcp_sender ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tcp_sender topic_info ${catkin_LIBRARIES} ${BZ2_LIBRARY} ${ZSTD_LIBRARIES} ${config_server_LIBRARIES})

add_executable(tcp_receiver
  src/tcp/tcp_receiver.cpp
)
add_dependencies(tcp_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tcp_receiver topic_info ${catkin_LIBRARIES} ${BZ2_LIBRARY} ${ZSTD_LIBRARIES})

# Tools
add_executable(action_proxy
  src/action_proxy.cpp
  )
target_link_libraries(action_proxy topic_info ${catkin_LIBRARIES})

# GUI
find_package(Qt5 REQUIRED COMPONENTS PrintSupport Widgets)
find_package(QCustomPlot REQUIRED)
set(QT_LIBRARIES Qt5::PrintSupport Qt5::Widgets)

qt5_wrap_cpp(MOC_SRCS
  src/gui/dot_widget.h
  src/gui/topic_gui.h
)

qt5_wrap_cpp(BAND_MOC_SRCS
  src/gui/bandwidth_gui.h
)

add_library(topic_gui
  ${MOC_SRCS}
  src/gui/dot_widget.cpp
  src/gui/topic_gui.cpp
)

add_library(bandwidth_gui
  ${BAND_MOC_SRCS}
  src/gui/bandwidth_gui.cpp
)

add_dependencies(topic_gui ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(topic_gui ${QT_LIBRARIES})

add_dependencies(bandwidth_gui ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(bandwidth_gui ${QT_LIBRARIES} ${QCustomPlot_LIBRARIES} yaml-cpp)

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(catch_ros REQUIRED)
  find_package(rostest REQUIRED)
  include_directories(${catch_ros_INCLUDE_DIRS})

  catch_add_rostest_node(test_comm
    test/test_comm.cpp
  )
  target_link_libraries(test_comm ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest(test/topic_transport.test ARGS port:=5778 protocol:=udp)
  add_rostest(test/topic_transport.test ARGS port:=5779 protocol:=tcp)

  if(OPENFEC_PATH)
    add_rostest(test/topic_transport.test ARGS port:=5780 protocol:=udp fec:=true)
  endif()

  catch_add_rostest_node(test_bidirectional
    test/test_bidirectional.cpp
  )
  target_link_libraries(test_bidirectional ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest(test/bidirectional.test ARGS allow_bidirectional:=true)
  add_rostest(test/bidirectional.test ARGS allow_bidirectional:=false)
endif()

#install
install(PROGRAMS
  src/tcp/tcp_receiver_latch_helper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS action_proxy tcp_receiver tcp_sender topic_info udp_receiver udp_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  bandwidth_plugin.xml rqt_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
