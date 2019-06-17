cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  wsg_50_common
  actionlib
  actionlib_msgs
  control_msgs
  message_generation
  sensor_msgs
  #  serial
)

catkin_package(
   INCLUDE_DIRS include
  LIBRARIES hand_lib optical_weiss_finger_lib fmf_weiss_finger_lib
  CATKIN_DEPENDS roscpp std_msgs std_srvs wsg_50_common actionlib_msgs control_msgs actionlib sensor_msgs
#  DEPENDS system_lib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# WSG_50_TCP version
set(DRIVER_SOURCES
  src/checksum.cpp include/wsg_50/checksum.h
  src/cmd.c include/wsg_50/cmd.h
  src/common.cpp include/wsg_50/common.h
  src/functions.cpp include/wsg_50/functions.h
  src/interface.cpp include/wsg_50/interface.h
#  src/main.cpp
  src/msg.c include/wsg_50/msg.h
  src/serial.c include/wsg_50/serial.h
  src/tcp.c include/wsg_50/tcp.h
  src/udp.c include/wsg_50/udp.h)

# WSG_50_CAN version
set(DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DOSNAME_LINUX)
add_definitions(-g)

# C++11 Needed
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_library(optical_weiss_finger_lib src/optical_weiss_finger.cpp)
target_link_libraries(optical_weiss_finger_lib ${catkin_LIBRARIES})

add_library(fmf_weiss_finger_lib src/fmf_weiss_finger.cpp)
target_link_libraries(fmf_weiss_finger_lib ${catkin_LIBRARIES})

add_library(hand_lib src/hand.cpp ${DRIVER_SOURCES})
target_link_libraries(hand_lib optical_weiss_finger_lib fmf_weiss_finger_lib ${catkin_LIBRARIES})
add_dependencies(hand_lib wsg_50_common_generate_messages_cpp)

#add_library(uart_comms_lib src/uart_comms.cpp)
#add_dependencies(uart_comms_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(uart_comms_lib ${catkin_LIBRARIES})

#add_library(load_cell_finger_lib src/load_cell_finger.cpp)
#target_link_libraries(load_cell_finger_lib uart_comms_lib ${catkin_LIBRARIES})

#add_executable(optical_weiss_finger src/optical_weiss_finger.cpp)
#target_link_libraries(optical_weiss_finger hand_lib optical_weiss_finger_lib fmf_weiss_finger_lib ${catkin_LIBRARIES})
#add_dependencies(optical_weiss_finger wsg_50_common_generate_messages_cpp)

add_executable(collect_data src/collect_data.cpp)
target_link_libraries(collect_data hand_lib optical_weiss_finger_lib fmf_weiss_finger_lib ${catkin_LIBRARIES})
add_dependencies(collect_data wsg_50_common_generate_messages_cpp)

add_executable(collect_dist_force src/collect_dist_force.cpp)
target_link_libraries(collect_dist_force hand_lib optical_weiss_finger_lib fmf_weiss_finger_lib ${catkin_LIBRARIES})
add_dependencies(collect_dist_force wsg_50_common_generate_messages_cpp)

#add_executable(load_cell_finger_test src/load_cell_finger.cpp)
#target_link_libraries(load_cell_finger_test uart_comms_lib ${catkin_LIBRARIES})

add_executable(hand_test src/hand.cpp ${DRIVER_SOURCES})
target_link_libraries(hand_test optical_weiss_finger_lib fmf_weiss_finger_lib ${catkin_LIBRARIES})
add_dependencies(hand_test wsg_50_common_generate_messages_cpp)

#add_executable(wsg_50_ip src/main.cpp ${DRIVER_SOURCES})
#target_link_libraries(wsg_50_ip ${catkin_LIBRARIES})
#add_dependencies(wsg_50_ip wsg_50_common_generate_messages_cpp)
add_executable(wsg_50_ip src/new/main_action_driver.cpp ${DRIVER_SOURCES})
target_link_libraries(wsg_50_ip ${catkin_LIBRARIES})
add_dependencies(wsg_50_ip wsg_50_common_generate_messages_cpp)

add_executable(spi_test src/spi_test.cpp ${DRIVER_SOURCES})
target_link_libraries(spi_test ${catkin_LIBRARIES})
add_dependencies(spi_test wsg_50_common_generate_messages_cpp)

add_executable(optical_weiss src/optical_weiss.cpp ${DRIVER_SOURCES})
target_link_libraries(optical_weiss ${catkin_LIBRARIES})
add_dependencies(optical_weiss wsg_50_common_generate_messages_cpp)

#add_executable(wsg_50_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)
#add_executable(wsg_50_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

#include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
#link_directories(/home/marc/peak-linux-driver-7.5/lib/)
#add_compile_flags(wsg_50_can -g -Wall)
#target_link_libraries(wsg_50_can pcan)

install(DIRECTORY include/wsg_50}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
