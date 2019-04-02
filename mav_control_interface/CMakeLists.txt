cmake_minimum_required(VERSION 2.8.3)
project(mav_control_interface)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11 -lpthread)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(libTCP libSerial)

set(SOURCE_FILES
  src/mav_control_interface.cpp
  src/mav_control_interface_impl.cpp

  src/mav_control_interface_cooperative.cpp
  src/mav_control_interface_cooperative_impl.cpp

  src/parameters.cpp

  src/position_controller_interface.cpp
  src/position_controller_interface_cooperative.cpp
  
  src/rc_interface.cpp
  src/rc_interface_aci.cpp
  src/rc_interface_aci_cooperative.cpp

  src/state_machine.cpp
  src/state_machine_cooperative.cpp

  src/mpc_queue.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCE_FILES})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=50 -DBOOST_MPL_LIMIT_MAP_SIZE=50 ")

cs_add_executable(command_converter src/command_converter.cpp)
target_link_libraries(command_converter ${PROJECT_NAME})

cs_add_executable(command_converter_2 src/command_converter_2.cpp)
target_link_libraries(command_converter_2 ${PROJECT_NAME})

cs_add_executable(command_converter_TCPServer src/command_converter_TCPServer.cpp libTCP/TCPServer.cpp)
target_link_libraries(command_converter_TCPServer ${PROJECT_NAME})

cs_add_executable(command_converter_2_TCPClient src/command_converter_2_TCPClient.cpp libTCP/TCPClient.cpp)
target_link_libraries(command_converter_2_TCPClient ${PROJECT_NAME})

cs_add_executable(command_converter_serial src/command_converter_serial.cpp libSerial/uart.c)
target_link_libraries(command_converter_serial ${PROJECT_NAME})

cs_add_executable(command_converter_2_serial src/command_converter_2_serial.cpp libSerial/uart.c)
target_link_libraries(command_converter_2_serial ${PROJECT_NAME})

cs_add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)
target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME})

cs_install()
cs_export()
