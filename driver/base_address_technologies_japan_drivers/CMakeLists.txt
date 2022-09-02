cmake_minimum_required(VERSION 3.5)
project(base_address_technologies_japan_drivers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_communication_lib REQUIRED)
find_package(shipcon_msgs REQUIRED)
find_package(unit_convert_lib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(base_address_technologies_japan_msgs REQUIRED)

add_executable(cpp_motor
  src/motor/cpp_motor.cc
  src/motor/cpp_motor_node.cc
)
ament_target_dependencies(cpp_motor
  rclcpp
  hardware_communication_lib
  shipcon_msgs
  unit_convert_lib
)
target_include_directories(cpp_motor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(normal_motor
  src/motor/normal_motor.cc
  src/motor/normal_motor_node.cc
)
ament_target_dependencies(normal_motor
  rclcpp
  hardware_communication_lib
  shipcon_msgs
  unit_convert_lib
)
target_include_directories(normal_motor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(vectwin_rudder
  src/rudder/vectwin_rudder.cc
  src/rudder/vectwin_rudder_node.cc
)
ament_target_dependencies(vectwin_rudder
  rclcpp
  hardware_communication_lib
  shipcon_msgs
  unit_convert_lib
)
target_include_directories(vectwin_rudder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(vectwin_radio_receiver
  src/radio_receiver/vectwin_radio_receiver.cc
  src/radio_receiver/vectwin_radio_receiver_node.cc
)
ament_target_dependencies(vectwin_radio_receiver
  rclcpp
  hardware_communication_lib
  shipcon_msgs
  unit_convert_lib
  sensor_msgs
  base_address_technologies_japan_msgs
)
target_include_directories(vectwin_radio_receiver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERF
)

install(TARGETS
  cpp_motor
  normal_motor
  vectwin_rudder
  vectwin_radio_receiver
  
  DESTINATION lib/${PROJECT_NAME}
  #RUNTIME DESTINATION bin
)

ament_package()
