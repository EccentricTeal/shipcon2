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

add_executable(motor_cpp
  src/motor/cpp_motor.cc
  src/motor/cpp_motor_node.cc
)
ament_target_dependencies(motor_cpp
  rclcpp
  hardware_communication_lib
  shipcon_msgs
  unit_convert_lib
)
target_include_directories(motor_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(motor_normal
  src/motor/normal_motor.cc
  src/motor/normal_motor_node.cc
)
ament_target_dependencies(motor_normal
  rclcpp
  hardware_communication_lib
  shipcon_msgs
  unit_convert_lib
)
target_include_directories(motor_normal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS motor_cpp motor_normal
  DESTINATION lib/${PROJECT_NAME}
  #RUNTIME DESTINATION bin
)

ament_package()
