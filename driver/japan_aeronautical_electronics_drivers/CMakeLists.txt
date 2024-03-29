cmake_minimum_required(VERSION 3.5)
project(japan_aeronautical_electronics_drivers)

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
find_package(unit_convert_lib REQUIRED)
find_package(japan_aeronautical_electronics_msgs REQUIRED)
find_package(hardware_communication_lib REQUIRED)
find_package(shipcon_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex)

add_executable(jg35fd
  src/gyro/jg35fd.cc
  src/gyro/jg35fd_node.cc
)
ament_target_dependencies(jg35fd
  rclcpp
  unit_convert_lib
  japan_aeronautical_electronics_msgs
  hardware_communication_lib
  shipcon_msgs
  Boost
)
target_include_directories(jg35fd
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS jg35fd
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()