cmake_minimum_required(VERSION 3.5)
project(japan_aeronautical_electronics_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Service Generation
set(srv_files
  "srv/gyro/jg35fd/Jg35fdCalibrateBiasDrift.srv"
  "srv/gyro/jg35fd/Jg35fdControlCalculate.srv"
  "srv/gyro/jg35fd/Jg35fdControlOutput.srv"
  "srv/gyro/jg35fd/Jg35fdResetAngle.srv"
  "srv/gyro/jg35fd/Jg35fdSetAnalogMode.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_package()