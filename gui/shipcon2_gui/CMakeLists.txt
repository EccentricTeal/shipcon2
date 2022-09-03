cmake_minimum_required(VERSION 3.5)
project(shipcon2_gui)

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
find_package(rviz_common REQUIRED)
find_package(pluginlib)
find_package(Qt5 REQUIRED COMPONENTS Widgets Svg)

#Qt Settings
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )
set( CMAKE_AUTORCC ON ) 
macro( qt_wrap_ui )
  qt5_wrap_ui( ${ARGN} )
endmacro()
include_directories( ${Qt5Core_INCLUDE_DIRS} )
include_directories( ${Qt5Widgets_INCLUDE_DIRS} )
include_directories( src ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging )


add_library(single_engine SHARED
  src/standard/single_engine.cc
)
ament_target_dependencies(single_engine
  rclcpp
  rviz_common
)
target_include_directories(single_engine
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(single_engine
  Qt5::Core
  Qt5::Svg
)
pluginlib_export_plugin_description_file(rviz_common
  plugin_description.xml
)


install(DIRECTORY img
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
    single_engine
  ARCHIVE
    DESTINATION lib
  LIBRARY
    DESTINATION lib
  RUNTIME
    DESTINATION bin
)


ament_package()