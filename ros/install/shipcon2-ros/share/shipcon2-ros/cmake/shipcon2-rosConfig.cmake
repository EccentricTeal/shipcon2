# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_shipcon2-ros_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED shipcon2-ros_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(shipcon2-ros_FOUND FALSE)
  elseif(NOT shipcon2-ros_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(shipcon2-ros_FOUND FALSE)
  endif()
  return()
endif()
set(_shipcon2-ros_CONFIG_INCLUDED TRUE)

# output package information
if(NOT shipcon2-ros_FIND_QUIETLY)
  message(STATUS "Found shipcon2-ros: 0.0.1 (${shipcon2-ros_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'shipcon2-ros' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(shipcon2-ros_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${shipcon2-ros_DIR}/${_extra}")
endforeach()
