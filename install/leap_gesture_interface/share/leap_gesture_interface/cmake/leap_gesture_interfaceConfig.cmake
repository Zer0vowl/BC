# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_leap_gesture_interface_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED leap_gesture_interface_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(leap_gesture_interface_FOUND FALSE)
  elseif(NOT leap_gesture_interface_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(leap_gesture_interface_FOUND FALSE)
  endif()
  return()
endif()
set(_leap_gesture_interface_CONFIG_INCLUDED TRUE)

# output package information
if(NOT leap_gesture_interface_FIND_QUIETLY)
  message(STATUS "Found leap_gesture_interface: 0.0.0 (${leap_gesture_interface_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'leap_gesture_interface' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${leap_gesture_interface_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(leap_gesture_interface_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${leap_gesture_interface_DIR}/${_extra}")
endforeach()
