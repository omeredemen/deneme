# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sbus_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sbus_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sbus_FOUND FALSE)
  elseif(NOT sbus_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sbus_FOUND FALSE)
  endif()
  return()
endif()
set(_sbus_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sbus_FIND_QUIETLY)
  message(STATUS "Found sbus: 0.0.0 (${sbus_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sbus' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sbus_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sbus_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sbus_DIR}/${_extra}")
endforeach()
