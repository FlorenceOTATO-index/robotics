# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_flightgoggles_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED flightgoggles_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(flightgoggles_FOUND FALSE)
  elseif(NOT flightgoggles_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(flightgoggles_FOUND FALSE)
  endif()
  return()
endif()
set(_flightgoggles_CONFIG_INCLUDED TRUE)

# output package information
if(NOT flightgoggles_FIND_QUIETLY)
  message(STATUS "Found flightgoggles: 0.0.0 (${flightgoggles_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'flightgoggles' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT flightgoggles_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(flightgoggles_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${flightgoggles_DIR}/${_extra}")
endforeach()
