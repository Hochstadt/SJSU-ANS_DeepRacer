# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ssh_driver_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ssh_driver_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ssh_driver_FOUND FALSE)
  elseif(NOT ssh_driver_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ssh_driver_FOUND FALSE)
  endif()
  return()
endif()
set(_ssh_driver_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ssh_driver_FIND_QUIETLY)
  message(STATUS "Found ssh_driver: 0.0.0 (${ssh_driver_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ssh_driver' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ssh_driver_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ssh_driver_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ssh_driver_DIR}/${_extra}")
endforeach()
