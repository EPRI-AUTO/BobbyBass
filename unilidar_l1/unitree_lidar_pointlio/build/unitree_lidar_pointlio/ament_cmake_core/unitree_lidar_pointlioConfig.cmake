# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_unitree_lidar_pointlio_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED unitree_lidar_pointlio_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(unitree_lidar_pointlio_FOUND FALSE)
  elseif(NOT unitree_lidar_pointlio_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(unitree_lidar_pointlio_FOUND FALSE)
  endif()
  return()
endif()
set(_unitree_lidar_pointlio_CONFIG_INCLUDED TRUE)

# output package information
if(NOT unitree_lidar_pointlio_FIND_QUIETLY)
  message(STATUS "Found unitree_lidar_pointlio: 0.0.0 (${unitree_lidar_pointlio_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'unitree_lidar_pointlio' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${unitree_lidar_pointlio_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(unitree_lidar_pointlio_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${unitree_lidar_pointlio_DIR}/${_extra}")
endforeach()
