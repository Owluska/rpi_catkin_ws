# Install script for directory: /home/pi/catkin_ws/src/bfl-release/examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pi/catkin_ws/devel_isolated/bfl")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/pi/catkin_ws/build_isolated/bfl/devel/examples/compare_filters/cmake_install.cmake")
  include("/home/pi/catkin_ws/build_isolated/bfl/devel/examples/linear_kalman/cmake_install.cmake")
  include("/home/pi/catkin_ws/build_isolated/bfl/devel/examples/nonlinear_kalman/cmake_install.cmake")
  include("/home/pi/catkin_ws/build_isolated/bfl/devel/examples/nonlinear_particle/cmake_install.cmake")
  include("/home/pi/catkin_ws/build_isolated/bfl/devel/examples/smoother/cmake_install.cmake")
  include("/home/pi/catkin_ws/build_isolated/bfl/devel/examples/discrete_filter/cmake_install.cmake")

endif()
