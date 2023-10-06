# Install script for directory: /home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bfl/pdf" TYPE FILE FILES
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/pdf.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/conditionalpdf.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/discretepdf.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/discreteconditionalpdf.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/mcpdf.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/mcpdf.cpp"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/gaussian.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/uniform.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/conditionalgaussian.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/conditionalgaussian_additivenoise.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/analyticconditionalgaussian.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/analyticconditionalgaussian_additivenoise.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/linearanalyticconditionalgaussian.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/filterproposaldensity.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/EKF_proposaldensity.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/mixture.h"
    "/home/wangxiao/catkin_ws/src/agv_sim/orocos-bfl/src/pdf/mixture.cpp"
    )
endif()

