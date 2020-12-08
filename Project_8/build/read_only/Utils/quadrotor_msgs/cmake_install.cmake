# Install script for directory: /home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhanhao/Project/src/__MACOSX/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/msg" TYPE FILE FILES
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/AuxCommand.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/Corrections.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/Gains.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/OutputData.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/PositionCommand.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/PPROutputData.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/Serial.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/SO3Command.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/StatusData.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/TRPYCommand.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/Odometry.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/PolynomialTrajectory.msg"
    "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg/LQRTrajectory.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/home/zhanhao/Project/src/__MACOSX/build/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zhanhao/Project/src/__MACOSX/devel/include/quadrotor_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zhanhao/Project/src/__MACOSX/devel/share/roseus/ros/quadrotor_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zhanhao/Project/src/__MACOSX/devel/share/common-lisp/ros/quadrotor_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zhanhao/Project/src/__MACOSX/devel/share/gennodejs/ros/quadrotor_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/zhanhao/Project/src/__MACOSX/devel/lib/python2.7/dist-packages/quadrotor_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zhanhao/Project/src/__MACOSX/devel/lib/python2.7/dist-packages/quadrotor_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhanhao/Project/src/__MACOSX/build/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/home/zhanhao/Project/src/__MACOSX/build/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES
    "/home/zhanhao/Project/src/__MACOSX/build/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig.cmake"
    "/home/zhanhao/Project/src/__MACOSX/build/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs" TYPE FILE FILES "/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/package.xml")
endif()

