# Install script for directory: /home/student/Asparagus_project/ros_proj/src/beckhoff_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/student/Asparagus_project/ros_proj/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/beckhoff_msgs/msg" TYPE FILE FILES
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/Vector_q5.msg"
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/array5.msg"
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/CmdRobot.msg"
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/JointStateRobot.msg"
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/catReceive.msg"
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/catSend.msg"
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/dataArray.msg"
    "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/msg/CmdTracks.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/beckhoff_msgs/cmake" TYPE FILE FILES "/home/student/Asparagus_project/ros_proj/build/beckhoff_msgs/catkin_generated/installspace/beckhoff_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/student/Asparagus_project/ros_proj/devel/include/beckhoff_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/student/Asparagus_project/ros_proj/devel/share/roseus/ros/beckhoff_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/student/Asparagus_project/ros_proj/devel/share/common-lisp/ros/beckhoff_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/student/Asparagus_project/ros_proj/devel/share/gennodejs/ros/beckhoff_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/student/anaconda3/bin/python3" -m compileall "/home/student/Asparagus_project/ros_proj/devel/lib/python3/dist-packages/beckhoff_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/student/Asparagus_project/ros_proj/devel/lib/python3/dist-packages/beckhoff_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/student/Asparagus_project/ros_proj/build/beckhoff_msgs/catkin_generated/installspace/beckhoff_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/beckhoff_msgs/cmake" TYPE FILE FILES "/home/student/Asparagus_project/ros_proj/build/beckhoff_msgs/catkin_generated/installspace/beckhoff_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/beckhoff_msgs/cmake" TYPE FILE FILES
    "/home/student/Asparagus_project/ros_proj/build/beckhoff_msgs/catkin_generated/installspace/beckhoff_msgsConfig.cmake"
    "/home/student/Asparagus_project/ros_proj/build/beckhoff_msgs/catkin_generated/installspace/beckhoff_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/beckhoff_msgs" TYPE FILE FILES "/home/student/Asparagus_project/ros_proj/src/beckhoff_msgs/package.xml")
endif()

