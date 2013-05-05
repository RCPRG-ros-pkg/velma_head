cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(PTProxy	src/ptproxy/PTProxy.cpp
								src/ptproxy/nfv2_config.c
								src/ptproxy/serialcomm/serialcomm.cpp
								src/ptproxy/nf/nfv2.c)
rosbuild_add_library(VelocityProfileSpline	src/velocityprofile_spline.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(head_controller src/head_controller.cpp)
#rosbuild_add_executable(iptsc src/iptsc.cpp)
#rosbuild_add_executable(servo src/servo.cpp)
target_link_libraries(head_controller PTProxy VelocityProfileSpline)
rosbuild_add_executable(keyboard_ctrl src/keyboard_ctrl.cpp)

# ###################################################
# Tracker using PS Move
# ###################################################
#include(FindPkgConfig)
#pkg_search_module(PSMOVEAPI psmoveapi)
#find_package(OpenCV)
#include_directories(${PSMOVEAPI_INCLUDE_DIRS})
#rosbuild_add_executable(tracker src/tracker.cpp)
#target_link_libraries(tracker ${PSMOVEAPI_LIBRARIES} ${OpenCV_LIBS})

