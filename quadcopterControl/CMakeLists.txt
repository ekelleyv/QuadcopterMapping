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

rosbuild_add_boost_directories()
set(SDK ARDroneLib/)
link_directories(${PROJECT_SOURCE_DIR}/lib/)
include_directories(${SDK} ${SDK}/FFMPEG/Includes ${SDK}/Soft/Common ${SDK}/Soft/Lib ${SDK}/VP_SDK ${SDK}/VP_SDK/VP_Os/linux )
rosbuild_add_executable(quadcopter_control src/main.cpp)
#target_link_libraries(ardrone_driver vlib pc_ardrone sdk SDL avcodec avdevice avfilter avformat avutil swscale)
target_link_libraries(quadcopter_control pc_ardrone avcodec avdevice avfilter avformat pc_ardrone_notool avutil swscale vlib  sdk SDL )

