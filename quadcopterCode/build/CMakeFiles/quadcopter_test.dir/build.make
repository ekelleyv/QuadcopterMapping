# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/build

# Include any dependencies generated for this target.
include CMakeFiles/quadcopter_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quadcopter_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quadcopter_test.dir/flags.make

CMakeFiles/quadcopter_test.dir/src/main.o: CMakeFiles/quadcopter_test.dir/flags.make
CMakeFiles/quadcopter_test.dir/src/main.o: ../src/main.cpp
CMakeFiles/quadcopter_test.dir/src/main.o: ../manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/ardrone_autonomy/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/quadcopter_test.dir/src/main.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/quadcopter_test.dir/src/main.o: /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/ardrone_autonomy/msg_gen/generated
CMakeFiles/quadcopter_test.dir/src/main.o: /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/ardrone_autonomy/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadcopter_test.dir/src/main.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/quadcopter_test.dir/src/main.o -c /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/src/main.cpp

CMakeFiles/quadcopter_test.dir/src/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadcopter_test.dir/src/main.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/src/main.cpp > CMakeFiles/quadcopter_test.dir/src/main.i

CMakeFiles/quadcopter_test.dir/src/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadcopter_test.dir/src/main.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/src/main.cpp -o CMakeFiles/quadcopter_test.dir/src/main.s

CMakeFiles/quadcopter_test.dir/src/main.o.requires:
.PHONY : CMakeFiles/quadcopter_test.dir/src/main.o.requires

CMakeFiles/quadcopter_test.dir/src/main.o.provides: CMakeFiles/quadcopter_test.dir/src/main.o.requires
	$(MAKE) -f CMakeFiles/quadcopter_test.dir/build.make CMakeFiles/quadcopter_test.dir/src/main.o.provides.build
.PHONY : CMakeFiles/quadcopter_test.dir/src/main.o.provides

CMakeFiles/quadcopter_test.dir/src/main.o.provides.build: CMakeFiles/quadcopter_test.dir/src/main.o

# Object files for target quadcopter_test
quadcopter_test_OBJECTS = \
"CMakeFiles/quadcopter_test.dir/src/main.o"

# External object files for target quadcopter_test
quadcopter_test_EXTERNAL_OBJECTS =

../bin/quadcopter_test: CMakeFiles/quadcopter_test.dir/src/main.o
../bin/quadcopter_test: CMakeFiles/quadcopter_test.dir/build.make
../bin/quadcopter_test: CMakeFiles/quadcopter_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/quadcopter_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadcopter_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quadcopter_test.dir/build: ../bin/quadcopter_test
.PHONY : CMakeFiles/quadcopter_test.dir/build

CMakeFiles/quadcopter_test.dir/requires: CMakeFiles/quadcopter_test.dir/src/main.o.requires
.PHONY : CMakeFiles/quadcopter_test.dir/requires

CMakeFiles/quadcopter_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quadcopter_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quadcopter_test.dir/clean

CMakeFiles/quadcopter_test.dir/depend:
	cd /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/build /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/build /home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/build/CMakeFiles/quadcopter_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quadcopter_test.dir/depend

