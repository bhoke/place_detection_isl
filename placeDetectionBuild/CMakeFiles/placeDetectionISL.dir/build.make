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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/berkanhoke/berkan_workspace/placeDetectionISL

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/berkanhoke/berkan_workspace/placeDetectionISL/placeDetectionBuild

# Include any dependencies generated for this target.
include CMakeFiles/placeDetectionISL.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/placeDetectionISL.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/placeDetectionISL.dir/flags.make

CMakeFiles/placeDetectionISL.dir/src/main.o: CMakeFiles/placeDetectionISL.dir/flags.make
CMakeFiles/placeDetectionISL.dir/src/main.o: ../src/main.cpp
CMakeFiles/placeDetectionISL.dir/src/main.o: ../manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/placeDetectionISL.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/berkanhoke/berkan_workspace/placeDetectionISL/placeDetectionBuild/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/placeDetectionISL.dir/src/main.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/placeDetectionISL.dir/src/main.o -c /home/berkanhoke/berkan_workspace/placeDetectionISL/src/main.cpp

CMakeFiles/placeDetectionISL.dir/src/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/placeDetectionISL.dir/src/main.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/berkanhoke/berkan_workspace/placeDetectionISL/src/main.cpp > CMakeFiles/placeDetectionISL.dir/src/main.i

CMakeFiles/placeDetectionISL.dir/src/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/placeDetectionISL.dir/src/main.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/berkanhoke/berkan_workspace/placeDetectionISL/src/main.cpp -o CMakeFiles/placeDetectionISL.dir/src/main.s

CMakeFiles/placeDetectionISL.dir/src/main.o.requires:
.PHONY : CMakeFiles/placeDetectionISL.dir/src/main.o.requires

CMakeFiles/placeDetectionISL.dir/src/main.o.provides: CMakeFiles/placeDetectionISL.dir/src/main.o.requires
	$(MAKE) -f CMakeFiles/placeDetectionISL.dir/build.make CMakeFiles/placeDetectionISL.dir/src/main.o.provides.build
.PHONY : CMakeFiles/placeDetectionISL.dir/src/main.o.provides

CMakeFiles/placeDetectionISL.dir/src/main.o.provides.build: CMakeFiles/placeDetectionISL.dir/src/main.o

# Object files for target placeDetectionISL
placeDetectionISL_OBJECTS = \
"CMakeFiles/placeDetectionISL.dir/src/main.o"

# External object files for target placeDetectionISL
placeDetectionISL_EXTERNAL_OBJECTS =

../lib/libplaceDetectionISL.so: CMakeFiles/placeDetectionISL.dir/src/main.o
../lib/libplaceDetectionISL.so: CMakeFiles/placeDetectionISL.dir/build.make
../lib/libplaceDetectionISL.so: CMakeFiles/placeDetectionISL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../lib/libplaceDetectionISL.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/placeDetectionISL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/placeDetectionISL.dir/build: ../lib/libplaceDetectionISL.so
.PHONY : CMakeFiles/placeDetectionISL.dir/build

CMakeFiles/placeDetectionISL.dir/requires: CMakeFiles/placeDetectionISL.dir/src/main.o.requires
.PHONY : CMakeFiles/placeDetectionISL.dir/requires

CMakeFiles/placeDetectionISL.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/placeDetectionISL.dir/cmake_clean.cmake
.PHONY : CMakeFiles/placeDetectionISL.dir/clean

CMakeFiles/placeDetectionISL.dir/depend:
	cd /home/berkanhoke/berkan_workspace/placeDetectionISL/placeDetectionBuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/berkanhoke/berkan_workspace/placeDetectionISL /home/berkanhoke/berkan_workspace/placeDetectionISL /home/berkanhoke/berkan_workspace/placeDetectionISL/placeDetectionBuild /home/berkanhoke/berkan_workspace/placeDetectionISL/placeDetectionBuild /home/berkanhoke/berkan_workspace/placeDetectionISL/placeDetectionBuild/CMakeFiles/placeDetectionISL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/placeDetectionISL.dir/depend

