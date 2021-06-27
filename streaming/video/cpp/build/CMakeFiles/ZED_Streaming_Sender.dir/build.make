# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/avatarquest/catkin_ws/src/streaming/video/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/avatarquest/catkin_ws/src/streaming/video/cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/ZED_Streaming_Sender.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ZED_Streaming_Sender.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ZED_Streaming_Sender.dir/flags.make

CMakeFiles/ZED_Streaming_Sender.dir/src/main.o: CMakeFiles/ZED_Streaming_Sender.dir/flags.make
CMakeFiles/ZED_Streaming_Sender.dir/src/main.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avatarquest/catkin_ws/src/streaming/video/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ZED_Streaming_Sender.dir/src/main.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ZED_Streaming_Sender.dir/src/main.o -c /home/avatarquest/catkin_ws/src/streaming/video/cpp/src/main.cpp

CMakeFiles/ZED_Streaming_Sender.dir/src/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ZED_Streaming_Sender.dir/src/main.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avatarquest/catkin_ws/src/streaming/video/cpp/src/main.cpp > CMakeFiles/ZED_Streaming_Sender.dir/src/main.i

CMakeFiles/ZED_Streaming_Sender.dir/src/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ZED_Streaming_Sender.dir/src/main.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avatarquest/catkin_ws/src/streaming/video/cpp/src/main.cpp -o CMakeFiles/ZED_Streaming_Sender.dir/src/main.s

CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.requires:

.PHONY : CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.requires

CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.provides: CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.requires
	$(MAKE) -f CMakeFiles/ZED_Streaming_Sender.dir/build.make CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.provides.build
.PHONY : CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.provides

CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.provides.build: CMakeFiles/ZED_Streaming_Sender.dir/src/main.o


# Object files for target ZED_Streaming_Sender
ZED_Streaming_Sender_OBJECTS = \
"CMakeFiles/ZED_Streaming_Sender.dir/src/main.o"

# External object files for target ZED_Streaming_Sender
ZED_Streaming_Sender_EXTERNAL_OBJECTS =

ZED_Streaming_Sender: CMakeFiles/ZED_Streaming_Sender.dir/src/main.o
ZED_Streaming_Sender: CMakeFiles/ZED_Streaming_Sender.dir/build.make
ZED_Streaming_Sender: /usr/local/zed/lib/libsl_zed.so
ZED_Streaming_Sender: /usr/lib/aarch64-linux-gnu/libopenblas.so
ZED_Streaming_Sender: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
ZED_Streaming_Sender: /usr/lib/aarch64-linux-gnu/libcuda.so
ZED_Streaming_Sender: /usr/local/cuda-10.2/lib64/libcudart.so
ZED_Streaming_Sender: CMakeFiles/ZED_Streaming_Sender.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/avatarquest/catkin_ws/src/streaming/video/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ZED_Streaming_Sender"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ZED_Streaming_Sender.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ZED_Streaming_Sender.dir/build: ZED_Streaming_Sender

.PHONY : CMakeFiles/ZED_Streaming_Sender.dir/build

CMakeFiles/ZED_Streaming_Sender.dir/requires: CMakeFiles/ZED_Streaming_Sender.dir/src/main.o.requires

.PHONY : CMakeFiles/ZED_Streaming_Sender.dir/requires

CMakeFiles/ZED_Streaming_Sender.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ZED_Streaming_Sender.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ZED_Streaming_Sender.dir/clean

CMakeFiles/ZED_Streaming_Sender.dir/depend:
	cd /home/avatarquest/catkin_ws/src/streaming/video/cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/avatarquest/catkin_ws/src/streaming/video/cpp /home/avatarquest/catkin_ws/src/streaming/video/cpp /home/avatarquest/catkin_ws/src/streaming/video/cpp/build /home/avatarquest/catkin_ws/src/streaming/video/cpp/build /home/avatarquest/catkin_ws/src/streaming/video/cpp/build/CMakeFiles/ZED_Streaming_Sender.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ZED_Streaming_Sender.dir/depend

