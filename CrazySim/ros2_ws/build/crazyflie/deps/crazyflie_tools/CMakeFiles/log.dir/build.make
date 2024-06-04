# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/src/crazyswarm2/crazyflie

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie

# Include any dependencies generated for this target.
include deps/crazyflie_tools/CMakeFiles/log.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include deps/crazyflie_tools/CMakeFiles/log.dir/compiler_depend.make

# Include the progress variables for this target.
include deps/crazyflie_tools/CMakeFiles/log.dir/progress.make

# Include the compile flags for this target's objects.
include deps/crazyflie_tools/CMakeFiles/log.dir/flags.make

deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.o: deps/crazyflie_tools/CMakeFiles/log.dir/flags.make
deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.o: /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/src/crazyswarm2/crazyflie/deps/crazyflie_tools/src/log.cpp
deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.o: deps/crazyflie_tools/CMakeFiles/log.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.o"
	cd /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/deps/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.o -MF CMakeFiles/log.dir/src/log.cpp.o.d -o CMakeFiles/log.dir/src/log.cpp.o -c /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/src/crazyswarm2/crazyflie/deps/crazyflie_tools/src/log.cpp

deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/log.dir/src/log.cpp.i"
	cd /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/deps/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/src/crazyswarm2/crazyflie/deps/crazyflie_tools/src/log.cpp > CMakeFiles/log.dir/src/log.cpp.i

deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/log.dir/src/log.cpp.s"
	cd /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/deps/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/src/crazyswarm2/crazyflie/deps/crazyflie_tools/src/log.cpp -o CMakeFiles/log.dir/src/log.cpp.s

# Object files for target log
log_OBJECTS = \
"CMakeFiles/log.dir/src/log.cpp.o"

# External object files for target log
log_EXTERNAL_OBJECTS =

deps/crazyflie_tools/log: deps/crazyflie_tools/CMakeFiles/log.dir/src/log.cpp.o
deps/crazyflie_tools/log: deps/crazyflie_tools/CMakeFiles/log.dir/build.make
deps/crazyflie_tools/log: deps/crazyflie_tools/crazyflie_cpp/libcrazyflie_cpp.a
deps/crazyflie_tools/log: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.74.0
deps/crazyflie_tools/log: deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/libcrazyflieLinkCpp.a
deps/crazyflie_tools/log: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
deps/crazyflie_tools/log: deps/crazyflie_tools/CMakeFiles/log.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable log"
	cd /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/deps/crazyflie_tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/log.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
deps/crazyflie_tools/CMakeFiles/log.dir/build: deps/crazyflie_tools/log
.PHONY : deps/crazyflie_tools/CMakeFiles/log.dir/build

deps/crazyflie_tools/CMakeFiles/log.dir/clean:
	cd /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/deps/crazyflie_tools && $(CMAKE_COMMAND) -P CMakeFiles/log.dir/cmake_clean.cmake
.PHONY : deps/crazyflie_tools/CMakeFiles/log.dir/clean

deps/crazyflie_tools/CMakeFiles/log.dir/depend:
	cd /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/src/crazyswarm2/crazyflie /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/src/crazyswarm2/crazyflie/deps/crazyflie_tools /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/deps/crazyflie_tools /home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws/build/crazyflie/deps/crazyflie_tools/CMakeFiles/log.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : deps/crazyflie_tools/CMakeFiles/log.dir/depend

