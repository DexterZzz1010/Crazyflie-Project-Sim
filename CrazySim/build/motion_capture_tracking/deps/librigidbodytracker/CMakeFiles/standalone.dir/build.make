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
CMAKE_SOURCE_DIR = /home/zyf/crazyflies/CrazySim/ros2_ws/src/motion_capture_tracking/motion_capture_tracking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking

# Include any dependencies generated for this target.
include deps/librigidbodytracker/CMakeFiles/standalone.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include deps/librigidbodytracker/CMakeFiles/standalone.dir/compiler_depend.make

# Include the progress variables for this target.
include deps/librigidbodytracker/CMakeFiles/standalone.dir/progress.make

# Include the compile flags for this target's objects.
include deps/librigidbodytracker/CMakeFiles/standalone.dir/flags.make

deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.o: deps/librigidbodytracker/CMakeFiles/standalone.dir/flags.make
deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.o: /home/zyf/crazyflies/CrazySim/ros2_ws/src/motion_capture_tracking/motion_capture_tracking/deps/librigidbodytracker/src/standalone.cpp
deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.o: deps/librigidbodytracker/CMakeFiles/standalone.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.o"
	cd /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/deps/librigidbodytracker && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.o -MF CMakeFiles/standalone.dir/src/standalone.cpp.o.d -o CMakeFiles/standalone.dir/src/standalone.cpp.o -c /home/zyf/crazyflies/CrazySim/ros2_ws/src/motion_capture_tracking/motion_capture_tracking/deps/librigidbodytracker/src/standalone.cpp

deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/standalone.dir/src/standalone.cpp.i"
	cd /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/deps/librigidbodytracker && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zyf/crazyflies/CrazySim/ros2_ws/src/motion_capture_tracking/motion_capture_tracking/deps/librigidbodytracker/src/standalone.cpp > CMakeFiles/standalone.dir/src/standalone.cpp.i

deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/standalone.dir/src/standalone.cpp.s"
	cd /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/deps/librigidbodytracker && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zyf/crazyflies/CrazySim/ros2_ws/src/motion_capture_tracking/motion_capture_tracking/deps/librigidbodytracker/src/standalone.cpp -o CMakeFiles/standalone.dir/src/standalone.cpp.s

# Object files for target standalone
standalone_OBJECTS = \
"CMakeFiles/standalone.dir/src/standalone.cpp.o"

# External object files for target standalone
standalone_EXTERNAL_OBJECTS =

deps/librigidbodytracker/standalone: deps/librigidbodytracker/CMakeFiles/standalone.dir/src/standalone.cpp.o
deps/librigidbodytracker/standalone: deps/librigidbodytracker/CMakeFiles/standalone.dir/build.make
deps/librigidbodytracker/standalone: deps/librigidbodytracker/liblibrigidbodytracker.a
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_people.so
deps/librigidbodytracker/standalone: /usr/lib/libOpenNI.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_features.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_search.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_io.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpng.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libz.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libpcl_common.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.74.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.74.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0
deps/librigidbodytracker/standalone: /usr/lib/libOpenNI.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkIOCore-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libfreetype.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkIOImage-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkRenderingUI-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkkissfft-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libGLEW.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libX11.so
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.15.3
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libtbb.so.12.5
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libvtksys-9.1.so.9.1.0
deps/librigidbodytracker/standalone: /usr/lib/x86_64-linux-gnu/libqhull_r.so.8.0.2
deps/librigidbodytracker/standalone: deps/librigidbodytracker/CMakeFiles/standalone.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable standalone"
	cd /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/deps/librigidbodytracker && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/standalone.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
deps/librigidbodytracker/CMakeFiles/standalone.dir/build: deps/librigidbodytracker/standalone
.PHONY : deps/librigidbodytracker/CMakeFiles/standalone.dir/build

deps/librigidbodytracker/CMakeFiles/standalone.dir/clean:
	cd /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/deps/librigidbodytracker && $(CMAKE_COMMAND) -P CMakeFiles/standalone.dir/cmake_clean.cmake
.PHONY : deps/librigidbodytracker/CMakeFiles/standalone.dir/clean

deps/librigidbodytracker/CMakeFiles/standalone.dir/depend:
	cd /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zyf/crazyflies/CrazySim/ros2_ws/src/motion_capture_tracking/motion_capture_tracking /home/zyf/crazyflies/CrazySim/ros2_ws/src/motion_capture_tracking/motion_capture_tracking/deps/librigidbodytracker /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/deps/librigidbodytracker /home/zyf/crazyflies/CrazySim/ros2_ws/build/motion_capture_tracking/deps/librigidbodytracker/CMakeFiles/standalone.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : deps/librigidbodytracker/CMakeFiles/standalone.dir/depend

