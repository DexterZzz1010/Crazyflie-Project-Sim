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
CMAKE_SOURCE_DIR = /home/zyf/crazyflies/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zyf/crazyflies/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz

# Include any dependencies generated for this target.
include CMakeFiles/gz_crazysim_plugin.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/gz_crazysim_plugin.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/gz_crazysim_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gz_crazysim_plugin.dir/flags.make

CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o: CMakeFiles/gz_crazysim_plugin.dir/flags.make
CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o: /home/zyf/crazyflies/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp
CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o: CMakeFiles/gz_crazysim_plugin.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zyf/crazyflies/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o -MF CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o.d -o CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o -c /home/zyf/crazyflies/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp

CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zyf/crazyflies/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp > CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.i

CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zyf/crazyflies/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp -o CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.s

# Object files for target gz_crazysim_plugin
gz_crazysim_plugin_OBJECTS = \
"CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o"

# External object files for target gz_crazysim_plugin
gz_crazysim_plugin_EXTERNAL_OBJECTS =

libgz_crazysim_plugin.so: CMakeFiles/gz_crazysim_plugin.dir/crazysim_plugin.cpp.o
libgz_crazysim_plugin.so: CMakeFiles/gz_crazysim_plugin.dir/build.make
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_timer.so.1.74.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-sim7.so.7.7.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-fuel_tools8.so.8.1.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-gui7.so.7.2.1
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2-loader.so.2.0.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5QuickControls2.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5Quick.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5QmlModels.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5Qml.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-physics6.so.6.5.1
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2.so.2.0.3
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-rendering7.so.7.4.2
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-profiler.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-events.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-av.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-io.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-testing.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-geospatial.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-graphics.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5.so.5.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat13.so.13.6.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-transport12-log.so.12.2.1
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-transport12-parameters.so.12.2.1
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-transport12.so.12.2.1
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-msgs9.so.9.5.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-math7.so.7.4.0
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libgz_crazysim_plugin.so: /usr/lib/x86_64-linux-gnu/libgz-utils2.so.2.2.0
libgz_crazysim_plugin.so: CMakeFiles/gz_crazysim_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zyf/crazyflies/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libgz_crazysim_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gz_crazysim_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gz_crazysim_plugin.dir/build: libgz_crazysim_plugin.so
.PHONY : CMakeFiles/gz_crazysim_plugin.dir/build

CMakeFiles/gz_crazysim_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gz_crazysim_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gz_crazysim_plugin.dir/clean

CMakeFiles/gz_crazysim_plugin.dir/depend:
	cd /home/zyf/crazyflies/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zyf/crazyflies/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim /home/zyf/crazyflies/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim /home/zyf/crazyflies/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz /home/zyf/crazyflies/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz /home/zyf/crazyflies/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz/CMakeFiles/gz_crazysim_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gz_crazysim_plugin.dir/depend

