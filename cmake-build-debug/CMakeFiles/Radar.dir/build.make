# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /opt/clion-2019.2.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.2.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/demon/CLionProjects/Radar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/demon/CLionProjects/Radar/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Radar.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Radar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Radar.dir/flags.make

CMakeFiles/Radar.dir/main.cpp.o: CMakeFiles/Radar.dir/flags.make
CMakeFiles/Radar.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Radar.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Radar.dir/main.cpp.o -c /home/demon/CLionProjects/Radar/main.cpp

CMakeFiles/Radar.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Radar.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demon/CLionProjects/Radar/main.cpp > CMakeFiles/Radar.dir/main.cpp.i

CMakeFiles/Radar.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Radar.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demon/CLionProjects/Radar/main.cpp -o CMakeFiles/Radar.dir/main.cpp.s

CMakeFiles/Radar.dir/camera/src/shm.cpp.o: CMakeFiles/Radar.dir/flags.make
CMakeFiles/Radar.dir/camera/src/shm.cpp.o: ../camera/src/shm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Radar.dir/camera/src/shm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Radar.dir/camera/src/shm.cpp.o -c /home/demon/CLionProjects/Radar/camera/src/shm.cpp

CMakeFiles/Radar.dir/camera/src/shm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Radar.dir/camera/src/shm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demon/CLionProjects/Radar/camera/src/shm.cpp > CMakeFiles/Radar.dir/camera/src/shm.cpp.i

CMakeFiles/Radar.dir/camera/src/shm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Radar.dir/camera/src/shm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demon/CLionProjects/Radar/camera/src/shm.cpp -o CMakeFiles/Radar.dir/camera/src/shm.cpp.s

CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.o: CMakeFiles/Radar.dir/flags.make
CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.o: ../my_radar/src/pretreatment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.o -c /home/demon/CLionProjects/Radar/my_radar/src/pretreatment.cpp

CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demon/CLionProjects/Radar/my_radar/src/pretreatment.cpp > CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.i

CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demon/CLionProjects/Radar/my_radar/src/pretreatment.cpp -o CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.s

CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.o: CMakeFiles/Radar.dir/flags.make
CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.o: ../my_radar/src/radar_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.o -c /home/demon/CLionProjects/Radar/my_radar/src/radar_main.cpp

CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demon/CLionProjects/Radar/my_radar/src/radar_main.cpp > CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.i

CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demon/CLionProjects/Radar/my_radar/src/radar_main.cpp -o CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.s

CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.o: CMakeFiles/Radar.dir/flags.make
CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.o: ../my_radar/src/calculate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.o -c /home/demon/CLionProjects/Radar/my_radar/src/calculate.cpp

CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demon/CLionProjects/Radar/my_radar/src/calculate.cpp > CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.i

CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demon/CLionProjects/Radar/my_radar/src/calculate.cpp -o CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.s

CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.o: CMakeFiles/Radar.dir/flags.make
CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.o: ../my_radar/src/video_write.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.o -c /home/demon/CLionProjects/Radar/my_radar/src/video_write.cpp

CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demon/CLionProjects/Radar/my_radar/src/video_write.cpp > CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.i

CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demon/CLionProjects/Radar/my_radar/src/video_write.cpp -o CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.s

# Object files for target Radar
Radar_OBJECTS = \
"CMakeFiles/Radar.dir/main.cpp.o" \
"CMakeFiles/Radar.dir/camera/src/shm.cpp.o" \
"CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.o" \
"CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.o" \
"CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.o" \
"CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.o"

# External object files for target Radar
Radar_EXTERNAL_OBJECTS =

../bin/Radar: CMakeFiles/Radar.dir/main.cpp.o
../bin/Radar: CMakeFiles/Radar.dir/camera/src/shm.cpp.o
../bin/Radar: CMakeFiles/Radar.dir/my_radar/src/pretreatment.cpp.o
../bin/Radar: CMakeFiles/Radar.dir/my_radar/src/radar_main.cpp.o
../bin/Radar: CMakeFiles/Radar.dir/my_radar/src/calculate.cpp.o
../bin/Radar: CMakeFiles/Radar.dir/my_radar/src/video_write.cpp.o
../bin/Radar: CMakeFiles/Radar.dir/build.make
../bin/Radar: /usr/local/lib/libopencv_stitching.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_gapi.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_ccalib.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_line_descriptor.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_face.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_reg.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_rgbd.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_xobjdetect.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_videostab.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_img_hash.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_aruco.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_bgsegm.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_hfs.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_dnn_objdetect.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_fuzzy.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_superres.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_hdf.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_bioinspired.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudaoptflow.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_saliency.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudaobjdetect.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_surface_matching.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_xfeatures2d.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_structured_light.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudabgsegm.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudastereo.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_quality.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_stereo.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudafeatures2d.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_xphoto.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_dpm.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_optflow.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_freetype.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_shape.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudacodec.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_ximgproc.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudawarping.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudalegacy.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_phase_unwrapping.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_tracking.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_video.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_datasets.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_text.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_dnn.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_ml.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_plot.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_photo.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudaimgproc.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudafilters.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudaarithm.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_objdetect.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_calib3d.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_features2d.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_flann.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_highgui.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_videoio.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_imgcodecs.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_imgproc.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_core.so.4.1.0
../bin/Radar: /usr/local/lib/libopencv_cudev.so.4.1.0
../bin/Radar: CMakeFiles/Radar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../bin/Radar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Radar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Radar.dir/build: ../bin/Radar

.PHONY : CMakeFiles/Radar.dir/build

CMakeFiles/Radar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Radar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Radar.dir/clean

CMakeFiles/Radar.dir/depend:
	cd /home/demon/CLionProjects/Radar/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/demon/CLionProjects/Radar /home/demon/CLionProjects/Radar /home/demon/CLionProjects/Radar/cmake-build-debug /home/demon/CLionProjects/Radar/cmake-build-debug /home/demon/CLionProjects/Radar/cmake-build-debug/CMakeFiles/Radar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Radar.dir/depend

