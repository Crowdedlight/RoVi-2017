# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /home/frederik/clion-2017.2.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/frederik/clion-2017.2.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/cmake-build-release

# Include any dependencies generated for this target.
include CMakeFiles/FinalProject_Marker3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FinalProject_Marker3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FinalProject_Marker3.dir/flags.make

CMakeFiles/FinalProject_Marker3.dir/main.cpp.o: CMakeFiles/FinalProject_Marker3.dir/flags.make
CMakeFiles/FinalProject_Marker3.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FinalProject_Marker3.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FinalProject_Marker3.dir/main.cpp.o -c /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/main.cpp

CMakeFiles/FinalProject_Marker3.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FinalProject_Marker3.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/main.cpp > CMakeFiles/FinalProject_Marker3.dir/main.cpp.i

CMakeFiles/FinalProject_Marker3.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FinalProject_Marker3.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/main.cpp -o CMakeFiles/FinalProject_Marker3.dir/main.cpp.s

CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.requires

CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.provides: CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/FinalProject_Marker3.dir/build.make CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.provides

CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.provides.build: CMakeFiles/FinalProject_Marker3.dir/main.cpp.o


# Object files for target FinalProject_Marker3
FinalProject_Marker3_OBJECTS = \
"CMakeFiles/FinalProject_Marker3.dir/main.cpp.o"

# External object files for target FinalProject_Marker3
FinalProject_Marker3_EXTERNAL_OBJECTS =

FinalProject_Marker3: CMakeFiles/FinalProject_Marker3.dir/main.cpp.o
FinalProject_Marker3: CMakeFiles/FinalProject_Marker3.dir/build.make
FinalProject_Marker3: /usr/local/lib/libopencv_stitching.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_superres.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_videostab.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_aruco.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_bgsegm.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_bioinspired.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_ccalib.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_cvv.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_dpm.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_face.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_freetype.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_fuzzy.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_img_hash.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_line_descriptor.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_optflow.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_reg.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_rgbd.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_saliency.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_stereo.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_structured_light.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_surface_matching.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_tracking.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_xfeatures2d.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_ximgproc.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_xobjdetect.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_xphoto.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_shape.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_photo.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_calib3d.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_phase_unwrapping.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_video.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_datasets.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_plot.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_text.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_dnn.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_features2d.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_flann.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_highgui.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_ml.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_videoio.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_imgcodecs.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_objdetect.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_imgproc.so.3.3.1
FinalProject_Marker3: /usr/local/lib/libopencv_core.so.3.3.1
FinalProject_Marker3: CMakeFiles/FinalProject_Marker3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FinalProject_Marker3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FinalProject_Marker3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FinalProject_Marker3.dir/build: FinalProject_Marker3

.PHONY : CMakeFiles/FinalProject_Marker3.dir/build

CMakeFiles/FinalProject_Marker3.dir/requires: CMakeFiles/FinalProject_Marker3.dir/main.cpp.o.requires

.PHONY : CMakeFiles/FinalProject_Marker3.dir/requires

CMakeFiles/FinalProject_Marker3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FinalProject_Marker3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FinalProject_Marker3.dir/clean

CMakeFiles/FinalProject_Marker3.dir/depend:
	cd /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3 /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3 /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/cmake-build-release /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/cmake-build-release /home/frederik/Documents/RoVi/Vision/Excercises/FinalProject_Marker3/cmake-build-release/CMakeFiles/FinalProject_Marker3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FinalProject_Marker3.dir/depend

