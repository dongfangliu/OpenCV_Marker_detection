# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.6

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "E:\Program Files (x86)\JetBrains\CLion 2016.3.4\bin\cmake\bin\cmake.exe"

# The command to remove a file.
RM = "E:\Program Files (x86)\JetBrains\CLion 2016.3.4\bin\cmake\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = E:\Mycodes\OpenCV\OpenCVtest0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = E:\Mycodes\OpenCV\OpenCVtest0\cmake-build-release

# Include any dependencies generated for this target.
include CMakeFiles/test_cv.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_cv.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_cv.dir/flags.make

CMakeFiles/test_cv.dir/main.cpp.obj: CMakeFiles/test_cv.dir/flags.make
CMakeFiles/test_cv.dir/main.cpp.obj: CMakeFiles/test_cv.dir/includes_CXX.rsp
CMakeFiles/test_cv.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=E:\Mycodes\OpenCV\OpenCVtest0\cmake-build-release\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_cv.dir/main.cpp.obj"
	g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\test_cv.dir\main.cpp.obj -c E:\Mycodes\OpenCV\OpenCVtest0\main.cpp

CMakeFiles/test_cv.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_cv.dir/main.cpp.i"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E E:\Mycodes\OpenCV\OpenCVtest0\main.cpp > CMakeFiles\test_cv.dir\main.cpp.i

CMakeFiles/test_cv.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_cv.dir/main.cpp.s"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S E:\Mycodes\OpenCV\OpenCVtest0\main.cpp -o CMakeFiles\test_cv.dir\main.cpp.s

CMakeFiles/test_cv.dir/main.cpp.obj.requires:

.PHONY : CMakeFiles/test_cv.dir/main.cpp.obj.requires

CMakeFiles/test_cv.dir/main.cpp.obj.provides: CMakeFiles/test_cv.dir/main.cpp.obj.requires
	$(MAKE) -f CMakeFiles\test_cv.dir\build.make CMakeFiles/test_cv.dir/main.cpp.obj.provides.build
.PHONY : CMakeFiles/test_cv.dir/main.cpp.obj.provides

CMakeFiles/test_cv.dir/main.cpp.obj.provides.build: CMakeFiles/test_cv.dir/main.cpp.obj


CMakeFiles/test_cv.dir/Marker.cpp.obj: CMakeFiles/test_cv.dir/flags.make
CMakeFiles/test_cv.dir/Marker.cpp.obj: CMakeFiles/test_cv.dir/includes_CXX.rsp
CMakeFiles/test_cv.dir/Marker.cpp.obj: ../Marker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=E:\Mycodes\OpenCV\OpenCVtest0\cmake-build-release\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_cv.dir/Marker.cpp.obj"
	g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\test_cv.dir\Marker.cpp.obj -c E:\Mycodes\OpenCV\OpenCVtest0\Marker.cpp

CMakeFiles/test_cv.dir/Marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_cv.dir/Marker.cpp.i"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E E:\Mycodes\OpenCV\OpenCVtest0\Marker.cpp > CMakeFiles\test_cv.dir\Marker.cpp.i

CMakeFiles/test_cv.dir/Marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_cv.dir/Marker.cpp.s"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S E:\Mycodes\OpenCV\OpenCVtest0\Marker.cpp -o CMakeFiles\test_cv.dir\Marker.cpp.s

CMakeFiles/test_cv.dir/Marker.cpp.obj.requires:

.PHONY : CMakeFiles/test_cv.dir/Marker.cpp.obj.requires

CMakeFiles/test_cv.dir/Marker.cpp.obj.provides: CMakeFiles/test_cv.dir/Marker.cpp.obj.requires
	$(MAKE) -f CMakeFiles\test_cv.dir\build.make CMakeFiles/test_cv.dir/Marker.cpp.obj.provides.build
.PHONY : CMakeFiles/test_cv.dir/Marker.cpp.obj.provides

CMakeFiles/test_cv.dir/Marker.cpp.obj.provides.build: CMakeFiles/test_cv.dir/Marker.cpp.obj


# Object files for target test_cv
test_cv_OBJECTS = \
"CMakeFiles/test_cv.dir/main.cpp.obj" \
"CMakeFiles/test_cv.dir/Marker.cpp.obj"

# External object files for target test_cv
test_cv_EXTERNAL_OBJECTS =

test_cv.exe: CMakeFiles/test_cv.dir/main.cpp.obj
test_cv.exe: CMakeFiles/test_cv.dir/Marker.cpp.obj
test_cv.exe: CMakeFiles/test_cv.dir/build.make
test_cv.exe: C:/opencv/mingw-build/install/x86/mingw/lib/libopencv_highgui320.dll.a
test_cv.exe: C:/opencv/mingw-build/install/x86/mingw/lib/libopencv_imgcodecs320.dll.a
test_cv.exe: C:/opencv/mingw-build/install/x86/mingw/lib/libopencv_imgproc320.dll.a
test_cv.exe: C:/opencv/mingw-build/install/x86/mingw/lib/libopencv_core320.dll.a
test_cv.exe: CMakeFiles/test_cv.dir/linklibs.rsp
test_cv.exe: CMakeFiles/test_cv.dir/objects1.rsp
test_cv.exe: CMakeFiles/test_cv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=E:\Mycodes\OpenCV\OpenCVtest0\cmake-build-release\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test_cv.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\test_cv.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_cv.dir/build: test_cv.exe

.PHONY : CMakeFiles/test_cv.dir/build

CMakeFiles/test_cv.dir/requires: CMakeFiles/test_cv.dir/main.cpp.obj.requires
CMakeFiles/test_cv.dir/requires: CMakeFiles/test_cv.dir/Marker.cpp.obj.requires

.PHONY : CMakeFiles/test_cv.dir/requires

CMakeFiles/test_cv.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\test_cv.dir\cmake_clean.cmake
.PHONY : CMakeFiles/test_cv.dir/clean

CMakeFiles/test_cv.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" E:\Mycodes\OpenCV\OpenCVtest0 E:\Mycodes\OpenCV\OpenCVtest0 E:\Mycodes\OpenCV\OpenCVtest0\cmake-build-release E:\Mycodes\OpenCV\OpenCVtest0\cmake-build-release E:\Mycodes\OpenCV\OpenCVtest0\cmake-build-release\CMakeFiles\test_cv.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_cv.dir/depend

