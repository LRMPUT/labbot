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
CMAKE_SOURCE_DIR = /home/labbot/labbot_ws/libfreenect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/labbot/labbot_ws/libfreenect

# Include any dependencies generated for this target.
include examples/CMakeFiles/freenect-glview.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/freenect-glview.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/freenect-glview.dir/flags.make

examples/CMakeFiles/freenect-glview.dir/glview.c.o: examples/CMakeFiles/freenect-glview.dir/flags.make
examples/CMakeFiles/freenect-glview.dir/glview.c.o: examples/glview.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/labbot/labbot_ws/libfreenect/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/freenect-glview.dir/glview.c.o"
	cd /home/labbot/labbot_ws/libfreenect/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freenect-glview.dir/glview.c.o   -c /home/labbot/labbot_ws/libfreenect/examples/glview.c

examples/CMakeFiles/freenect-glview.dir/glview.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freenect-glview.dir/glview.c.i"
	cd /home/labbot/labbot_ws/libfreenect/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/labbot/labbot_ws/libfreenect/examples/glview.c > CMakeFiles/freenect-glview.dir/glview.c.i

examples/CMakeFiles/freenect-glview.dir/glview.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freenect-glview.dir/glview.c.s"
	cd /home/labbot/labbot_ws/libfreenect/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/labbot/labbot_ws/libfreenect/examples/glview.c -o CMakeFiles/freenect-glview.dir/glview.c.s

examples/CMakeFiles/freenect-glview.dir/glview.c.o.requires:

.PHONY : examples/CMakeFiles/freenect-glview.dir/glview.c.o.requires

examples/CMakeFiles/freenect-glview.dir/glview.c.o.provides: examples/CMakeFiles/freenect-glview.dir/glview.c.o.requires
	$(MAKE) -f examples/CMakeFiles/freenect-glview.dir/build.make examples/CMakeFiles/freenect-glview.dir/glview.c.o.provides.build
.PHONY : examples/CMakeFiles/freenect-glview.dir/glview.c.o.provides

examples/CMakeFiles/freenect-glview.dir/glview.c.o.provides.build: examples/CMakeFiles/freenect-glview.dir/glview.c.o


# Object files for target freenect-glview
freenect__glview_OBJECTS = \
"CMakeFiles/freenect-glview.dir/glview.c.o"

# External object files for target freenect-glview
freenect__glview_EXTERNAL_OBJECTS =

bin/freenect-glview: examples/CMakeFiles/freenect-glview.dir/glview.c.o
bin/freenect-glview: examples/CMakeFiles/freenect-glview.dir/build.make
bin/freenect-glview: lib/libfreenect.so.0.6.0
bin/freenect-glview: /usr/lib/x86_64-linux-gnu/libGL.so
bin/freenect-glview: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/freenect-glview: /usr/lib/x86_64-linux-gnu/libglut.so
bin/freenect-glview: /usr/lib/x86_64-linux-gnu/libXmu.so
bin/freenect-glview: /usr/lib/x86_64-linux-gnu/libXi.so
bin/freenect-glview: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
bin/freenect-glview: examples/CMakeFiles/freenect-glview.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/labbot/labbot_ws/libfreenect/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable ../bin/freenect-glview"
	cd /home/labbot/labbot_ws/libfreenect/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/freenect-glview.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/freenect-glview.dir/build: bin/freenect-glview

.PHONY : examples/CMakeFiles/freenect-glview.dir/build

examples/CMakeFiles/freenect-glview.dir/requires: examples/CMakeFiles/freenect-glview.dir/glview.c.o.requires

.PHONY : examples/CMakeFiles/freenect-glview.dir/requires

examples/CMakeFiles/freenect-glview.dir/clean:
	cd /home/labbot/labbot_ws/libfreenect/examples && $(CMAKE_COMMAND) -P CMakeFiles/freenect-glview.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/freenect-glview.dir/clean

examples/CMakeFiles/freenect-glview.dir/depend:
	cd /home/labbot/labbot_ws/libfreenect && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/labbot/labbot_ws/libfreenect /home/labbot/labbot_ws/libfreenect/examples /home/labbot/labbot_ws/libfreenect /home/labbot/labbot_ws/libfreenect/examples /home/labbot/labbot_ws/libfreenect/examples/CMakeFiles/freenect-glview.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/freenect-glview.dir/depend
