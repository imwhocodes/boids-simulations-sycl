# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.20.5/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.20.5/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/lucapassarella/Documents/projects/boid_fluid

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/lucapassarella/Documents/projects/boid_fluid/build

# Include any dependencies generated for this target.
include lib/rtmidi/CMakeFiles/midiprobe.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include lib/rtmidi/CMakeFiles/midiprobe.dir/compiler_depend.make

# Include the progress variables for this target.
include lib/rtmidi/CMakeFiles/midiprobe.dir/progress.make

# Include the compile flags for this target's objects.
include lib/rtmidi/CMakeFiles/midiprobe.dir/flags.make

lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o: lib/rtmidi/CMakeFiles/midiprobe.dir/flags.make
lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o: ../lib/rtmidi/tests/midiprobe.cpp
lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o: lib/rtmidi/CMakeFiles/midiprobe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lucapassarella/Documents/projects/boid_fluid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o"
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o -MF CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o.d -o CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o -c /Users/lucapassarella/Documents/projects/boid_fluid/lib/rtmidi/tests/midiprobe.cpp

lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.i"
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lucapassarella/Documents/projects/boid_fluid/lib/rtmidi/tests/midiprobe.cpp > CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.i

lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.s"
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lucapassarella/Documents/projects/boid_fluid/lib/rtmidi/tests/midiprobe.cpp -o CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.s

# Object files for target midiprobe
midiprobe_OBJECTS = \
"CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o"

# External object files for target midiprobe
midiprobe_EXTERNAL_OBJECTS =

lib/rtmidi/tests/midiprobe: lib/rtmidi/CMakeFiles/midiprobe.dir/tests/midiprobe.cpp.o
lib/rtmidi/tests/midiprobe: lib/rtmidi/CMakeFiles/midiprobe.dir/build.make
lib/rtmidi/tests/midiprobe: lib/rtmidi/librtmidi.6.0.0.dylib
lib/rtmidi/tests/midiprobe: lib/rtmidi/CMakeFiles/midiprobe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/lucapassarella/Documents/projects/boid_fluid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable tests/midiprobe"
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/midiprobe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/rtmidi/CMakeFiles/midiprobe.dir/build: lib/rtmidi/tests/midiprobe
.PHONY : lib/rtmidi/CMakeFiles/midiprobe.dir/build

lib/rtmidi/CMakeFiles/midiprobe.dir/clean:
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi && $(CMAKE_COMMAND) -P CMakeFiles/midiprobe.dir/cmake_clean.cmake
.PHONY : lib/rtmidi/CMakeFiles/midiprobe.dir/clean

lib/rtmidi/CMakeFiles/midiprobe.dir/depend:
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/lucapassarella/Documents/projects/boid_fluid /Users/lucapassarella/Documents/projects/boid_fluid/lib/rtmidi /Users/lucapassarella/Documents/projects/boid_fluid/build /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi/CMakeFiles/midiprobe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/rtmidi/CMakeFiles/midiprobe.dir/depend

