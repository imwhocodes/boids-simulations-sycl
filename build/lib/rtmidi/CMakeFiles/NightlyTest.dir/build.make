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

# Utility rule file for NightlyTest.

# Include any custom commands dependencies for this target.
include lib/rtmidi/CMakeFiles/NightlyTest.dir/compiler_depend.make

# Include the progress variables for this target.
include lib/rtmidi/CMakeFiles/NightlyTest.dir/progress.make

lib/rtmidi/CMakeFiles/NightlyTest:
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi && /usr/local/Cellar/cmake/3.20.5/bin/ctest -D NightlyTest

NightlyTest: lib/rtmidi/CMakeFiles/NightlyTest
NightlyTest: lib/rtmidi/CMakeFiles/NightlyTest.dir/build.make
.PHONY : NightlyTest

# Rule to build all files generated by this target.
lib/rtmidi/CMakeFiles/NightlyTest.dir/build: NightlyTest
.PHONY : lib/rtmidi/CMakeFiles/NightlyTest.dir/build

lib/rtmidi/CMakeFiles/NightlyTest.dir/clean:
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi && $(CMAKE_COMMAND) -P CMakeFiles/NightlyTest.dir/cmake_clean.cmake
.PHONY : lib/rtmidi/CMakeFiles/NightlyTest.dir/clean

lib/rtmidi/CMakeFiles/NightlyTest.dir/depend:
	cd /Users/lucapassarella/Documents/projects/boid_fluid/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/lucapassarella/Documents/projects/boid_fluid /Users/lucapassarella/Documents/projects/boid_fluid/lib/rtmidi /Users/lucapassarella/Documents/projects/boid_fluid/build /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi /Users/lucapassarella/Documents/projects/boid_fluid/build/lib/rtmidi/CMakeFiles/NightlyTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/rtmidi/CMakeFiles/NightlyTest.dir/depend

