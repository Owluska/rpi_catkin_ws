# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/pi/catkin_ws/src/bfl-release

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/catkin_ws/build_isolated/bfl/devel

# Include any dependencies generated for this target.
include tests/CMakeFiles/test_ekf.dir/depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/test_ekf.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/test_ekf.dir/flags.make

tests/CMakeFiles/test_ekf.dir/test-runner.o: tests/CMakeFiles/test_ekf.dir/flags.make
tests/CMakeFiles/test_ekf.dir/test-runner.o: /home/pi/catkin_ws/src/bfl-release/tests/test-runner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/catkin_ws/build_isolated/bfl/devel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tests/CMakeFiles/test_ekf.dir/test-runner.o"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_ekf.dir/test-runner.o -c /home/pi/catkin_ws/src/bfl-release/tests/test-runner.cpp

tests/CMakeFiles/test_ekf.dir/test-runner.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ekf.dir/test-runner.i"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/catkin_ws/src/bfl-release/tests/test-runner.cpp > CMakeFiles/test_ekf.dir/test-runner.i

tests/CMakeFiles/test_ekf.dir/test-runner.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ekf.dir/test-runner.s"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/catkin_ws/src/bfl-release/tests/test-runner.cpp -o CMakeFiles/test_ekf.dir/test-runner.s

tests/CMakeFiles/test_ekf.dir/approxEqual.o: tests/CMakeFiles/test_ekf.dir/flags.make
tests/CMakeFiles/test_ekf.dir/approxEqual.o: /home/pi/catkin_ws/src/bfl-release/tests/approxEqual.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/catkin_ws/build_isolated/bfl/devel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tests/CMakeFiles/test_ekf.dir/approxEqual.o"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_ekf.dir/approxEqual.o -c /home/pi/catkin_ws/src/bfl-release/tests/approxEqual.cpp

tests/CMakeFiles/test_ekf.dir/approxEqual.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ekf.dir/approxEqual.i"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/catkin_ws/src/bfl-release/tests/approxEqual.cpp > CMakeFiles/test_ekf.dir/approxEqual.i

tests/CMakeFiles/test_ekf.dir/approxEqual.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ekf.dir/approxEqual.s"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/catkin_ws/src/bfl-release/tests/approxEqual.cpp -o CMakeFiles/test_ekf.dir/approxEqual.s

tests/CMakeFiles/test_ekf.dir/ekf_test.o: tests/CMakeFiles/test_ekf.dir/flags.make
tests/CMakeFiles/test_ekf.dir/ekf_test.o: /home/pi/catkin_ws/src/bfl-release/tests/ekf_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/catkin_ws/build_isolated/bfl/devel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object tests/CMakeFiles/test_ekf.dir/ekf_test.o"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_ekf.dir/ekf_test.o -c /home/pi/catkin_ws/src/bfl-release/tests/ekf_test.cpp

tests/CMakeFiles/test_ekf.dir/ekf_test.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ekf.dir/ekf_test.i"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/catkin_ws/src/bfl-release/tests/ekf_test.cpp > CMakeFiles/test_ekf.dir/ekf_test.i

tests/CMakeFiles/test_ekf.dir/ekf_test.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ekf.dir/ekf_test.s"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/catkin_ws/src/bfl-release/tests/ekf_test.cpp -o CMakeFiles/test_ekf.dir/ekf_test.s

# Object files for target test_ekf
test_ekf_OBJECTS = \
"CMakeFiles/test_ekf.dir/test-runner.o" \
"CMakeFiles/test_ekf.dir/approxEqual.o" \
"CMakeFiles/test_ekf.dir/ekf_test.o"

# External object files for target test_ekf
test_ekf_EXTERNAL_OBJECTS =

tests/test_ekf: tests/CMakeFiles/test_ekf.dir/test-runner.o
tests/test_ekf: tests/CMakeFiles/test_ekf.dir/approxEqual.o
tests/test_ekf: tests/CMakeFiles/test_ekf.dir/ekf_test.o
tests/test_ekf: tests/CMakeFiles/test_ekf.dir/build.make
tests/test_ekf: src/liborocos-bfl.so
tests/test_ekf: /usr/lib/arm-linux-gnueabihf/libcppunit.so
tests/test_ekf: tests/CMakeFiles/test_ekf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/catkin_ws/build_isolated/bfl/devel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable test_ekf"
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_ekf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/test_ekf.dir/build: tests/test_ekf

.PHONY : tests/CMakeFiles/test_ekf.dir/build

tests/CMakeFiles/test_ekf.dir/clean:
	cd /home/pi/catkin_ws/build_isolated/bfl/devel/tests && $(CMAKE_COMMAND) -P CMakeFiles/test_ekf.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/test_ekf.dir/clean

tests/CMakeFiles/test_ekf.dir/depend:
	cd /home/pi/catkin_ws/build_isolated/bfl/devel && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/catkin_ws/src/bfl-release /home/pi/catkin_ws/src/bfl-release/tests /home/pi/catkin_ws/build_isolated/bfl/devel /home/pi/catkin_ws/build_isolated/bfl/devel/tests /home/pi/catkin_ws/build_isolated/bfl/devel/tests/CMakeFiles/test_ekf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/test_ekf.dir/depend
