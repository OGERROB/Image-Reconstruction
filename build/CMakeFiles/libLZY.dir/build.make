# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/liuyz/桌面/仿射变换

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liuyz/桌面/仿射变换/build

# Include any dependencies generated for this target.
include CMakeFiles/libLZY.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/libLZY.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/libLZY.dir/flags.make

CMakeFiles/libLZY.dir/functions.cpp.o: CMakeFiles/libLZY.dir/flags.make
CMakeFiles/libLZY.dir/functions.cpp.o: ../functions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuyz/桌面/仿射变换/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/libLZY.dir/functions.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libLZY.dir/functions.cpp.o -c /home/liuyz/桌面/仿射变换/functions.cpp

CMakeFiles/libLZY.dir/functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libLZY.dir/functions.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuyz/桌面/仿射变换/functions.cpp > CMakeFiles/libLZY.dir/functions.cpp.i

CMakeFiles/libLZY.dir/functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libLZY.dir/functions.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuyz/桌面/仿射变换/functions.cpp -o CMakeFiles/libLZY.dir/functions.cpp.s

CMakeFiles/libLZY.dir/functions.cpp.o.requires:

.PHONY : CMakeFiles/libLZY.dir/functions.cpp.o.requires

CMakeFiles/libLZY.dir/functions.cpp.o.provides: CMakeFiles/libLZY.dir/functions.cpp.o.requires
	$(MAKE) -f CMakeFiles/libLZY.dir/build.make CMakeFiles/libLZY.dir/functions.cpp.o.provides.build
.PHONY : CMakeFiles/libLZY.dir/functions.cpp.o.provides

CMakeFiles/libLZY.dir/functions.cpp.o.provides.build: CMakeFiles/libLZY.dir/functions.cpp.o


# Object files for target libLZY
libLZY_OBJECTS = \
"CMakeFiles/libLZY.dir/functions.cpp.o"

# External object files for target libLZY
libLZY_EXTERNAL_OBJECTS =

liblibLZY.a: CMakeFiles/libLZY.dir/functions.cpp.o
liblibLZY.a: CMakeFiles/libLZY.dir/build.make
liblibLZY.a: CMakeFiles/libLZY.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liuyz/桌面/仿射变换/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblibLZY.a"
	$(CMAKE_COMMAND) -P CMakeFiles/libLZY.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libLZY.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/libLZY.dir/build: liblibLZY.a

.PHONY : CMakeFiles/libLZY.dir/build

CMakeFiles/libLZY.dir/requires: CMakeFiles/libLZY.dir/functions.cpp.o.requires

.PHONY : CMakeFiles/libLZY.dir/requires

CMakeFiles/libLZY.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/libLZY.dir/cmake_clean.cmake
.PHONY : CMakeFiles/libLZY.dir/clean

CMakeFiles/libLZY.dir/depend:
	cd /home/liuyz/桌面/仿射变换/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liuyz/桌面/仿射变换 /home/liuyz/桌面/仿射变换 /home/liuyz/桌面/仿射变换/build /home/liuyz/桌面/仿射变换/build /home/liuyz/桌面/仿射变换/build/CMakeFiles/libLZY.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/libLZY.dir/depend

