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
CMAKE_SOURCE_DIR = /home/flejv/bc_ws/src/bc_pkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/flejv/bc_ws/build/bc_pkg

# Include any dependencies generated for this target.
include CMakeFiles/gest.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/gest.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/gest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gest.dir/flags.make

CMakeFiles/gest.dir/src/gesture.cpp.o: CMakeFiles/gest.dir/flags.make
CMakeFiles/gest.dir/src/gesture.cpp.o: /home/flejv/bc_ws/src/bc_pkg/src/gesture.cpp
CMakeFiles/gest.dir/src/gesture.cpp.o: CMakeFiles/gest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/flejv/bc_ws/build/bc_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gest.dir/src/gesture.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/gest.dir/src/gesture.cpp.o -MF CMakeFiles/gest.dir/src/gesture.cpp.o.d -o CMakeFiles/gest.dir/src/gesture.cpp.o -c /home/flejv/bc_ws/src/bc_pkg/src/gesture.cpp

CMakeFiles/gest.dir/src/gesture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gest.dir/src/gesture.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/flejv/bc_ws/src/bc_pkg/src/gesture.cpp > CMakeFiles/gest.dir/src/gesture.cpp.i

CMakeFiles/gest.dir/src/gesture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gest.dir/src/gesture.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/flejv/bc_ws/src/bc_pkg/src/gesture.cpp -o CMakeFiles/gest.dir/src/gesture.cpp.s

# Object files for target gest
gest_OBJECTS = \
"CMakeFiles/gest.dir/src/gesture.cpp.o"

# External object files for target gest
gest_EXTERNAL_OBJECTS =

gest: CMakeFiles/gest.dir/src/gesture.cpp.o
gest: CMakeFiles/gest.dir/build.make
gest: /opt/ros/humble/lib/librclcpp.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_cpp.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_generator_py.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
gest: /opt/ros/humble/lib/liblibstatistics_collector.so
gest: /opt/ros/humble/lib/librcl.so
gest: /opt/ros/humble/lib/librmw_implementation.so
gest: /opt/ros/humble/lib/libament_index_cpp.so
gest: /opt/ros/humble/lib/librcl_logging_spdlog.so
gest: /opt/ros/humble/lib/librcl_logging_interface.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
gest: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
gest: /opt/ros/humble/lib/librcl_yaml_param_parser.so
gest: /opt/ros/humble/lib/libyaml.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
gest: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
gest: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
gest: /opt/ros/humble/lib/libtracetools.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_c.so
gest: /home/flejv/bc_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_generator_c.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
gest: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
gest: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
gest: /opt/ros/humble/lib/libfastcdr.so.1.0.24
gest: /opt/ros/humble/lib/librmw.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
gest: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
gest: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
gest: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
gest: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
gest: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
gest: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
gest: /opt/ros/humble/lib/librosidl_typesupport_c.so
gest: /opt/ros/humble/lib/librcpputils.so
gest: /opt/ros/humble/lib/librosidl_runtime_c.so
gest: /opt/ros/humble/lib/librcutils.so
gest: /usr/lib/x86_64-linux-gnu/libpython3.10.so
gest: CMakeFiles/gest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/flejv/bc_ws/build/bc_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gest.dir/build: gest
.PHONY : CMakeFiles/gest.dir/build

CMakeFiles/gest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gest.dir/clean

CMakeFiles/gest.dir/depend:
	cd /home/flejv/bc_ws/build/bc_pkg && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/flejv/bc_ws/src/bc_pkg /home/flejv/bc_ws/src/bc_pkg /home/flejv/bc_ws/build/bc_pkg /home/flejv/bc_ws/build/bc_pkg /home/flejv/bc_ws/build/bc_pkg/CMakeFiles/gest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gest.dir/depend

