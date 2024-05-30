cmake_minimum_required(VERSION 3.8)
project(bc_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

 #Setup LeapSDK root path based on the OS
 set(ULTRALEAP_PATH_ROOT "")

 #Locate the LeapSDK on the system
 find_package(LeapSDK 5 REQUIRED PATHS "${ULTRALEAP_PATH_ROOT}")

 #Define include directories for header files
 include_directories(
     #${rclcpp_INCLUDE_DIRS}
     #${std_msgs_INCLUDE_DIRS}
     #${sensor_msgs_INCLUDE_DIRS}
     #${geometry_msgs_INCLUDE_DIRS}
     #${mavros_msgs_INCLUDE_DIRS}
     /usr/include  # For LeapC.h
 )

 #Compile the ExampleConnection library as an object library
 add_library(
     libExampleConnection
      OBJECT
      "/usr/share/doc/ultraleap-hand-tracking-service/samples/ExampleConnection.c"
 )

 #Link the LeapSDK library to the ExampleConnection library
 target_link_libraries(
     libExampleConnection
     PUBLIC
     LeapSDK::LeapC
 )

 #If on a UNIX system, link the Threads library to the ExampleConnection library
 if (UNIX)
     target_link_libraries(
         libExampleConnection
         PRIVATE
         Threads::Threads
     )
 endif()

 #Define where to find the header files for the ExampleConnection library
 target_include_directories(
     libExampleConnection
     PUBLIC
     "/usr/share/doc/ultraleap-hand-tracking-service/samples"  # For ExampleConnection.h
 )

add_executable(connect src/communication_hub)
ament_target_dependencies(connect rclcpp std_msgs sensor_msgs px4_msgs geometry_msgs)

add_executable(control src/leap_control)
ament_target_dependencies(control rclcpp std_msgs px4_msgs geometry_msgs)

add_executable(hand src/virtual_hand)
ament_target_dependencies(hand rclcpp std_msgs px4_msgs geometry_msgs visualization_msgs)

add_executable(gest src/gesture)
ament_target_dependencies(gest rclcpp std_msgs px4_msgs geometry_msgs)

add_executable(rpy src/rpy_pub)
ament_target_dependencies(rpy rclcpp geometry_msgs)
target_link_libraries(rpy ncurses libExampleConnection LeapSDK::LeapC)

add_executable(carth src/carth_pub)
ament_target_dependencies(carth rclcpp geometry_msgs visualization_msgs)
target_link_libraries(carth ncurses libExampleConnection LeapSDK::LeapC)

install(TARGETS
  connect
  control
  hand
  gest
  rpy
  carth
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
