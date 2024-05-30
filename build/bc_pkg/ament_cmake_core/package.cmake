set(_AMENT_PACKAGE_NAME "bc_pkg")
set(bc_pkg_VERSION "0.0.0")
set(bc_pkg_MAINTAINER "flejv <erkoj.jirkakutyka@gmail.com>")
set(bc_pkg_BUILD_DEPENDS "builtin_interfaces" "rclcpp" "px4_msgs" "geometry_msgs" "sensor_msgs")
set(bc_pkg_BUILDTOOL_DEPENDS "ament_cmake" "eigen3_cmake_module" "eigen" "ros_environment")
set(bc_pkg_BUILD_EXPORT_DEPENDS "builtin_interfaces" "rclcpp" "px4_msgs" "geometry_msgs" "sensor_msgs")
set(bc_pkg_BUILDTOOL_EXPORT_DEPENDS "eigen4_cmake_module" "eigen")
set(bc_pkg_EXEC_DEPENDS "rosidl_default_runtime" "builtin_interfaces" "rclcpp" "px4_msgs" "geometry_msgs" "sensor_msgs")
set(bc_pkg_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(bc_pkg_GROUP_DEPENDS )
set(bc_pkg_MEMBER_OF_GROUPS )
set(bc_pkg_DEPRECATED "")
set(bc_pkg_EXPORT_TAGS)
list(APPEND bc_pkg_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
