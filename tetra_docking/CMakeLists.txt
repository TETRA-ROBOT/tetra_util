cmake_minimum_required(VERSION 3.8)
project(tetra_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  pluginlib
  opennav_docking_core
  opennav_docking
  nav2_util
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
  interfaces
)

include_directories(
  include
)

add_library(tetra_docking_plugins SHARED
  src/tetra_charging_dock.cpp
  src/tetra_lift_dock.cpp
  src/tag_staging.cpp
  src/tag_view.cpp
)

ament_target_dependencies(tetra_docking_plugins ${dependencies})

get_filename_component(
  OPENNAV_DOCKING_LIB_DIR
  "${opennav_docking_DIR}/../../../lib"
  REALPATH
)

target_link_libraries(
  tetra_docking_plugins
  "${OPENNAV_DOCKING_LIB_DIR}/libsimple_charging_dock.so"
  "${OPENNAV_DOCKING_LIB_DIR}/libsimple_non_charging_dock.so"
)

pluginlib_export_plugin_description_file(opennav_docking_core opennav_docking_plugins.xml)

install(TARGETS tetra_docking_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES opennav_docking_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(tetra_docking_plugins)
ament_export_dependencies(${dependencies})

ament_package()
