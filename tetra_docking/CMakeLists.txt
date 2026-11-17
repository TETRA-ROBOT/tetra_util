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
find_package(angles REQUIRED)

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
  angles
)

include_directories(
  include
)

add_library(tetra_docking_plugins SHARED
  src/tetra_charging_dock.cpp
  src/tetra_lift_dock.cpp
  src/tag_staging.cpp
  src/tag_view.cpp
  src/bumper_guard.cpp
  src/approach_shaping.cpp
  src/dock_offset.cpp
  src/staging_yaw_lock.cpp
  src/tag_search.cpp
  src/tag_search_plan.cpp
  src/refined_pose.cpp
  src/tag_dock_estimate.cpp
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 계산만 하는 소스는 플러그인 라이브러리(도킹 서버·Nav2 에 묶임) 없이 따로 컴파일해 테스트한다
  ament_add_gtest(test_tag_search_plan test/test_tag_search_plan.cpp src/tag_search_plan.cpp)
  ament_target_dependencies(test_tag_search_plan angles)
  ament_add_gtest(test_dock_geometry test/test_dock_geometry.cpp src/tag_view.cpp)
  ament_target_dependencies(test_dock_geometry angles tf2)
endif()

ament_export_include_directories(include)
ament_export_libraries(tetra_docking_plugins)
ament_export_dependencies(${dependencies})

ament_package()
