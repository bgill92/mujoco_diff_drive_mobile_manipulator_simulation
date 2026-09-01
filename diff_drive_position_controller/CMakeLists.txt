cmake_minimum_required(VERSION 3.16)
project(diff_drive_position_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(diff_drive_position_controller_parameters
  src/diff_drive_position_controller_parameters.yaml
)

add_library(diff_drive_position_controller SHARED
  src/diff_drive_position_controller.cpp
  src/odometry.cpp
)
target_compile_features(diff_drive_position_controller PUBLIC cxx_std_20)
target_include_directories(diff_drive_position_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_drive_position_controller>
)
target_link_libraries(diff_drive_position_controller PUBLIC
  diff_drive_position_controller_parameters
)
ament_target_dependencies(diff_drive_position_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(
  controller_interface diff_drive_position_controller_plugin.xml
)

install(
  DIRECTORY include/
  DESTINATION include/diff_drive_position_controller
)
install(
  TARGETS diff_drive_position_controller diff_drive_position_controller_parameters
  EXPORT export_diff_drive_position_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_law test/test_control_law.cpp)
  target_link_libraries(test_control_law diff_drive_position_controller)
endif()

ament_export_targets(export_diff_drive_position_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
