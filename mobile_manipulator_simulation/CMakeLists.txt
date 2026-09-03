cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})

# Only the RViz/URDF half of description/ is installed. The MJCF half
# (scene.xml, mujoco_description_formatted.xml, assets/) is 75 MB and is run
# in-tree from the source directory, so copying it into share/ buys nothing.
install(FILES
  description/rox_diff_ur5e.urdf
  description/rox_diff_ur5e_mujoco.urdf
  description/rox_diff_ur5e_control.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/description)
install(DIRECTORY description/meshes DESTINATION share/${PROJECT_NAME}/description)

install(PROGRAMS scripts/ee_path_publisher.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
