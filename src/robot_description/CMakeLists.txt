cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# This package is a single, versioned source for robot models and generated assets.
# Preserve the complete directory trees so relative MJCF mesh paths also work after
# an ordinary (non-symlink) colcon install.
install(DIRECTORY
  urdf
  meshes
  mjcf
  rviz
  config
  docs
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
