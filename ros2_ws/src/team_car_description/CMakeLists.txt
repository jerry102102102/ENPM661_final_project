cmake_minimum_required(VERSION 3.8)
project(team_car_description)

find_package(ament_cmake REQUIRED)

# Description-only ROS2 package: install assets for launch files to consume.
install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    world
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
