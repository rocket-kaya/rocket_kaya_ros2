cmake_minimum_required(VERSION 3.5)
project(kaya_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY rviz urdf meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
