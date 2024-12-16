cmake_minimum_required(VERSION 3.5)
project(gazebo_world)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# Install the world files
install(FILES
  smalltown.world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

