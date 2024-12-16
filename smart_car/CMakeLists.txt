cmake_minimum_required(VERSION 3.8)
project(smart_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(smartcar_msgs REQUIRED)  # Added back

# smart_car_node
add_executable(smart_car_node src/smart_car_node.cpp)
target_include_directories(smart_car_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(smart_car_node PUBLIC c_std_99 cxx_std_17)

# Link dependencies 
ament_target_dependencies(smart_car_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  sensor_msgs
  smartcar_msgs  # Added back
)

# Python scripts
install(DIRECTORY
  script/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# smart_car_node 
install(TARGETS smart_car_node
  DESTINATION lib/${PROJECT_NAME})

# launch, urdf, rviz, and config directories
install(DIRECTORY
  launch
  urdf
  rviz
  config
  map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
