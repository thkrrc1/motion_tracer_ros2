cmake_minimum_required(VERSION 3.8)
project(motion_tracer_ros2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(aero_controller_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS filesystem system thread)

add_executable(tracer_teleop_node
  src/tracer_teleop.cpp
  src/tracer_command.cpp
)
add_executable(upper_controller_node src/upper_controller.cpp)
add_executable(lower_controller_node src/lower_controller.cpp)

target_include_directories(tracer_teleop_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(upper_controller_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(lower_controller_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(tracer_teleop_node ${Boost_LIBRARIES})
target_link_libraries(upper_controller_node ${Boost_LIBRARIES})
target_link_libraries(lower_controller_node ${Boost_LIBRARIES})

ament_target_dependencies(tracer_teleop_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
)
ament_target_dependencies(upper_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rclcpp_action
  control_msgs
  aero_controller_msgs
  trajectory_msgs
)
ament_target_dependencies(lower_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rclcpp_action
  control_msgs
  trajectory_msgs
)

install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/ 
  DESTINATION include
)

install(TARGETS
  tracer_teleop_node
  upper_controller_node
  lower_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
