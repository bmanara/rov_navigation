cmake_minimum_required(VERSION 3.8)
project(rov_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compile_commands.json for editor IntelliSense (.vscode/c_cpp_properties.json)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS-independent control math (PID, allocation, control law): unit-testable
# without Gazebo or a running ROS graph.
add_library(rov_control_core
  src/pid.cpp
  src/thruster_allocation.cpp
  src/velocity_control_law.cpp)
target_include_directories(rov_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rov_control_core PUBLIC Eigen3::Eigen)

add_executable(velocity_controller src/velocity_controller_node.cpp)
target_link_libraries(velocity_controller rov_control_core
  rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(thruster_allocator src/thruster_allocator_node.cpp)
target_link_libraries(thruster_allocator rov_control_core
  rclcpp::rclcpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

install(TARGETS rov_control_core EXPORT export_rov_control
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS velocity_controller thruster_allocator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/step_response.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid rov_control_core)
  ament_add_gtest(test_allocation test/test_allocation.cpp)
  target_link_libraries(test_allocation rov_control_core)
  ament_add_gtest(test_velocity_control_law test/test_velocity_control_law.cpp)
  target_link_libraries(test_velocity_control_law rov_control_core)
endif()

ament_export_targets(export_rov_control HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
