cmake_minimum_required(VERSION 3.8)
project(rov_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/goto_pose_check.py DESTINATION lib/${PROJECT_NAME})

ament_package()
