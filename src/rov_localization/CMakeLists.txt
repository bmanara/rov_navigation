cmake_minimum_required(VERSION 3.8)
project(rov_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ground_truth_tf.py DESTINATION lib/${PROJECT_NAME})

ament_package()
