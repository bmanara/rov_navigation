cmake_minimum_required(VERSION 3.8)
project(rov_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module (ray casting for the emulated sonar), importable by scripts
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/sensor_emulator.py
  scripts/sonar_emulator.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_raycast test/test_raycast.py)
  ament_add_pytest_test(test_sonar_config test/test_sonar_config.py)
endif()

ament_package()
