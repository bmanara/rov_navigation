cmake_minimum_required(VERSION 3.8)
project(rov_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config models urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model_consistency test/test_model_consistency.py)
endif()

ament_package()
