cmake_minimum_required(VERSION 3.8)
project(wheeled_quadruped_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/default_posture.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

ament_package()
