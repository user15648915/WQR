cmake_minimum_required(VERSION 3.8)
project(wheeled_quadruped_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Находим необходимые пакеты ROS 2 и PCL
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Создаем узел распознавания
add_executable(step_detector_node src/step_detector_node.cpp)
ament_target_dependencies(step_detector_node rclcpp sensor_msgs std_msgs pcl_conversions)
target_link_libraries(step_detector_node ${PCL_LIBRARIES})

# Установка бинарника
install(TARGETS
  step_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
