cmake_minimum_required(VERSION 3.5)
project(mpu6050)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

set(EXEC_NAME "run")

add_executable(${EXEC_NAME} src/mpu6050.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()

