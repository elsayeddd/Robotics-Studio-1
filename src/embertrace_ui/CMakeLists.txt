cmake_minimum_required(VERSION 3.8)
project(embertrace_ui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(embertrace_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(operator_dashboard src/operator_dashboard.cpp)
ament_target_dependencies(operator_dashboard embertrace_interfaces rclcpp rcl_interfaces nav_msgs geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(operator_dashboard Qt5::Widgets)
install(TARGETS operator_dashboard DESTINATION lib/${PROJECT_NAME})

ament_package()

