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

add_executable(operator_dashboard src/operator_dashboard.cpp)
ament_target_dependencies(operator_dashboard embertrace_interfaces rclcpp)
target_link_libraries(operator_dashboard Qt5::Widgets)
install(TARGETS operator_dashboard DESTINATION lib/${PROJECT_NAME})

ament_package()

