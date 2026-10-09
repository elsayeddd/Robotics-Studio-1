cmake_minimum_required(VERSION 3.8)
project(embertrace_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(embertrace_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

add_library(plume_model src/plume_model.cpp)
target_include_directories(plume_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(nav2_goal_adapter_lib src/nav2_goal_adapter_node.cpp)
target_include_directories(nav2_goal_adapter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav2_goal_adapter_lib
  embertrace_interfaces geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs visualization_msgs
  nav2_msgs rclcpp_action tf2 tf2_ros tf2_geometry_msgs
)
add_library(search_estimation src/source_estimator.cpp src/known_free_grid.cpp)
target_include_directories(search_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(waypoint_patrol src/waypoint_patrol.cpp)
target_include_directories(waypoint_patrol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(waypoint_patrol PUBLIC ${YAML_CPP_TARGET})

function(add_embertrace_node target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    embertrace_interfaces geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs visualization_msgs
  )
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_embertrace_node(environment_simulator_node src/environment_simulator_node.cpp)
target_link_libraries(environment_simulator_node plume_model)

add_embertrace_node(camera_verifier_node src/camera_verifier_node.cpp)
add_embertrace_node(perception_node src/perception_node.cpp)
add_embertrace_node(mission_manager_node src/mission_manager_node.cpp)
add_embertrace_node(search_planner_node src/search_planner_node.cpp)
target_link_libraries(search_planner_node search_estimation)
ament_target_dependencies(search_planner_node tf2 tf2_geometry_msgs tf2_ros)
add_embertrace_node(heatmap_node src/heatmap_node.cpp)


add_executable(nav2_goal_adapter_node src/nav2_goal_adapter_lib.cpp)
target_include_directories(nav2_goal_adapter_node PRIVATE include)
ament_target_dependencies(nav2_goal_adapter_node
  embertrace_interfaces geometry_msgs nav_msgs rclcpp rclcpp_action nav2_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(nav2_goal_adapter_node nav2_goal_adapter_lib)
install(TARGETS nav2_goal_adapter_node DESTINATION lib/${PROJECT_NAME})


add_embertrace_node(patrol_manager_node src/patrol_manager_node.cpp)
target_link_libraries(patrol_manager_node waypoint_patrol)
ament_target_dependencies(patrol_manager_node tf2 tf2_geometry_msgs tf2_ros)


install(
  TARGETS plume_model search_estimation waypoint_patrol nav2_goal_adapter_lib
  EXPORT plume_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES src/README.md DESTINATION share/${PROJECT_NAME}) #remove if issue

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plume_model test/test_plume_model.cpp)
  if(TARGET test_plume_model)
    target_link_libraries(test_plume_model plume_model)
    target_include_directories(test_plume_model PRIVATE include)
  endif()

  ament_add_gtest(test_nav2_goal_adapter test/test_nav2_goal_adapter.cpp)
  if(TARGET test_nav2_goal_adapter)
    target_link_libraries(test_nav2_goal_adapter nav2_goal_adapter_lib)
    target_include_directories(test_nav2_goal_adapter PRIVATE include)
    ament_target_dependencies(test_nav2_goal_adapter
      rclcpp rclcpp_action nav2_msgs geometry_msgs nav_msgs embertrace_interfaces tf2 tf2_ros tf2_geometry_msgs)
  endif()
  
  ament_add_gtest(test_search_estimation test/test_search_estimation.cpp)
  if(TARGET test_search_estimation)
    target_link_libraries(test_search_estimation search_estimation plume_model)
    target_include_directories(test_search_estimation PRIVATE include)
  endif()

  ament_add_gtest(test_waypoint_patrol test/test_waypoint_patrol.cpp)
  if(TARGET test_waypoint_patrol)
    target_link_libraries(test_waypoint_patrol waypoint_patrol)
    target_include_directories(test_waypoint_patrol PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(plume_model_targets HAS_LIBRARY_TARGET)
ament_package()
