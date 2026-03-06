cmake_minimum_required(VERSION 3.14)
project(autonomy_manager VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(autonomy_manager_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hector_ros2_utils REQUIRED)

# ##############################################################################
# Build ##
# ##############################################################################

add_library(
  ${PROJECT_NAME} SHARED
  src/autonomy_manager.cpp src/monitors/estop_monitor.cpp
  src/monitors/action_monitor.cpp src/monitors/controller_monitor.cpp
  src/monitors/topic_monitor.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${std_msgs_TARGETS}
         ${autonomy_manager_msgs_TARGETS}
         ${action_msgs_TARGETS}
         ${controller_manager_msgs_TARGETS}
         ${sensor_msgs_TARGETS}
         ${lifecycle_msgs_TARGETS}
         rclcpp::rclcpp
         rclcpp_components::component
         hector_ros2_utils::hector_ros2_utils)

rclcpp_components_register_nodes(${PROJECT_NAME}
                                 "autonomy_manager::AutonomyManager")

add_executable(${PROJECT_NAME}_node src/autonomy_manager_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# ##############################################################################
# Test ##
# ##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rtest REQUIRED)
  find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)
  find_package(hector_testing_utils REQUIRED)

  # Unit tests (with rtest mock doubles)
  test_tools_add_doubles(
    ${PROJECT_NAME}
    AMENT_DEPENDENCIES
    rclcpp::rclcpp
    rclcpp_components::component
    ${std_msgs_TARGETS}
    ${autonomy_manager_msgs_TARGETS}
    ${action_msgs_TARGETS}
    ${controller_manager_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
    hector_ros2_utils::hector_ros2_utils)

  ament_add_gmock(
    ${PROJECT_NAME}_test
    test/test_main.cpp
    test/test_types.cpp
    test/test_estop_monitor.cpp
    test/test_action_monitor.cpp
    test/test_controller_monitor.cpp
    test/test_topic_monitor.cpp
    test/test_autonomy_manager.cpp)

  target_link_libraries(${PROJECT_NAME}_test
                        ${PROJECT_NAME}::${PROJECT_NAME}_test_doubles)

  # Integration tests (real nodes, no mocks)
  ament_add_gtest(${PROJECT_NAME}_integration_test
                  test/test_dynamic_reconfiguration.cpp)

  target_link_libraries(${PROJECT_NAME}_integration_test ${PROJECT_NAME}
                        hector_testing_utils::hector_testing_utils)
endif()

# ##############################################################################
# Install ##
# ##############################################################################

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  autonomy_manager_msgs
  action_msgs
  controller_manager_msgs
  sensor_msgs
  lifecycle_msgs
  hector_ros2_utils)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

ament_package()
