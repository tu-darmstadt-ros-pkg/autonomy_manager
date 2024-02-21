cmake_minimum_required(VERSION 3.10)
project(autonomy_manager VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED) 
find_package(lifecycle_msgs REQUIRED)
find_package(autonomy_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
set(CMAKE_CXX_STANDARD 14)

###########
## Build ##
###########

include_directories(include)

add_library(${PROJECT_NAME} SHARED
        src/autonomy_manager.cpp)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        rclcpp_components
        rclcpp_lifecycle 
        lifecycle_msgs
        autonomy_manager_msgs
        geometry_msgs
        )
rclcpp_components_register_nodes(${PROJECT_NAME} "autonomy_manager::AutonomyManager")

add_executable(${PROJECT_NAME}_node src/autonomy_manager_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        )
ament_target_dependencies(${PROJECT_NAME}_node
        "rclcpp"
        "rclcpp_lifecycle" 
        "lifecycle_msgs"
        "autonomy_manager_msgs"
        "geometry_msgs"
        )
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Add the test executable
#add_executable(${PROJECT_NAME}_gtest test/test_autonomy_manager.cpp)
# Link the test executable to the library

# Add the test
ament_add_gtest(${PROJECT_NAME}_gtest test/test_autonomy_manager.cpp)
target_link_libraries(${PROJECT_NAME}_gtest ${PROJECT_NAME})

#############
## Install ##
#############
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_include_directories(include)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(
        DIRECTORY include
        DESTINATION include
)
        


ament_package()