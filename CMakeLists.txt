cmake_minimum_required(VERSION 3.8)
project(fleet_adapter)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
    include
    ${ament_cmake_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rclcpp_action_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${nav2_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

set(SOURCES
    src/lib/robot.cpp
    src/lib/fleet_adapter.cpp
)

add_library(${PROJECT_NAME}_lib ${SOURCES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
    ament_cmake
    rclcpp
    rclcpp_components
    rclcpp_action
    nav_msgs
    nav2_msgs
    tf2
    tf2_ros
    tf2
    tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
    ament_index_cpp::ament_index_cpp
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_traffic_ros2::rmf_traffic_ros2
)

ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
    rclcpp
    rclcpp_components
    rclcpp_action
    rmf_fleet_adapter
    rmf_traffic_ros2
    nav_msgs
    nav2_msgs
    tf2
    tf2_ros
    tf2
    tf2_geometry_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}_libTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_lib)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()