cmake_minimum_required(VERSION 3.8)
project(sbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs)

add_executable(sbus_node src/sbus.cpp)
ament_target_dependencies(sbus_node rclcpp sensor_msgs)
target_include_directories(sbus_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  sbus_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
