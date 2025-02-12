cmake_minimum_required(VERSION 3.8)
project(telemetry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(telemetry_interfaces REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(rmw_fastrtps_shared_cpp REQUIRED)

add_executable(telemetry_master src/telemetry_master.cpp src/main.cpp)
add_executable(telemetry_tester src/telemetry_tester.cpp)

ament_target_dependencies(telemetry_master rclcpp telemetry_interfaces rosidl_typesupport_cpp rmw_fastrtps_cpp rmw_fastrtps_shared_cpp)
ament_target_dependencies(telemetry_tester rclcpp telemetry_interfaces rosidl_typesupport_cpp rmw_fastrtps_cpp rmw_fastrtps_shared_cpp)


install(TARGETS telemetry_master telemetry_tester
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
