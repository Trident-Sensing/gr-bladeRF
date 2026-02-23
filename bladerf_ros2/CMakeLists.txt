cmake_minimum_required(VERSION 3.14)
project(bladerf_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find libbladeRF via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBBLADERF REQUIRED libbladeRF)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IQSamples.msg"
  DEPENDENCIES std_msgs
)

# Composable node shared library
add_library(bladerf_source_component SHARED
  src/bladerf_source_node.cpp
)

target_include_directories(bladerf_source_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBBLADERF_INCLUDE_DIRS}
)

target_link_libraries(bladerf_source_component
  ${LIBBLADERF_LIBRARIES}
)

ament_target_dependencies(bladerf_source_component
  rclcpp
  rclcpp_components
  std_msgs
)

# Link generated message typesupport into the component
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(bladerf_source_component "${cpp_typesupport_target}")

# Register as composable node and produce a standalone executable
rclcpp_components_register_node(bladerf_source_component
  PLUGIN "bladerf_ros2::BladeRFSourceNode"
  EXECUTABLE bladerf_source_node
)

install(TARGETS bladerf_source_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
