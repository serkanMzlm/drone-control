cmake_minimum_required(VERSION 3.8)
project(controller)

set(node_name "${PROJECT_NAME}_node")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp
                                ${PROJECT_SOURCE_DIR}/src/*.c)

add_executable(controller_node ${SRC_FILE})

target_include_directories(controller_node PRIVATE include)

ament_target_dependencies(controller_node 
  rclcpp
  px4_msgs
  sensor_msgs
)

install(TARGETS
  controller_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
