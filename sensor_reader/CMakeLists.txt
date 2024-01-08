cmake_minimum_required(VERSION 3.8)
project(sensor_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(sensor_reader_node ${SRC_FILE})

target_include_directories(sensor_reader_node PRIVATE include)
ament_target_dependencies(sensor_reader_node 
  rclcpp
  px4_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  sensor_reader_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
