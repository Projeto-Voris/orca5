cmake_minimum_required(VERSION 3.22.1)
project(voris_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(rc_control src/rc_control.cpp)

ament_target_dependencies(rc_control 
rclcpp
sensor_msgs
mavros_msgs
geometry_msgs
nav_msgs
nav_msgs
tf2_geometry_msgs
)

install(TARGETS rc_control 
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()