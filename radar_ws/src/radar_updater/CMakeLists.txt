cmake_minimum_required(VERSION 3.8)
project(radar_updater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++20)
add_compile_options(-std=c++20)
set(CMAKE_CXX_FLAGS "-std=c++20")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_util REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB FUNCTIONS_SOURCE src/updater/*.cpp)

# BUILD EXE
add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  ${FUNCTIONS_SOURCE}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs serial_util OpenCV)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()