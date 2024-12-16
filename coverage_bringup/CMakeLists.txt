cmake_minimum_required(VERSION 3.8)
project(coverage_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(coverage_path_planning REQUIRED)
find_package(navigator REQUIRED)
find_package(map_gen REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${coverage_path_planning_INCLUDE_DIRS} ${navigator_INCLUDE_DIRS} ${map_gen_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${rclpy_LIBRARIES} ${std_msgs_LIBRARIES} ${coverage_path_planning_LIBRARIES} ${navigator_LIBRARIES} ${map_gen_LIBRARIES})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(coverage_path_planning)
ament_export_dependencies(navigator)
ament_export_dependencies(map_gen)
ament_export_include_directories(${INCLUDE_DIRS})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
