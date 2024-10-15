cmake_minimum_required(VERSION 3.12)
project(coverage_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()  

set(CGAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../CGAL-6.0")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Qt6)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArr.msg"
  "action/CoverageAction.action"
  DEPENDENCIES
  geometry_msgs
)

include_directories(include 
                    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp 
                    ${CGAL_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
add_compile_options(-Wall -Werror -Wno-unused -fPIC)

set(SOURCES
  src/coverage_path_planning.cpp
  src/LKHWrapper.cpp
)

add_executable(CoveragePathPlanning ${SOURCES})

ament_target_dependencies(
  CoveragePathPlanning
  ament_cmake
  rclcpp
  rclcpp_action
  rosidl_default_runtime
  geometry_msgs
  ament_index_cpp
)

rosidl_target_interfaces(CoveragePathPlanning ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(CoveragePathPlanning
                      CGAL::CGAL 
                      rclcpp::rclcpp  
                      Qt6::Core 
                      Qt6::Widgets
                      CGAL::CGAL_Basic_viewer
)

install(TARGETS CoveragePathPlanning DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts USE_SOURCE_PERMISSIONS
        DESTINATION lib/${PROJECT_NAME})

ament_package()
