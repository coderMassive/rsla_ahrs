cmake_minimum_required(VERSION 3.10)
project(rsla_ahrs)

add_subdirectory(../vnsdk-1-0-0/cpp VnSensor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

target_compile_options(VnSensor PRIVATE
-Wno-format-overflow
-Wno-format-truncation
-Wno-pedantic)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(rsla_ahrs src/main.cpp)
ament_target_dependencies(rsla_ahrs rclcpp std_msgs geometry_msgs tf2 tf2_geometry_msgs)

target_link_libraries(rsla_ahrs VnSensor)

install(TARGETS rsla_ahrs VnSensor
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

