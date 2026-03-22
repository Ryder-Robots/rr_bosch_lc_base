cmake_minimum_required(VERSION 3.20)
project(rr_bosch_lc_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rr_bno055 REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rr_bosch_common_sensor.cpp
  src/rr_bosch_factory.cpp
  src/rr_bosch_imu_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${sensor_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
  sensor_msgs::sensor_msgs_library
  rr::rr_bno055
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
