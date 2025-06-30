cmake_minimum_required(VERSION 3.8)
project(cachecons_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(cachecons_interfaces REQUIRED)

include_directories(include)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)

target_link_libraries(obstacle_avoidance rclcpp::rclcpp ${std_msgs_TARGETS})

ament_target_dependencies(obstacle_avoidance rclcpp irobot_create_msgs cachecons_interfaces)

target_include_directories(obstacle_avoidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(obstacle_avoidance PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME})

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
