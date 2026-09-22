cmake_minimum_required(VERSION 3.8)
project(platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(main_interface REQUIRED)

# Example of packages that may be used
# find_package(irobot_create_msgs REQUIRED)
# find_package(turtlebot4_msgs REQUIRED)

add_executable(RP src/RP.cpp)
ament_target_dependencies(
  RP 
  rclcpp 
  rclcpp_action
  rclcpp_components
  std_msgs 
  # Example of package dependencies
  # irobot_create_msgs
  # turtlebot4_msgs
  main_interface
)
target_include_directories(RP PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(RP PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS RP
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
