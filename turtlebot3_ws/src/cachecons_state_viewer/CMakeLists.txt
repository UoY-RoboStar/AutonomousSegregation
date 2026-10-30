cmake_minimum_required(VERSION 3.8)
project(cachecons_state_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME}
  src/cachecons_state_viewer.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME}
  simulation_interfaces
  rclcpp
  std_msgs
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY images/
  DESTINATION share/${PROJECT_NAME}/images
)

ament_package()
