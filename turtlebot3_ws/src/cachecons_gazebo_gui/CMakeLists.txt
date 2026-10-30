cmake_minimum_required(VERSION 3.8)
project(cachecons_gazebo_gui)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED COMPONENTS gui)
find_package(Qt5 REQUIRED COMPONENTS Quick)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} gz-sim8::gui Qt5::Quick)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
