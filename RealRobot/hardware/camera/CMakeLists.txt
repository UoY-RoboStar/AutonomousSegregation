cmake_minimum_required(VERSION 3.10)
project(my_cvb_project)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add your source files
add_executable(my_cvb_project src/main.cpp src/cvbcamerathread.cpp)

# Link libraries (replace with actual paths)
target_link_libraries(my_cvb_project 
                      ${OpenCV_LIBS}
                      /path/to/cvb/lib/cvb.lib
                      /path/to/cvb/lib/genicam.lib
) 
