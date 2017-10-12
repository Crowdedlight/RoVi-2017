cmake_minimum_required(VERSION 3.8)
project(ex6_contours)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(ex6_contours ${SOURCE_FILES})

find_package(OpenCV 3 REQUIRED)
target_link_libraries(ex6_contours ${OpenCV_LIBS})
