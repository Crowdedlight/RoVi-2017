cmake_minimum_required(VERSION 3.8)
project(MiniProject1)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(MiniProject1 ${SOURCE_FILES})

find_package(OpenCV 3 REQUIRED)
target_link_libraries(MiniProject1 ${OpenCV_LIBS})