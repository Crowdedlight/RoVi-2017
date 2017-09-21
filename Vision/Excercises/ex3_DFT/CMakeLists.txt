cmake_minimum_required(VERSION 3.8)
project(ex3_DFT)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(ex3_DFT ${SOURCE_FILES})

find_package(OpenCV 3 REQUIRED)
target_link_libraries(ex3_DFT ${OpenCV_LIBS})