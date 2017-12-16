cmake_minimum_required(VERSION 3.8)
project(FinalProject_Marker3)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(FinalProject_Marker3 ${SOURCE_FILES})

find_package(OpenCV 3 REQUIRED)
target_link_libraries(FinalProject_Marker3 ${OpenCV_LIBS})
