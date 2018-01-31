cmake_minimum_required(VERSION 3.8)
project(FinalProject_Marker1)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(FinalProject_Marker1 ${SOURCE_FILES})

find_package(OpenCV 3 REQUIRED)
target_link_libraries(FinalProject_Marker1 ${OpenCV_LIBS})
