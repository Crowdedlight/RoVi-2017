cmake_minimum_required(VERSION 3.8)
project(HSVsegmentation)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(HSVsegmentation ${SOURCE_FILES})

find_package(OpenCV 3 REQUIRED)
target_link_libraries(HSVsegmentation ${OpenCV_LIBS})