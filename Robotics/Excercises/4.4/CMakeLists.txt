cmake_minimum_required(VERSION 3.8)
project(4_4)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/main.cpp src/RobworkTools.hpp)
add_executable(4_4 ${SOURCE_FILES})

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
SET(CMAKE_BUILD_TYPE debug)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# The shared library to build:
TARGET_LINK_LIBRARIES(4_4 ${ROBWORK_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")
