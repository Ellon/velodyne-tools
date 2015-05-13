cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(convert)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(PCL 1.2 REQUIRED common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (convert convert.cpp)
target_link_libraries (convert ${PCL_LIBRARIES})
