cmake_minimum_required(VERSION 3.10)
project(SeparateFrames)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(yaml-cpp REQUIRED)

add_executable(separate_frames separate_frames.cpp)
target_link_libraries(separate_frames ${PCL_LIBRARIES})

add_executable(align_map align_map.cpp)
target_link_libraries(align_map ${PCL_LIBRARIES} yaml-cpp)

add_executable(align_routes align_routes.cpp)
target_link_libraries(align_routes ${PCL_LIBRARIES} yaml-cpp)
