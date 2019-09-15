cmake_minimum_required(VERSION 2.8.3)
project(jps)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-g -Wall")

include_directories(include)

add_library(jps src/jps.cpp)

add_executable(main src/main.cpp)

target_link_libraries(main jps)