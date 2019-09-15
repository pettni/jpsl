cmake_minimum_required(VERSION 2.8.3)
project(jps)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-g -Wall")

########## DOWNLOAD GTEST ############

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.8.0
)

FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()

enable_testing()

########### BUILD LIB ################

include_directories(include)

add_library(jps src/jps.cpp src/point.cpp src/dir.cpp)

############ BUILD TESTS ################

add_executable(test3x3 test/test3x3.cpp)
target_link_libraries(test3x3 jps gtest_main)
add_test(NAME test3x3 COMMAND test3x3)

############# TEMP MAIN #################

add_executable(main src/main.cpp)

target_link_libraries(main jps)
