cmake_minimum_required(VERSION 2.8.3)
project(jps)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-g -O3 -s -DNDEBUG -Wall")   # release
# set(CMAKE_CXX_FLAGS "-g -Wall")                 # debug

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

add_library(jps src/jps.cpp src/point.cpp src/dir.cpp src/encodings.cpp src/lookup_table.cpp)

############ BUILD TESTS ################

add_executable(test_3x3 test/test_3x3.cpp)
target_link_libraries(test_3x3 jps gtest_main)
add_test(NAME test_3x3 COMMAND test_3x3)

add_executable(test_jump test/test_jump.cpp)
target_link_libraries(test_jump jps gtest_main)
add_test(NAME test_jump COMMAND test_jump)

add_executable(test_jps test/test_jps.cpp)
target_link_libraries(test_jps jps gtest_main)
add_test(NAME test_jps COMMAND test_jps)

add_executable(test_standardize_dir test/test_standardize_dir.cpp)
target_link_libraries(test_standardize_dir jps gtest_main)
add_test(NAME test_standardize_dir COMMAND test_standardize_dir)

add_executable(test_encodings test/test_encodings.cpp)
target_link_libraries(test_encodings jps gtest_main)
add_test(NAME test_encodings COMMAND test_encodings)

############# TEMP MAIN #################

add_executable(main src/main.cpp)

target_link_libraries(main jps)

add_executable(map_3d examples/map_3d.cpp)
target_link_libraries(map_3d jps)

add_executable(generate_lookup_table src/generate_lookup_table.cpp)
target_link_libraries(generate_lookup_table jps)