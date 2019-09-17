cmake_minimum_required(VERSION 3.1.0)
project(jpsl)

add_compile_options(-std=c++17)
# set(CMAKE_CXX_FLAGS "-g -O3 -s -DNDEBUG -Wall")   # release
set(CMAKE_CXX_FLAGS "-g -Wall")                 # debug

########### BUILD LIB ################

include_directories(include)
add_library(jpsl src/jpsl.cpp src/point.cpp src/dir.cpp src/encodings.cpp src/lookup_table.cpp)

########### TESTING ############

# Download GTest
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

find_package(GTest)
enable_testing()

# Build tests
add_executable(test_3x3 test/test_3x3.cpp)
target_link_libraries(test_3x3 jpsl gtest_main)
gtest_discover_tests(test_3x3)

add_executable(test_jump test/test_jump.cpp)
target_link_libraries(test_jump jpsl gtest_main)
gtest_discover_tests(test_jump)

add_executable(test_jps test/test_jps.cpp)
target_link_libraries(test_jps jpsl gtest_main)
gtest_discover_tests(test_jps)

add_executable(test_standardize_dir test/test_standardize_dir.cpp)
target_link_libraries(test_standardize_dir jpsl gtest_main)
gtest_discover_tests(test_standardize_dir)

add_executable(test_encodings test/test_encodings.cpp)
target_link_libraries(test_encodings jpsl gtest_main)
gtest_discover_tests(test_encodings)

############# EXECUTABLES #################

add_executable(map_3d examples/map_3d.cpp)
target_link_libraries(map_3d jpsl)

add_executable(generate_lookup_table src/generate_lookup_table.cpp)
target_link_libraries(generate_lookup_table jpsl)

########### INSTALL ##################

install(TARGETS jpsl 
        ARCHIVE DESTINATION lib)

install(FILES "include/jpsl/jpsl.hpp" 
              "include/jpsl/point.hpp"
              "include/jpsl/encodings.hpp"
              "include/jpsl/dir.hpp"
        DESTINATION "include/jpsl")
