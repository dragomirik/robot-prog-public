cmake_minimum_required(VERSION 3.24)
project("robot-prog")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# faking Arduino.h
include_directories(./test/faking/)
add_executable(Arduino ./test/faking/Arduino.cpp)
target_include_directories(Arduino PUBLIC ./test/faking)

# include own test files
file(GLOB TEST_SOURCES "test/*.cpp")
add_executable(unitTest ${TEST_SOURCES})

target_link_libraries(unitTest gtest_main)
add_test(NAME example_test COMMAND unitTest)
