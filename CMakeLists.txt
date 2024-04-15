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

file(GLOB TEST_SOURCES "test/*.cpp")
add_executable(test1 ${TEST_SOURCES})

target_link_libraries(test1 gtest_main)
add_test(NAME example_test COMMAND test1)

# message(STATUS "Fichiers sources liés à la cible test1:")
# foreach(source ${TEST_SOURCES_LIST})
#     message(STATUS "   ${source}")
# endforeach()

# message(STATUS "Répertoires d'inclusion liés à la cible test1:")
# foreach(include_dir ${TEST_INCLUDES_LIST})
#     message(STATUS "   ${include_dir}")
# endforeach()