cmake_minimum_required(VERSION 3.0)
project(ur_rtde_tests LANGUAGES CXX)

# Prepare doctest for other targets to use
find_package(doctest REQUIRED)

# find ur_rtde
set(DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "${DIR}")
find_package(ur_rtde REQUIRED PATHS "${DIR}/../Build/ur_rtde" "${DIR}/../build/ur_rtde")

# Make test executable
add_executable(tests main.cpp)
target_compile_features(tests PRIVATE cxx_std_11)
target_link_libraries(tests PRIVATE doctest::doctest PUBLIC ur_rtde::rtde)
