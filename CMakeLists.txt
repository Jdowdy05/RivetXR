cmake_minimum_required(VERSION 3.24)

project(quest_newton LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(cmake/CompilerWarnings.cmake)

set(NEWTON_VERSION "1.6.0.dev0" CACHE STRING "Pinned Newton version")
set(WARP_VERSION "1.18.0.dev2" CACHE STRING "Pinned Warp version")
set(NEWTON_COMMIT "d37f4d3d341ccce1e06a1dff21e9a054759b4855"
    CACHE STRING "Pinned Newton source commit")
set(WARP_COMMIT "d4de134b97b961f1a19bd76830e71ea7f9df2470"
    CACHE STRING "Pinned Warp source commit")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/newton_runtime/CMakeLists.txt")
  add_subdirectory(src/newton_runtime)
endif()

add_subdirectory(src/kinematics)
add_subdirectory(tests)
