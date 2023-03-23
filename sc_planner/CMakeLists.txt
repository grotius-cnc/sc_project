cmake_minimum_required(VERSION 3.14)

project(sc_planner LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

add_library(sc_planner SHARED
    ${SRC_DIR}/sc_planner.cpp
    ${SRC_DIR}/sc_planner.h
    )

# Set include paths for libraries to link.
add_library(engine SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(engine PROPERTIES
    IMPORTED_LOCATION "${BUILD_DIR}/../sc_engine/libsc_engine.so"
    )

add_library(interpolate SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(interpolate PROPERTIES
    IMPORTED_LOCATION "${BUILD_DIR}/../sc_interpolate/libsc_interpolate.so"
    )

target_compile_definitions(sc_planner PRIVATE SC_PLANNER_LIBRARY engine interpolate eigen3)
