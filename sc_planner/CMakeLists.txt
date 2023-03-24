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
    ${SRC_DIR}/../sc_engine/sc_engine.h
    ${SRC_DIR}/../sc_engine/sc_engine.cpp
    ${SRC_DIR}/../sc_interpolate/sc_interpolate.h
    ${SRC_DIR}/../sc_interpolate/sc_interpolate.cpp
    ${SRC_DIR}/../sc_optimizer/sc_optimizer.h
    ${SRC_DIR}/../sc_optimizer/sc_optimizer.cpp
    )

target_compile_definitions(sc_planner PRIVATE SC_PLANNER_LIBRARY eigen3)
