cmake_minimum_required(VERSION 3.14)

project(sc_optimizer LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

add_library(sc_optimizer SHARED
    ${SRC_DIR}/../sc_engine/sc_engine.h
    ${SRC_DIR}/../sc_engine/sc_engine.cpp
    ${SRC_DIR}/../sc_common/sc_struct.h
    ${SRC_DIR}/../sc_common/sc_arcs.h
    ${SRC_DIR}/../sc_common/sc_arcs.cpp
    ${SRC_DIR}/../sc_common/sc_lines.h
    ${SRC_DIR}/../sc_common/sc_lines.cpp
    ${SRC_DIR}/../sc_interpolate/sc_interpolate.h
    ${SRC_DIR}/../sc_interpolate/sc_interpolate.cpp
    ${SRC_DIR}/sc_optimizer.cpp
    ${SRC_DIR}/sc_optimizer.h
    )

target_compile_definitions(sc_optimizer PRIVATE SC_OPTIMIZER_LIBRARY)

