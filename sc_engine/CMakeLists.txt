cmake_minimum_required(VERSION 3.14)

project(sc_engine LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

add_library(sc_engine SHARED
    ${SRC_DIR}/sc_formula.h
    ${SRC_DIR}/sc_engine.cpp
    ${SRC_DIR}/sc_engine.h
    )

target_compile_definitions(sc_engine PRIVATE SC_ENGINE_LIBRARY)
