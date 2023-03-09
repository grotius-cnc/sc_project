cmake_minimum_required(VERSION 3.14)

project(sc_interpolate LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(sc_interpolate SHARED
    sc_struct.h
    sc_interpolate.h
    sc_interpolate.cpp
    sc_lines.h
    sc_lines.cpp
    sc_arcs.h
    sc_arcs.cpp
    )

target_compile_definitions(sc_interpolate PRIVATE SC_INTERPOLATE_LIBRARY eigen3)
