cmake_minimum_required(VERSION 3.14)

project(sc_engine LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(sc_engine SHARED
    sc_formula.h
    sc_engine.cpp
    sc_engine.h
    )

target_compile_definitions(sc_engine PRIVATE SC_ENGINE_LIBRARY)
