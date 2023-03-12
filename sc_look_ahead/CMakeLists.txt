cmake_minimum_required(VERSION 3.14)

project(sc_look_ahead LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(sc_look_ahead SHARED
  sc_look_ahead.cpp
  sc_look_ahead.h
)

target_compile_definitions(sc_look_ahead PRIVATE SC_LOOK_AHEAD_LIBRARY)
