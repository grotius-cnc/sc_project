cmake_minimum_required(VERSION 3.0)
# Project name.
set(PROJECT "- project_scurve -") # The name of the file excluding .c or .h
project(${PROJECT} CXX C) # Asm is optional.

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Set the path to your qt here.
set(CMAKE_PREFIX_PATH "/opt/qt-creator/5.15.1/gcc_64/lib/cmake/Qt5")

add_subdirectory(${SRC_DIR}/sc_engine)
add_subdirectory(${SRC_DIR}/sc_primitives_gui)
add_subdirectory(${SRC_DIR}/sc_curves_gui)
add_subdirectory(${SRC_DIR}/sc_planner)
