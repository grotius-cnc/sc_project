cmake_minimum_required(VERSION 3.0)
# Project name.
set(PROJECT "- project_scurve -") # The name of the file excluding .c or .h
project(${PROJECT} CXX C) # Asm is optional.

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Set path to your qt dir.

# For a qt install, you could do $ sudo apt-get install qtcreator
set(CMAKE_PREFIX_PATH "/opt/qt-creator/5.15.1/gcc_64/lib/cmake/Qt5")
# Set path to eigen3, for install $ sudo apt-get install libeigen3-dev
include_directories("/usr/include/eigen3/Eigen")

# Core libs:
add_subdirectory(${SRC_DIR}/sc_engine)
add_subdirectory(${SRC_DIR}/sc_interpolate)
add_subdirectory(${SRC_DIR}/sc_planner)
# add_subdirectory(${SRC_DIR}/sc_pid)
add_subdirectory(${SRC_DIR}/sc_optimizer)

# Graphical interfaces:
# add_subdirectory(${SRC_DIR}/sc_primitives_gui)
# add_subdirectory(${SRC_DIR}/sc_curves_gui)
# add_subdirectory(${SRC_DIR}/sc_planner_gui)
# add_subdirectory(${SRC_DIR}/sc_pid_gui)
add_subdirectory(${SRC_DIR}/sc_optimizer_gui)
