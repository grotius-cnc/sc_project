cmake_minimum_required(VERSION 3.5)

project(sc_planner_gui LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

set(PROJECT_SOURCES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/mainwindow.cpp
    ${SRC_DIR}/mainwindow.h
    ${SRC_DIR}/mainwindow.ui
    ${SRC_DIR}/opengl.h
    ${SRC_DIR}/opengl.cpp
    ${SRC_DIR}/../sc_engine/sc_engine.h
    ${SRC_DIR}/../sc_engine/sc_engine.cpp
    ${SRC_DIR}/../sc_common/sc_struct.h
    ${SRC_DIR}/../sc_common/sc_lines.h
    ${SRC_DIR}/../sc_common/sc_lines.cpp
    ${SRC_DIR}/../sc_common/sc_arcs.h
    ${SRC_DIR}/../sc_common/sc_arcs.cpp
    ${SRC_DIR}/../sc_interpolate/sc_interpolate.h
    ${SRC_DIR}/../sc_interpolate/sc_interpolate.cpp
    ${SRC_DIR}/../sc_optimizer/sc_optimizer.h
    ${SRC_DIR}/../sc_optimizer/sc_optimizer.cpp
    ${SRC_DIR}/../sc_pid/sc_pid.h
    ${SRC_DIR}/../sc_pid/sc_pid.cpp
    ${SRC_DIR}/../sc_planner/sc_planner.h
    ${SRC_DIR}/../sc_planner/sc_planner.cpp
    )

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(sc_planner_gui
        ${PROJECT_SOURCES}
        )
else()
    if(ANDROID)
        add_library(sc_planner_gui SHARED
            ${PROJECT_SOURCES}
            )
    else()
        add_executable(sc_planner_gui
            ${PROJECT_SOURCES}
            )
    endif()
endif()

target_link_libraries(sc_planner_gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets -lGL)



















