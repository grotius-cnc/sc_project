cmake_minimum_required(VERSION 3.5)

project(sc_optimizer_gui LANGUAGES CXX)

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

# Set include paths for libraries to link.
add_library(optimizer SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(optimizer PROPERTIES
    IMPORTED_LOCATION "${BUILD_DIR}/../sc_optimizer/libsc_optimizer.so"
    )

set(PROJECT_SOURCES
    ${SRC_DIR}/../sc_optimizer/sc_optimizer.h
    ${SRC_DIR}/../sc_optimizer/sc_optimizer.cpp
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/mainwindow.cpp
    ${SRC_DIR}/mainwindow.h
    ${SRC_DIR}/mainwindow.ui
    )

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(sc_optimizer_gui
        ${PROJECT_SOURCES}
        )
else()
    if(ANDROID)
        add_library(sc_optimizer_gui SHARED
            ${PROJECT_SOURCES}
            )
    else()
        add_executable(sc_optimizer_gui
            ${PROJECT_SOURCES}
            )
    endif()
endif()

target_link_libraries(sc_optimizer_gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets optimizer)
