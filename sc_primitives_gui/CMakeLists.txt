cmake_minimum_required(VERSION 3.5)
set(PROJECT "scurve_primitives_gui")
project(${PROJECT} LANGUAGES CXX)

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

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        opengl.h
        opengl.cpp
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(scurve_primitives_gui
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(${PROJECT} SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(${PROJECT}
            ${PROJECT_SOURCES}
        )
    endif()
endif()

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Set include paths for libraries to link.
add_library(engine SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(engine PROPERTIES
    IMPORTED_LOCATION "${BUILD_DIR}/../sc_engine/libsc_engine.so"
    )

target_link_libraries(${PROJECT} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets -lGL pthread rt engine)

# Make install
install(TARGETS ${PROJECT} DESTINATION ${BUILD_DIR}/bin)

# Notes
