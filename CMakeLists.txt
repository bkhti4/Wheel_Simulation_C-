cmake_minimum_required(VERSION 3.15)

project(WheelSim VERSION 0.1 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

set(PROJECT_SOURCES
        src/main.cpp
        src/CarSim.h
        src/WheelSim.cpp
        src/WheelSim.h
        src/WheelVis.cpp
        src/WheelVis.h
        src/wheelvisualizer.ui
        fig/steering-icon.png
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(WheelSim
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
	add_executable(WheelSim
		${PROJECT_SOURCES}
	)
endif()

target_link_libraries(WheelSim PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(WheelSim)
endif()
