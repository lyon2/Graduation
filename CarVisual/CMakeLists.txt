cmake_minimum_required(VERSION 3.6)
project(CarVisual)

set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/opt/opencv3")
endif ()
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(lib)


set(SOURCE_FILES main.cpp test.cpp test.h lib)


add_executable(CarVisual ${SOURCE_FILES})

target_link_libraries(CarVisual easypr thirdparty ${OpenCV_LIBS} )