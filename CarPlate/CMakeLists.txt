cmake_minimum_required(VERSION 3.6)
project(CarPlate)

set(CMAKE_CXX_STANDARD 11)

#set(SOURCE_FILES main.cpp CPlateLocate.cpp CPlateLocate.h MyTest.cpp MyTest.h svm_train.cpp)
set(SOURCE_FILES  ColorLocate.cpp)
add_executable(CarPlate ${SOURCE_FILES})
find_package( OpenCV REQUIRED )
target_link_libraries( CarPlate ${OpenCV_LIBS} )