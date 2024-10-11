cmake_minimum_required(VERSION 2.8.7)

project(kalibrasi)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(kalibrasi kalibrasi.cpp)
target_link_libraries(kalibrasi ${OpenCV_LIBS})


