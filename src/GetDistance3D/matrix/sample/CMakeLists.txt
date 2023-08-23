cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -std=c++11")
project(matrix_samples)

# インクルードファイルのパスは適時置き換えること
include_directories(/usr/local/include ../../../include)

find_package (OpenCV REQUIRED)

add_executable (main main.cpp)
target_link_libraries(main ${OpenCV_LIBRARIES})

