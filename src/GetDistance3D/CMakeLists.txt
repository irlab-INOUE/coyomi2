cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
set (CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "")

project(GetDistance3D)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  /usr/local/include/urg_cpp)

link_directories(${PROJECT_SOURCE_DIR}
  /usr/local/lib/)

find_package (OpenCV REQUIRED)

#### 実行ファイルの作成
add_executable(GetDistance3D
    GetDistance3D.cpp
    GetUrg3d.cpp
    matrix.cpp
    )

add_executable(data_analysis data_analysis.cpp GetUrg3d.cpp matrix.cpp)

target_link_libraries(GetDistance3D urg3d ${OpenCV_LIBRARIES})
target_link_libraries(data_analysis urg3d ${OpenCV_LIBRARIES})
