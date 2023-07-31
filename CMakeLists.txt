cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
set (CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "")

project(coyomi2)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/GetDistance2D/inc
  ${PROJECT_BINARY_DIR}
  /usr/local/include/urg_cpp)

link_directories(${PROJECT_SOURCE_DIR})

set (DEFAULT_LOG_DIR ${PROJECT_SOURCE_DIR})
set (DEFAULT_ROOT ${PROJECT_SOURCE_DIR})
set (SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set (BIN_DIR ${PROJECT_SOURCE_DIR}/bin)

configure_file (
    "${PROJECT_SOURCE_DIR}/Config.h.in"
    "${PROJECT_BINARY_DIR}/Config.h"
    )

find_package (OpenCV REQUIRED)

FILE(GLOB URG2D_FILES ${PROJECT_SOURCE_DIR}/src/GetDistance2D/src/*.cpp)

#### 実行ファイルの作成
set(EXECUTABLE_OUTPUT_PATH ${BIN_DIR})   # 実行ファイルの保存場所
add_executable (main main.cpp
  ${SRC_DIR}/Urg2d.cpp
  ${URG2D_FILES})

add_executable (iii test.cpp)
add_executable (battery_log battery_log.cpp)

target_link_libraries(main yaml-cpp ${OpenCV_LIBRARIES} urg_cpp)
