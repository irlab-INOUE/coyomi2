cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
set (CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "")

project(coyomi2)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/GetDistance2D/inc
  ${PROJECT_SOURCE_DIR}/src/GetDistance3D
  ${PROJECT_BINARY_DIR}
  /opt/homebrew/opt/lua@5.4/include/lua
  /usr/local/include/urg_cpp
  /usr/local/include)

link_directories(${PROJECT_SOURCE_DIR}
  /usr/local/lib)

set (DEFAULT_LOG_DIR ${PROJECT_SOURCE_DIR})
set (DEFAULT_ROOT ${PROJECT_SOURCE_DIR})
set (SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set (BIN_DIR ${PROJECT_SOURCE_DIR}/bin)

configure_file (
    "${PROJECT_SOURCE_DIR}/Config.h.in"
    "${PROJECT_BINARY_DIR}/Config.h"
    )

find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)

FILE(GLOB URG2D_FILES ${PROJECT_SOURCE_DIR}/src/GetDistance2D/src/*.cpp)
set(URG3D_FILES
  ${SRC_DIR}/GetDistance3D/GetUrg3d.cpp
  ${SRC_DIR}/GetDistance3D/matrix.cpp
)

# SDL2のパッケージを探す
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
# インクルードディレクトリとライブラリディレクトリの設定
include_directories(${SDL2_INCLUDE_DIRS})
link_directories(${SDL2_LIBRARY_DIRS})

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

#### 実行ファイルの作成
set(EXECUTABLE_OUTPUT_PATH ${BIN_DIR})   # 実行ファイルの保存場所
add_executable (main main.cpp
  ${SRC_DIR}/Urg2d.cpp
  ${SRC_DIR}/MCL.cpp
  ${SRC_DIR}/Viewer.cpp
  ${SRC_DIR}/DWA.cpp
  ${SRC_DIR}/CreateCostMap.cpp
  ${SRC_DIR}/DELFM.cpp
  ${SRC_DIR}/MCL.cpp
  ${URG2D_FILES}
  ${URG3D_FILES}
  )
target_link_libraries(main yaml-cpp ${OpenCV_LIBRARIES} urg_cpp urg3d ncurses ${SDL2_LIBRARIES} ${CURSES_LIBRARIES})

add_executable (data_collect data_collect.cpp
  ${SRC_DIR}/Urg2d.cpp
  ${SRC_DIR}/MCL.cpp
  ${SRC_DIR}/Viewer.cpp
  ${SRC_DIR}/DWA.cpp
  ${SRC_DIR}/CreateCostMap.cpp
  ${SRC_DIR}/DELFM.cpp
  ${SRC_DIR}/MCL.cpp
  ${URG2D_FILES}
  ${URG3D_FILES}
  )
target_link_libraries(data_collect yaml-cpp ${OpenCV_LIBRARIES} urg_cpp urg3d ncurses ${SDL2_LIBRARIES} ${CURSES_LIBRARIES})

add_executable (slam slam.cpp)
target_link_libraries(slam ${OpenCV_LIBRARIES} Eigen3::Eigen)

add_executable (manual_loop_closure manual_loop_closure.cpp)
target_link_libraries(manual_loop_closure yaml-cpp ${OpenCV_LIBRARIES})
