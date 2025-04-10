cmake_minimum_required(VERSION 3.10)
project(sdl_joy)

set(CMAKE_CXX_STANDARD 17)

# SDL2のパッケージを探す
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# インクルードディレクトリとライブラリディレクトリの設定
include_directories(${SDL2_INCLUDE_DIRS})
link_directories(${SDL2_LIBRARY_DIRS})

# ソースファイル
add_executable(sdl_joy main.cpp)

# SDL2ライブラリとリンク
target_link_libraries(sdl_joy ${SDL2_LIBRARIES})
