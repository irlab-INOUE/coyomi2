cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
set (CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "")

project(coyomi2)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/GetDistance2D/inc
  /usr/local/include/urg_cpp)
link_directories(${PROJECT_SOURCE_DIR})

set (SRC_DIR ${PROJECT_SOURCE_DIR}/src)

find_package (OpenCV REQUIRED)

FILE(GLOB URG2D_FILES ${PROJECT_SOURCE_DIR}/src/GetDistance2D/src/*.cpp)

add_executable (main main.cpp
  ${SRC_DIR}/Urg2d.cpp
  ${URG2D_FILES})

add_executable (iii test.cpp)

target_link_libraries(main ${OpenCV_LIBRARIES} urg_cpp)
