include_directories(
	/opt/homebrew/include
	)

add_library(matrix matrix.cpp)
target_link_libraries(matrix ${OpenCV_LIBRARIES})

# make install でインストールする場所の指定
install(TARGETS matrix ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/lib)
install(FILES matrix.h DESTINATION ${PROJECT_SOURCE_DIR}/include)
