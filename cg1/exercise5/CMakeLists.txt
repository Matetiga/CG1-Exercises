include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_executable(Exercise5 MACOSX_BUNDLE
	src/main.cpp
	src/Viewer.cpp include/Viewer.h
	src/AABBTree.cpp include/AABBTree.h
	src/Box.cpp include/Box.h
	src/LineSegment.cpp include/LineSegment.h
	src/Point.cpp include/Point.h
	src/Triangle.cpp include/Triangle.h
	include/GridUtils.h
	src/HashGrid.cpp include/HashGrid.h
	src/GridTraverser.cpp include/GridTraverser.h
	)

target_link_libraries(Exercise5 CG1Common ${LIBS})