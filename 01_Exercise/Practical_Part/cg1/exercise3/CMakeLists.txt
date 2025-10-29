set(GLSL_FILES	shader.vert shader.frag)

ProcessGLSLFiles(GLSL_FILES)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_executable(Exercise3 MACOSX_BUNDLE
	glsl.cpp
	src/main.cpp
	src/Viewer.cpp include/Viewer.h
	${GLSL_FILES})

target_link_libraries(Exercise3 CG1Common ${LIBS})