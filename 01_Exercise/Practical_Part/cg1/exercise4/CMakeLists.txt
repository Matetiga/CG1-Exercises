set(GLSL_FILES	sky.vert sky.frag
				terrain.vert terrain.frag)

ProcessGLSLFiles(GLSL_FILES)

set(TEXTURE_FILES grass.jpg rock.jpg roadColor.jpg roadNormals.jpg roadSpecular.jpg alpha.jpg)
PREPEND(TEXTURE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/" ${TEXTURE_FILES})
JOIN("${TEXTURE_FILES}" "," texture_string)
set(bin2c_cmdline
		-DOUTPUT_C=textures.cpp
		-DOUTPUT_H=textures.h
		"-DINPUT_FILES=${texture_string}"
		-P "${NANOGUI_DIR}/resources/bin2c.cmake")

	add_custom_command(
	  OUTPUT textures.cpp textures.h
	  COMMAND ${CMAKE_COMMAND} ARGS ${bin2c_cmdline}
	  DEPENDS ${TEXTURE_FILES}
	  COMMENT "Running bin2c"
	  PRE_BUILD VERBATIM)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_executable(Exercise4 MACOSX_BUNDLE
	glsl.cpp
	textures.cpp
	src/main.cpp
	src/Viewer.cpp include/Viewer.h
	${GLSL_FILES})
target_compile_definitions(Exercise4 PUBLIC _USE_MATH_DEFINES=1)

target_link_libraries(Exercise4 CG1Common ${LIBS})
