include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR})

set(GLSL_FILES	mesh.vert mesh.frag
				simple.vert simple.frag)
ProcessGLSLFiles(GLSL_FILES)

add_library(CG1Common STATIC
	src/gui/AbstractViewer.cpp
	src/gui/Camera.cpp
	src/gui/GLBuffer.cpp
	src/gui/GLShader.cpp
	src/gui/GLVertexArray.cpp
	src/gui/SliderHelper.cpp
	src/gui/ShaderPool.cpp
	
	src/util/GLDebug.cpp
	src/util/UnionFind.cpp
	src/util/OpenMeshUtils.cpp

	glsl.cpp)

target_compile_definitions(CG1Common PUBLIC _USE_MATH_DEFINES=1)
