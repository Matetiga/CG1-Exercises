include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_executable(Exercise1_2 MACOSX_BUNDLE
	src/main.cpp
	src/Viewer.cpp include/Viewer.h
	src/Primitives.cpp include/Primitives.h
	src/SurfaceArea.cpp include/SurfaceArea.h
	src/Volume.cpp include/Volume.h
	src/Valence.cpp include/Valence.h
	src/ShellExtraction.cpp include/ShellExtraction.h
	src/Smoothing.cpp include/Smoothing.h
	src/Stripification.cpp include/Stripification.h
	include/sample_set.h)

target_compile_features(Exercise1_2 PUBLIC cxx_std_17)
target_compile_definitions(Exercise1_2 PUBLIC _USE_MATH_DEFINES=1)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	target_compile_definitions(Exercise1_2 PRIVATE _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING=1)
	target_compile_options(Exercise1_2 PRIVATE -Wno-deprecated-declarations)
endif()

target_link_libraries(Exercise1_2 CG1Common ${LIBS})
