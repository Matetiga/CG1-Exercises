
FUNCTION(PREPEND var prefix)
	SET(listVar "")
	FOREACH(f ${ARGN})
		LIST(APPEND listVar "${prefix}/${f}")
	ENDFOREACH(f)
	SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(JOIN VALUES GLUE OUTPUT)
	string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
	string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
	set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
ENDFUNCTION()

FUNCTION(ProcessGLSLFiles GLSL_FILES_VAR)
	set(GLSL_FILES ${${GLSL_FILES_VAR}})
	PREPEND(GLSL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/glsl/" ${GLSL_FILES})	
	set(${GLSL_FILES_VAR} ${GLSL_FILES} PARENT_SCOPE)
	
	source_group(glsl FILES ${GLSL_FILES} glsl.cpp glsl.h)

	JOIN("${GLSL_FILES}" "," glsl_string)
	set(bin2c_cmdline
		-DOUTPUT_C=glsl.cpp
		-DOUTPUT_H=glsl.h
		"-DINPUT_FILES=${glsl_string}"
		-P "${NANOGUI_DIR}/resources/bin2c.cmake")

	add_custom_command(
		OUTPUT glsl.cpp glsl.h
		COMMAND ${CMAKE_COMMAND} ARGS ${bin2c_cmdline}
		DEPENDS ${GLSL_FILES}
		COMMENT "Running bin2c"
		PRE_BUILD VERBATIM)
ENDFUNCTION()

FUNCTION(SetupBuildEnvironment)
	# Enable folders for projects in Visual Studio
	if(CMAKE_GENERATOR MATCHES "Visual Studio")
		set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	endif()

	# Sanitize build environment for static build with C++14
	set(MICROSOFT_CRT_DEFINES "_CRT_SECURE_NO_WARNINGS=1;__TBB_NO_IMPLICIT_LINKAGE=1;NOMINMAX")
	if(MSVC)
		# Get more familiar MSVC standard library behaviour
		add_compile_definitions(${MICROSOFT_CRT_DEFINES})

		# Parallel build on MSVC (all targets)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

		if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
			# Enable use of SIMD for 32bit builds...
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")

			# ... but disable Eigen vectorization as it causes segfaults on 32bit MSVC builds
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DEIGEN_DONT_ALIGN")
	 	 endif()

		# Static build
		set(CompilerFlags
				CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
				CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
				CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
				CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
		foreach(CompilerFlag ${CompilerFlags})
			string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
		endforeach()
	elseif(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		add_compile_definitions(${MICROSOFT_CRT_DEFINES})
	elseif(APPLE)
		# Try to auto-detect a suitable SDK
		execute_process(COMMAND bash -c "xcodebuild -version -sdk | grep MacOSX | grep Path | head -n 1 | cut -f 2 -d ' '" OUTPUT_VARIABLE CMAKE_OSX_SYSROOT)
		string(REGEX REPLACE "(\r?\n)+$" "" CMAKE_OSX_SYSROOT "${CMAKE_OSX_SYSROOT}")
		string(REGEX REPLACE "^.*X([0-9.]*).sdk$" "\\1" CMAKE_OSX_DEPLOYMENT_TARGET "${CMAKE_OSX_SYSROOT}")
	endif()

	# Export settings to global scope so they're used as basis for later per-target configurations
	set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} PARENT_SCOPE) 
	set(CMAKE_CXX_STANDARD 14 PARENT_SCOPE)
ENDFUNCTION()

FUNCTION(FinishBuildEnvironment)
	# Inject compile options into 3rd party targets to try and get rid of as many warnings as possible
	# - NanoGUI
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		# Common to both GCC and Clang
		if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
			target_compile_options(nanogui-obj PRIVATE -Wno-unused-command-line-argument -Wno-unknown-pragmas -Wno-deprecated-declarations -Wno-unused-result -Wno-misleading-indentation -Wno-implicit-fallthrough)
		endif()

		# Special options depending on whether it's GCC or Clang
		if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
			# Warnings generated all over the code base
			target_compile_options(nanogui-obj PRIVATE -Wno-maybe-uninitialized -Wno-format-truncation -Wno-cast-function-type -Wno-dangling-pointer)
			# Warnings we need to remove surgically from certain files in order to prevent other warnings from appearing
			set_source_files_properties("${NANOGUI_DIR}/src/button.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/checkbox.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/colorpicker.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/colorwheel.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/combobox.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/common.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/glcanvas.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/glutil.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/graph.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/imagepanel.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/imageview.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/label.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/layout.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/messagedialog" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/popup.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/popupbutton.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/progressbar.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/screen.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/serializer.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/slider.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS "-Wno-deprecated-copy -Wno-mismatched-new-delete")
			set_source_files_properties("${NANOGUI_DIR}/src/stackedwidget.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/tabheader.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/tabwidget.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/textbox.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/vscrollpanel.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/window.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
			set_source_files_properties("${NANOGUI_DIR}/src/widget.cpp" TARGET_DIRECTORY nanogui-obj PROPERTIES COMPILE_FLAGS -Wno-deprecated-copy)
		elseif(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
			target_compile_options(nanogui-obj PRIVATE -Wno-unknown-warning-option -Wno-bad-function-cast -Wno-deprecated-copy-with-user-provided-copy)
		endif()
	endif()
	# - GLFW
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		target_compile_options(glfw_objects PRIVATE -Wno-format-truncation)
	endif()
	# - OpenMesh
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		target_compile_options(OpenMeshCore PRIVATE -Wno-unused-result -Wno-deprecated-copy)
		if(TARGET OpenMeshCoreStatic)
			target_compile_options(OpenMeshCoreStatic PRIVATE -Wno-unused-result -Wno-deprecated-copy)
		endif()
		target_compile_options(OpenMeshTools PRIVATE -Wno-deprecated-copy)
		if(TARGET OpenMeshToolsStatic)
			target_compile_options(OpenMeshToolsStatic PRIVATE -Wno-deprecated-copy)
		endif()
	endif()
ENDFUNCTION()
