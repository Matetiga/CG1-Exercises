cmake_minimum_required (VERSION 3.5)
project(CG1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/Modules/")

include(CG1Helper)

set(LIBS)

SetupBuildEnvironment()

# Add NanoGUI
set(NANOGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui)
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_SHARED  OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_USE_GLAD ON CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui ext_build/nanogui)
set_property(TARGET nanogui nanogui-obj glfw glfw_objects PROPERTY FOLDER "dependencies")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/include
					${NANOGUI_EXTRA_INCS})
set(LIBS ${LIBS} nanogui ${NANOGUI_EXTRA_LIBS})
add_definitions(${NANOGUI_EXTRA_DEFS})

# Add OpenMesh
set(BUILD_APPS OFF CACHE BOOL " " FORCE)
set(OPENMESH_BUILD_PYTHON_BINDINGS OFF CACHE BOOL " " FORCE)
set(OPENMESH_BUILD_SHARED OFF CACHE BOOL " ")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/OpenMesh ext_build/OpenMesh)
if(TARGET OpenMeshCoreStatic)
    set(LIBS ${LIBS} OpenMeshCoreStatic)
	set_property(TARGET OpenMeshCoreStatic OpenMeshToolsStatic PROPERTY FOLDER "dependencies")
	set_property(TARGET OpenMeshCore OpenMeshToolsStatic PROPERTY EXCLUDE_FROM_ALL TRUE)
else()
	set(LIBS ${LIBS} OpenMeshCore)
endif()
include_directories(ext/OpenMesh/src)
set_property(TARGET OpenMeshCore OpenMeshTools PROPERTY FOLDER "dependencies")
set_property(TARGET OpenMeshTools PROPERTY EXCLUDE_FROM_ALL TRUE)
add_definitions(/DOM_STATIC_BUILD)

add_subdirectory(common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common/include)

add_subdirectory(exercise1_2)
add_subdirectory(exercise3)
add_subdirectory(exercise4)
add_subdirectory(exercise5)

FinishBuildEnvironment()
