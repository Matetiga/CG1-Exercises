// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <gui/AbstractViewer.h>
#include <util/OpenMeshUtils.h>

class Viewer : public nse::gui::AbstractViewer
{
public:
	Viewer();

	void drawContents();

private:
	void SetupGUI();

	Eigen::Matrix4f modelViewMatrix, projectionMatrix;

	//GUI Elements for the various options
	nanogui::CheckBox* chkHasFaceCulling;	//Shall back face culling be activated?
	nanogui::CheckBox* chkHasDepthTesting;	//Shall depth testing be activated?

	nanogui::Slider* sldJuliaCX;	//Seed for the Julia fractal
	nanogui::Slider* sldJuliaCY;
	nanogui::Slider* sldJuliaZoom;	//Zoom factor for the Julia fractal

	// The following variables hold OpenGL object IDs
	GLuint vertex_shader_id,	// ID of the vertex shader
		fragment_shader_id,	// ID of the fragment shader
		program_id,			// ID of the shader program
		vertex_array_id,		// ID of the vertex array
		position_buffer_id,	// ID of the position buffer
		color_buffer_id,		// ID of the color buffer
		uv_map_buffer_id;	// ID of the uv_map

							// Read, Compile and link the shader codes to a shader program
	void CreateShaders();
	// Create and define the vertex array and add a number of vertex buffers
	void CreateVertexBuffers();
};
