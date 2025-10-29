// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <gui/AbstractViewer.h>
#include <gui/GLShader.h>
#include <gui/GLBuffer.h>
#include <gui/GLVertexArray.h>

class Viewer : public nse::gui::AbstractViewer
{
public:
	Viewer();

	void LoadShaders();
	void CreateGeometry();

	void drawContents();	
	bool resizeEvent(const Eigen::Vector2i&);


private:
	void ensureFBO();
	void RenderSky();

	Eigen::Matrix4f view, proj;

	nse::gui::GLShader skyShader;
	nse::gui::GLVertexArray emptyVAO;

	nse::gui::GLShader terrainShader;
	nse::gui::GLVertexArray terrainVAO;
	nse::gui::GLBuffer terrainPositions;
	nse::gui::GLBuffer terrainIndices;

	GLuint grassTexture, rockTexture, roadColorTexture, roadNormalMap, roadSpecularMap, alphaMap;
	GLuint backgroundFBO, backgroundTexture;

	nse::gui::GLBuffer offsetBuffer;
};
