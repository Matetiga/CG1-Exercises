// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Viewer.h"

#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/checkbox.h>

#include <gui/SliderHelper.h>

#include <iostream>

#include <stb_image.h>

#include "glsl.h"
#include "textures.h"

const uint32_t PATCH_SIZE = 256; //number of vertices along one side of the terrain patch

Viewer::Viewer()
	: AbstractViewer("CG1 Exercise 4"),
	terrainPositions(nse::gui::VertexBuffer), terrainIndices(nse::gui::IndexBuffer),
	offsetBuffer(nse::gui::VertexBuffer)
{
	LoadShaders();
	CreateGeometry();
	
	//Create a texture and framebuffer for the background
	glGenFramebuffers(1, &backgroundFBO);
	glGenTextures(1, &backgroundTexture);
	ensureFBO();

	//Align camera to view a reasonable part of the terrain
	camera().SetSceneExtent(nse::math::BoundingBox<float, 3>(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(PATCH_SIZE - 1, 0, PATCH_SIZE - 1)));
	camera().FocusOnPoint(0.5f * Eigen::Vector3f(PATCH_SIZE - 1, 15, PATCH_SIZE - 1));	
	camera().Zoom(-30);
	camera().RotateAroundFocusPointLocal(Eigen::AngleAxisf(-0.5f, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(-0.05f, Eigen::Vector3f::UnitX()));
	camera().FixClippingPlanes(0.1f, 1000.f);
}

bool Viewer::resizeEvent(const Eigen::Vector2i&)
{
	//Re-generate the texture and FBO for the background
	ensureFBO();
	return false;
}

void Viewer::LoadShaders()
{
	skyShader.init("Sky Shader", std::string((const char*)sky_vert, sky_vert_size), std::string((const char*)sky_frag, sky_frag_size));
	terrainShader.init("Terrain Shader", std::string((const char*)terrain_vert, terrain_vert_size), std::string((const char*)terrain_frag, terrain_frag_size));
}

GLuint CreateTexture(const unsigned char* fileData, size_t fileLength, bool repeat = true)
{
	GLuint textureName;
	int textureWidth, textureHeight, textureChannels;
	auto pixelData = stbi_load_from_memory(fileData, (int)fileLength, &textureWidth, &textureHeight, &textureChannels, 3);
	textureName = 0;
	stbi_image_free(pixelData);
	return textureName;
}

void Viewer::CreateGeometry()
{
	//empty VAO for sky
	emptyVAO.generate();

	//terrain VAO	
	terrainVAO.generate();
	terrainVAO.bind();
	
	std::vector<Eigen::Vector4f> positions;
	std::vector<uint32_t> indices;
	
	/*Generate positions and indices for a terrain patch with a
	  single triangle strip */

	terrainShader.bind();
	terrainPositions.uploadData(positions).bindToAttribute("position");
	terrainIndices.uploadData((uint32_t)indices.size() * sizeof(uint32_t), indices.data());

	

	//textures
	grassTexture = CreateTexture((unsigned char*)grass_jpg, grass_jpg_size);
	rockTexture = CreateTexture((unsigned char*)rock_jpg, rock_jpg_size);
	roadColorTexture = CreateTexture((unsigned char*)roadcolor_jpg, roadcolor_jpg_size);
	roadNormalMap = CreateTexture((unsigned char*)roadnormals_jpg, roadnormals_jpg_size);
	roadSpecularMap = CreateTexture((unsigned char*)roadspecular_jpg, roadspecular_jpg_size);
	alphaMap = CreateTexture((unsigned char*)alpha_jpg, alpha_jpg_size, false);
}

void Viewer::ensureFBO()
{
	//Re-generate the texture and FBO for the background
	glBindFramebuffer(GL_FRAMEBUFFER, backgroundFBO);
	glBindTexture(GL_TEXTURE_2D, backgroundTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width(), height(), 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, backgroundTexture, 0);
	auto fboStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (fboStatus != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "Warning: Background framebuffer is not complete: " << fboStatus << std::endl;
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Viewer::RenderSky()
{
	Eigen::Matrix4f skyView = view;
	for (int i = 0; i < 3; ++i)
		skyView.col(i).normalize();
	skyView.col(3).head<3>().setZero();
	Eigen::Matrix4f skyMvp = proj * skyView;
	glDepthMask(GL_FALSE);
	glEnable(GL_DEPTH_CLAMP);
	emptyVAO.bind();
	skyShader.bind();
	skyShader.setUniform("mvp", skyMvp);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 6);
	glDisable(GL_DEPTH_CLAMP);
	glDepthMask(GL_TRUE);

	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, backgroundFBO);
	glBlitFramebuffer(0, 0, width(), height(), 0, 0, width(), height(), GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}

void CalculateViewFrustum(const Eigen::Matrix4f& mvp, Eigen::Vector4f* frustumPlanes, nse::math::BoundingBox<float, 3>& bbox)
{
	frustumPlanes[0] = (mvp.row(3) + mvp.row(0)).transpose();
	frustumPlanes[1] = (mvp.row(3) - mvp.row(0)).transpose();
	frustumPlanes[2] = (mvp.row(3) + mvp.row(1)).transpose();
	frustumPlanes[3] = (mvp.row(3) - mvp.row(1)).transpose();
	frustumPlanes[4] = (mvp.row(3) + mvp.row(2)).transpose();
	frustumPlanes[5] = (mvp.row(3) - mvp.row(2)).transpose();

	Eigen::Matrix4f invMvp = mvp.inverse();
	bbox.reset();
	for(int x = -1; x <= 1; x += 2)
		for(int y = -1; y <= 1; y += 2)
			for (int z = -1; z <= 1; z += 2)
	{
		Eigen::Vector4f corner = invMvp * Eigen::Vector4f((float)x, (float)y, (float)z, 1);
		corner /= corner.w();
		bbox.expand(corner.head<3>());
	}
}

bool IsBoxCompletelyBehindPlane(const Eigen::Vector3f& boxMin, const Eigen::Vector3f& boxMax, const Eigen::Vector4f& plane)
{
	return
		plane.dot(Eigen::Vector4f(boxMin.x(), boxMin.y(), boxMin.z(), 1)) < 0 &&
		plane.dot(Eigen::Vector4f(boxMin.x(), boxMin.y(), boxMax.z(), 1)) < 0 &&
		plane.dot(Eigen::Vector4f(boxMin.x(), boxMax.y(), boxMin.z(), 1)) < 0 &&
		plane.dot(Eigen::Vector4f(boxMin.x(), boxMax.y(), boxMax.z(), 1)) < 0 &&
		plane.dot(Eigen::Vector4f(boxMax.x(), boxMin.y(), boxMin.z(), 1)) < 0 &&
		plane.dot(Eigen::Vector4f(boxMax.x(), boxMin.y(), boxMax.z(), 1)) < 0 &&
		plane.dot(Eigen::Vector4f(boxMax.x(), boxMax.y(), boxMin.z(), 1)) < 0 &&
		plane.dot(Eigen::Vector4f(boxMax.x(), boxMax.y(), boxMin.z(), 1)) < 0;
}

void Viewer::drawContents()
{
	camera().ComputeCameraMatrices(view, proj);

	Eigen::Matrix4f mvp = proj * view;
	Eigen::Vector3f cameraPosition = view.inverse().col(3).head<3>();
	int visiblePatches = 0;

	RenderSky();
	
	//render terrain
	glEnable(GL_DEPTH_TEST);
	terrainVAO.bind();
	terrainShader.bind();	
	
	terrainShader.setUniform("screenSize", Eigen::Vector2f(width(), height()), false);
	terrainShader.setUniform("mvp", mvp);
	terrainShader.setUniform("cameraPos", cameraPosition, false);
	/* Task: Render the terrain */

	

	//Render text
	nvgBeginFrame(mNVGContext, (float)width(), (float)height(), mPixelRatio);
	std::string text = "Patches visible: " + std::to_string(visiblePatches);
	nvgText(mNVGContext, 10, 20, text.c_str(), nullptr);
	nvgEndFrame(mNVGContext);
}
