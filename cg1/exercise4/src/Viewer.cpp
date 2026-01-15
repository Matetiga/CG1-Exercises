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

const GLuint RESTART_IDX = 314159;
const uint32_t PATCH_SIZE = 256; //number of vertices along one side of the terrain patch

Viewer::Viewer()
	: AbstractViewer("CG1 Exercise 4"),
	terrainPositions(nse::gui::VertexBuffer), terrainIndices(nse::gui::IndexBuffer),
	offsetBuffer(nse::gui::VertexBuffer), waterPositionsBuf(nse::gui::VertexBuffer), waterIndicesBuf(nse::gui::IndexBuffer)
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
	waterShader.init("Water Shader", std::string((const char*)water_vert, water_vert_size), std::string((const char*)water_frag, water_frag_size));
}

GLuint CreateTexture(const unsigned char* fileData, size_t fileLength,Viewer::textureIndex texIdx ,  bool repeat = true)
{
	GLuint textureName;
	int textureWidth, textureHeight, textureChannels;
	auto pixelData = stbi_load_from_memory(fileData, (int)fileLength, &textureWidth, &textureHeight, &textureChannels, 3);
	textureName = 0;

	// glActivateTexture is not necessary
	glGenTextures(1, &textureName);
	glBindTexture(GL_TEXTURE_2D, textureName);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 
		textureWidth, textureHeight, 
		0, GL_RGB, 
		GL_UNSIGNED_BYTE, 
		pixelData);


	// repeat = false for the alpha map
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, repeat ? GL_REPEAT : GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, repeat ? GL_REPEAT : GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);

	glGenerateMipmap(GL_TEXTURE_2D);
	
	stbi_image_free(pixelData);
	return textureName;
}

void CreateTerrain(std::vector<Eigen::Vector4f>& positions, std::vector<uint32_t>& indices, float height)
{
	for(int z = 0; z <= PATCH_SIZE ; z++){
		for(int x = 0; x <= PATCH_SIZE ; x++){
			positions.push_back(Eigen::Vector4f((float)x, height, (float)z, 1.0f));
		}
	}
	
	// Anti Clockwise Order
	for(int z = 0; z < PATCH_SIZE; z++){
		for(int x = 0; x <= PATCH_SIZE; x++){
			indices.push_back((z + 1) * (PATCH_SIZE + 1) + x);
			indices.push_back(z * (PATCH_SIZE + 1) + x);
		}
	
		//Restart the strip/
		indices.push_back(RESTART_IDX);
	}
}

void Viewer::CreateGeometry()
{
	glPrimitiveRestartIndex(RESTART_IDX);
	//empty VAO for sky
	emptyVAO.generate();

	
	// Information for terrain
	std::vector<Eigen::Vector4f> positions;
	std::vector<uint32_t> indices;
	CreateTerrain(positions, indices, 0.0f);

	// Information for water ---> only the height changes, so maybe reuse position from terrain and change height in shader
	//std::vector<Eigen::Vector4f> waterPositions;
	//std::vector<uint32_t> waterIndices;
	//CreateTerrain(waterPositions, waterIndices, 1.5f);

	//terrain VAO	
	terrainVAO.generate();
	terrainVAO.bind();
	terrainShader.bind();
	// no uniform, but just "in" inside the shader? YES, because its a buffer that has Info pro vertex
	terrainPositions.uploadData(positions).bindToAttribute("position"); 
	terrainIndices.uploadData((uint32_t)indices.size() * sizeof(uint32_t), indices.data());
	// bind the offset Information to the buffer to the shader
	// IMPORTANT : These two following lines cause a 501 Error , why?
	//offsetBuffer.bind();
	//offsetBuffer.bindToAttribute("offset");
	// this will activate the pointer in offset buffer to move only after a complete instance has been drawn
	glVertexAttribDivisor(terrainShader.attrib("offset"), 1);  

	// water VAO ---> the same process has to be done for water
	waterVAO.generate();
	waterVAO.bind();
	waterShader.bind();

	waterPositionsBuf.uploadData(positions).bindToAttribute("waterPosition");
	waterIndicesBuf.uploadData((uint32_t)positions.size() * sizeof(uint32_t), indices.data());
	//offsetBuffer.bindToAttribute("offset");
	glVertexAttribDivisor(terrainShader.attrib("offset"), 1);  


	//textures
	grassTexture = CreateTexture((unsigned char*)grass_jpg, grass_jpg_size, Viewer::GRASS_TEXTURE);
	rockTexture = CreateTexture((unsigned char*)rock_jpg, rock_jpg_size, Viewer::ROCK_TEXTURE);
	roadColorTexture = CreateTexture((unsigned char*)roadcolor_jpg, roadcolor_jpg_size, Viewer::ROAD_COLOR_TEXTURE);
	roadNormalMap = CreateTexture((unsigned char*)roadnormals_jpg, roadnormals_jpg_size, Viewer::ROAD_NORMAL_MAP);
	roadSpecularMap = CreateTexture((unsigned char*)roadspecular_jpg, roadspecular_jpg_size, Viewer::ROAD_SPECULAR_MAP);
	alphaMap = CreateTexture((unsigned char*)alpha_jpg, alpha_jpg_size,Viewer::ALPHA_MAP);
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

// wo wird drawContents aufgerufen? (nicht in main.cpp)
void Viewer::drawContents()
{
	camera().ComputeCameraMatrices(view, proj);

	Eigen::Matrix4f mvp = proj * view;
	Eigen::Vector3f cameraPosition = view.inverse().col(3).head<3>();
	int visiblePatches = 0;

	RenderSky();


	// Calculate the view frustum planes and the scene bounding box
	Eigen::Vector4f frustumPlanes[6];
	nse::math::BoundingBox<float, 3> bbox;
	CalculateViewFrustum(mvp, frustumPlanes, bbox);

	// bind and pass the offset buffer to the shader
	std::vector<Eigen::Vector2f> offsets;
	int visibleChuncks = 10;
	float patch_pixel = (float)PATCH_SIZE;

	int camX = floor(cameraPosition.x() / patch_pixel);
	int camZ = floor(cameraPosition.z() / patch_pixel);
	for (int x = -visibleChuncks; x <= visibleChuncks; x++) {
		for (int z = -visibleChuncks; z <= visibleChuncks; z++) {

			// create the AABB box
			Eigen::Vector3f boxMin = Eigen::Vector3f((x + camX) * patch_pixel , 0.0f, (z + camZ) * patch_pixel) ;
			Eigen::Vector3f boxMax = Eigen::Vector3f((x + 1 + camX) * patch_pixel,  15.0f, (z + 1 + camZ) * patch_pixel); // +1 to get to the other corner

			// Frustum Culling Test
			bool visible = true;
			for (int i = 0; i < 6; i++) {
				if (IsBoxCompletelyBehindPlane(boxMin, boxMax, frustumPlanes[i])) {
					visible = false;
					break;
				}
			}

			if (visible) {
				offsets.push_back(Eigen::Vector2f((x + camX) * patch_pixel, (z + camZ) * patch_pixel));
			}

		}
	}

	visiblePatches = offsets.size();
	// Render terrain
	glEnable(GL_DEPTH_TEST);
	terrainVAO.bind();
	terrainShader.bind();

	terrainShader.setUniform("screenSize", Eigen::Vector2f(width(), height()), false);
	terrainShader.setUniform("mvp", mvp);
	terrainShader.setUniform("cameraPos", cameraPosition, false);

	offsetBuffer.bind(); // really necessary?
	offsetBuffer.uploadData(offsets);
	offsetBuffer.bindToAttribute("offset"); // otherwise the other instances are not rendered, then why do the same in CreateGeometry?
	
	/* Task: Render the terrain */

	// Bind textures to shader
	// here is necessary to call glActiveTexture to select an alredy generated texture
	// Background texture for the grass
	glActiveTexture(GL_TEXTURE0 + Viewer::GRASS_TEXTURE);
	glBindTexture(GL_TEXTURE_2D, grassTexture);
	// this 0 refers to texture Channel in the shader 
	terrainShader.setUniform("grassTexture", 0); // provided method only works for integers and not for GLuint

	// texture for the road ontop
	glActiveTexture(GL_TEXTURE0 + Viewer::ROCK_TEXTURE);
	glBindTexture(GL_TEXTURE_2D, rockTexture);
	terrainShader.setUniform("rockTexture",1);

	// road texture
	glActiveTexture(GL_TEXTURE0 + Viewer::ROAD_COLOR_TEXTURE);
	glBindTexture(GL_TEXTURE_2D, roadColorTexture);
	terrainShader.setUniform("roadColorTexture",2);

	// road specular
	glActiveTexture(GL_TEXTURE0 + Viewer::ROAD_SPECULAR_MAP);
	glBindTexture(GL_TEXTURE_2D, roadSpecularMap);
	terrainShader.setUniform("roadSpecularMap", 3);

	// road normal map
	glActiveTexture(GL_TEXTURE0 + Viewer::ROAD_NORMAL_MAP);
	glBindTexture(GL_TEXTURE_2D, roadNormalMap);
	terrainShader.setUniform("roadNormalMap", 4);

	// road alpha
	glActiveTexture(GL_TEXTURE0 + Viewer::ALPHA_MAP);
	glBindTexture(GL_TEXTURE_2D, alphaMap);
	terrainShader.setUniform("alphaMap", 5);

	// background texture for fog
	// the 8 comes because the texture was generated in Viewer()
	glActiveTexture(GL_TEXTURE0 + 8);
	glBindTexture(GL_TEXTURE_2D, backgroundTexture);
	terrainShader.setUniform("backgroundTexture", 8);

	// Difference between glDrawArrays and glDrawElements
	// glDrawArrays reads vertices sequentially 
	// glDrawElements reads vertices according to the indices provided in the index buffer
	// using the RESTART Index to create strips
	glEnable(GL_PRIMITIVE_RESTART);
	glDrawElementsInstanced(GL_TRIANGLE_STRIP, terrainIndices.bufferSize(), GL_UNSIGNED_INT, 0, offsets.size()); // offset.size marks the number of instances


	// TODO : patches of water behind user will be generated, the previous methdod has to be reused
	// Render water surface
	waterVAO.bind();
	waterShader.bind();
	waterShader.setUniform("mvp", mvp);

	// we have to rebind the buffer, because this is a different VAO
	offsetBuffer.bind();
	offsetBuffer.bindToAttribute("offset");
	glDrawElementsInstanced(GL_TRIANGLE_STRIP, waterIndicesBuf.bufferSize(), GL_UNSIGNED_INT, 0, offsets.size() ); 

	

	//Render text
	nvgBeginFrame(mNVGContext, (float)width(), (float)height(), mPixelRatio);
	std::string text = "Patches visible: " + std::to_string(visiblePatches);
	nvgText(mNVGContext, 10, 20, text.c_str(), nullptr);
	nvgEndFrame(mNVGContext);
}
