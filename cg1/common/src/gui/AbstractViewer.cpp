/*
	This file is part of NSEssentials.

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "gui/AbstractViewer.h"

#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <cstring>
#include <iostream>

using namespace nse::gui;

AbstractViewer::AbstractViewer(const std::string& title, int width, int height, int nSamples)
	: nanogui::Screen(Eigen::Vector2i(width, height), title, true, false, 8, 8, 24, 8, nSamples),
	_camera(*this), _ctrlDown(false), _shiftDown(false), nSamples(nSamples)
{	
}

nanogui::Window* AbstractViewer::SetupMainWindow()
{
	auto mainWindow = new nanogui::Window(this, this->caption());
	mainWindow->setPosition(Eigen::Vector2i(15, 15));
	mainWindow->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 4, 4));

	CheckOpenGLSupport();

	return mainWindow;
}

// Check if the correct version of OpenGL is supported
void AbstractViewer::CheckOpenGLSupport()
{
	char tempBuffer[255];
	char* oglVersion = (char*)glGetString(GL_VERSION);
	strcpy(tempBuffer, oglVersion);
	char* token = strtok(tempBuffer, ". ");
	std::vector<int> oglVersionNumbers;
	while (token != nullptr)
	{
		try 
		{
			oglVersionNumbers.push_back(std::atoi(token));
			token = strtok(nullptr, ". ");
		}
		catch (...)
		{
			std::cout << "Error parsing OpenGL version string " << oglVersion << std::endl;
			return;
		}
	}

	char* glslVersion = (char*)glGetString(GL_SHADING_LANGUAGE_VERSION);
	strcpy(tempBuffer, oglVersion);
	token = strtok(tempBuffer, ". ");
	std::vector<int> glslVersionNumbers;
	while (token != nullptr)
	{
		try
		{
			glslVersionNumbers.push_back(std::atoi(token));
			token = strtok(nullptr, ". ");
		}
		catch (...)
		{
			std::cout << "Error parsing GLSL version string " << glslVersion << std::endl;
			return;
		}
	}

	if (oglVersionNumbers.size() < 1 || glslVersionNumbers.size() < 2)
	{
		std::cout << "Error parsing OpenGL or GLSL version numbers." << std::endl;
		return;
	}

	if (oglVersionNumbers[0] < 3 || (glslVersionNumbers[1] < 3 && glslVersionNumbers[0] == 1)) 
	{
		std::cout
			<< "ERROR: YOUR VERSION OF OPENGL IS TOO OLD!" << std::endl
			<< "  - Needed Opengl-Version: 3.0. On this system: " << glGetString(GL_VERSION) << std::endl
			<< "  - Needed GLSL-Version: 1.3. On this system: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
	}
}



bool AbstractViewer::keyboardEvent(int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_LEFT_CONTROL && action == 0)
		_ctrlDown = false;
	if (key == GLFW_KEY_LEFT_CONTROL && action == 1)
		_ctrlDown = true;

	if (key == GLFW_KEY_LEFT_SHIFT && action == 0)
		_shiftDown = false;
	if (key == GLFW_KEY_LEFT_SHIFT && action == 1)
		_shiftDown = true;

	return true;
}

bool AbstractViewer::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel)
{
	if (Screen::scrollEvent(p, rel))
		return true;

	if (scrollHook(p, rel))
		return true;

	if (!_ctrlDown && !_shiftDown)
	{
		_camera.Zoom(rel.y());
		return true;
	}

	return false;
}

bool AbstractViewer::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers)
{
	if (Screen::mouseButtonEvent(p, button, down, modifiers) && down)
		return true;

	if (mouseButtonHook(p, button, down, modifiers) && down)
		return true;

	auto now = std::chrono::high_resolution_clock::now();
	if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastClickTime).count() < 300 && !down && button == lastClickButton && (p - lastClickPosition).cwiseAbs().sum() <= 2)
	{
		if (button == GLFW_MOUSE_BUTTON_LEFT)
		{
			Eigen::Vector3f point;
			auto depth = get3DPosition(p, point);
			if (depth < 1)
			{
				_camera.FocusOnPoint(point);
				glfwSetCursorPos(mGLFWWindow, width() / 2, height() / 2);
			}
		}
	}

	if (!down)
	{
		lastClickTime = now;
		lastClickButton = button;
		lastClickPosition = p;
	}

	return _camera.HandleMouseButton(p, button, down, modifiers);	
}

bool AbstractViewer::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel,
	int button, int modifiers)
{
	if (Screen::mouseMotionEvent(p, rel, button, modifiers))
		return true;

	if (mouseMotionHook(p, rel, button, modifiers))
		return true;

	return _camera.HandleMouseMove(p, rel, button, modifiers);
}

bool AbstractViewer::resizeEvent(const Eigen::Vector2i & s)
{
	Screen::resizeEvent(s);
	_camera.resize(s);
	return true;
}

float AbstractViewer::get3DPosition(const Eigen::Vector2i & screenPos, Eigen::Vector4f & pos)
{
	float depth;
	glReadPixels(screenPos.x(), height() - 1 - screenPos.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

	float ndcDepth = 2 * (depth - 0.5f);

	float x = 2 * ((float)screenPos.x() / width() - 0.5f);
	float y = 2 * ((float)-screenPos.y() / height() + 0.5f);

	Eigen::Matrix4f view, proj;
	camera().ComputeCameraMatrices(view, proj);

	Eigen::Matrix4f mvp = proj * view;
	Eigen::Matrix4f invMvp = mvp.inverse();

	pos = invMvp * Eigen::Vector4f(x, y, ndcDepth, 1);
	pos /= pos.w();

	return depth;
}

float AbstractViewer::get3DPosition(const Eigen::Vector2i & screenPos, Eigen::Vector3f & pos)
{
	Eigen::Vector4f pos4;
	float depth = get3DPosition(screenPos, pos4);
	pos.x() = pos4.x();
	pos.y() = pos4.y();
	pos.z() = pos4.z();
	return depth;
}