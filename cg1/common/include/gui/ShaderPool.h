// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <gui/GLShader.h>

struct ShaderPool
{
private:
	static ShaderPool* _instance;
public:
	static ShaderPool* Instance();

	void CompileShaders();

	nse::gui::GLShader meshShader;
	nse::gui::GLShader simpleShader;
};