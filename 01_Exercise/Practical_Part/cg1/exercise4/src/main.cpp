// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include <iostream>

#include <util/GLDebug.h>

#include "Viewer.h"

int main(int argc, char* argv[])
{
	nanogui::init();

	{
		nanogui::ref<Viewer> viewer = new Viewer();
		viewer->setVisible(true);

		nse::util::GLDebug::SetupDebugCallback();
		nse::util::GLDebug::IgnoreGLError(131185); //buffer usage info

		try
		{
			nanogui::mainloop();
		}
		catch (std::runtime_error& e)
		{
			std::cerr << e.what() << std::endl;
		}

	}

	nanogui::shutdown();

	return 0;
}
