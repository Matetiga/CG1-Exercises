#pragma once

#include <nanogui/opengl.h>
#include <unordered_set>

namespace nse
{
	namespace util
	{
		class GLDebug
		{
		public:
			static void IgnoreGLError(GLuint errorId);
			static void SetupDebugCallback();
			
		private:
			static std::unordered_set<GLuint> ignoredIds;
		};		
	}
}