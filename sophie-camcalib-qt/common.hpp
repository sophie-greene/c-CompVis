/**
* @brief Common Headers that are external for OpenGL
* @file gl/common.hpp
* @date 27/07/2012
*
*/


///\todo precompile!


#ifndef GL_COMMON_HPP
#define GL_COMMON_HPP

#include <GL/glew.h>

namespace s9 {
	namespace gl {
		
		/*
		 * Small Interface class to wrap the VAO state for an object
		 */

		class ViaVAO {
		public:
			void bind() { glBindVertexArray(mVAO); };
			void unbind()  { glBindVertexArray(0); };
			GLuint mVAO;
			unsigned int *handle;

		};
	}
}


#endif
