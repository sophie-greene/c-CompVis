/**
* @brief Shader Classes
* @file shader.hpp
* @date 05/07/2012
*
*/


#ifndef GL_SHADER_HPP
#define GL_SHADER_HPP

#include "com/common.hpp"
#include "common.hpp"
#include "utils.hpp"

/*
 * Basic Shader class - loads and binds
 * \todo fluent and shorthand interface
 * \todo geometry shader
 * \todo compiled in shaders within the code
 */

namespace s9 {

	namespace gl {

		class Shader {
		public:
			void load(std::string vert, std::string frag);
			GLuint getProgram() { return mProgram; };
			
			GLint location(const char * name) {return glGetUniformLocation(mProgram, name); }
			
			// Fluent interface for quick setting

			Shader& s(const char * name, glm::vec3 v);
			Shader& s(const char * name, glm::vec4 v);
			Shader& s(const char * name, glm::mat4 v);
			Shader& s(const char * name, float_t f);
			Shader& s(const char * name, int i);

			void bind() { glUseProgram(mProgram);};
			void unbind() {glUseProgram(0);};
			
			~Shader() { glDetachShader(mProgram, mVS); glDetachShader(mProgram, mFS);  } 
			
		protected:
		   
			GLuint mVS, mFS;
			GLuint mProgram;

		};
	}
}


#endif
