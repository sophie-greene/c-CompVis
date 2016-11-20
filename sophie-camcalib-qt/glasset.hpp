/**
* @brief OpenGL Specfic Primitives and such
* @file shapes.hpp
* @date 18/07/2012
*
*/

#ifndef GL_ASSET_HPP
#define GL_ASSET_HPP
#include "com/common.hpp"
#include "common.hpp"
#include "utils.hpp"
#include "primitive.hpp"
#include "asset.hpp"


namespace s9 {

	namespace gl {

		/*
		 * This is a mixture of basic geometry and a primtive for OpenGL
		 * We inherit the asset and ViaVAO classes
		 * \todo provide convinience methods to link directly through to loading models
		 * \todo do we even need to mix asset with primitive? - Possibly as Primitive provides hierarchy!
		 * \todo what about if we pass an asset? What do we do about potential sub assets? - COPY CONSTRUCTOR! :D
		 */

		template <class T>
		class GLAsset : public Asset<T>, public ViaVAO {

		protected:
			virtual void _gen() {}

			/*
			 * Allocate the GLBuffer in the first instance or resize
			 */
			virtual void _allocate() {
				// Allocation type depends whether we are buffering
				GLint method = GL_STATIC_DRAW;
				if(getGeometry().size() != getGeometry().numElements() || 
					(getGeometry().indexsize() > 0 && getGeometry().indexsize() != getGeometry().numElements() ))
				method = GL_STATIC_DRAW;

				glBindBuffer(GL_ARRAY_BUFFER, handle[0]);
				glBufferData(GL_ARRAY_BUFFER, getGeometry().size() * getGeometry().elementsize(), getGeometry().addr(), method);

				glBindBuffer(GL_ARRAY_BUFFER, 0);

				if (getGeometry().indexsize() > 0){
					glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
					glBufferData(GL_ELEMENT_ARRAY_BUFFER, getGeometry().indexsize() * sizeof(uint32_t), getGeometry().indexaddr(), method);
					glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
				}	

				getGeometry().setResized(false);
			}

			/*
			 * Update part of the buffer - for now update the whole thing
			 * \todo this is inefficent as we are replacing the entire buffer
			 * if one value changes. We need a better method
			 */
			virtual void _update() {
				glBindBuffer(GL_ARRAY_BUFFER, handle[0]);
				glBufferSubData(GL_ARRAY_BUFFER, 0, getGeometry().size() * getGeometry().elementsize(), getGeometry().addr());
				getGeometry().setDirty(false);
				glBindBuffer(GL_ARRAY_BUFFER, 0);
			}
		
		public:
			GLAsset() { this->mVAO = 0; };
			GLAsset(T a) : Asset<T>(a) {  this->mVAO = 0;};
			GLAsset(Asset<T> b) : Asset<T>(b) { this->mVAO = 0;}
			T getGeometry() { return this->mObj->mGeom; }

			// Override this 
			virtual void draw(GLint type=GL_TRIANGLES) {

				if(this->mVAO == 0) _gen();
				
				bind();

				if (getGeometry().isResized()) _allocate();
				if (getGeometry().isDirty()) _update();

				if ( getGeometry().indexsize() > 0){
					glDrawElements(type, getGeometry().numElements(), GL_UNSIGNED_INT, 0);
				}
				else{
					glDrawArrays(type,0, getGeometry().numElements());
				}

				unbind();
			 }
		};

	
		/*
		 * Creating a VAO around a basic Asset
		 */

		template<>
		inline void GLAsset<GeometryPNF>::_gen() {
			glGenVertexArrays(1, &(this->mVAO));
			int s = getGeometry().indexsize() > 0 ? 2 : 1;

			handle = new unsigned int[s];
			glGenBuffers(s,handle);

			_allocate();

			bind();

			glBindBuffer(GL_ARRAY_BUFFER, handle[0]);

			glEnableVertexAttribArray(0); // Pos
			glEnableVertexAttribArray(1); // Normal
		
			glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNF,mP) );
			glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNF,mN) );

			// Indices
			if (getGeometry().indexsize() > 0){
				glEnableVertexAttribArray(2); // Indices
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
				glVertexAttribPointer(2, 1,GL_UNSIGNED_INT,GL_FALSE,0, (GLubyte*) NULL);
			}

			unbind();

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);

		}

		/*
		 * Assets with 8 texture buffers!
		 */


		template<>
		inline void GLAsset<GeometryPNT8F>::_gen() {
			glGenVertexArrays(1, &(this->mVAO));
			int s = getGeometry().indexsize() > 0 ? 2 : 1;
			
			handle = new unsigned int[s];
			glGenBuffers(s,handle);

			_allocate();

			bind();

			glBindBuffer(GL_ARRAY_BUFFER, handle[0]);

			glEnableVertexAttribArray(0); // Pos
			glEnableVertexAttribArray(1); // Normal
		
			glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNT8F,mP) );
			glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNT8F,mN) );

			// Textures
			for (int i=0; i < 8; ++i){
				glEnableVertexAttribArray(i+2);
				glVertexAttribPointer(i+2,2, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNT8F,mT) + (sizeof(Float2) * i) );		
			}

			// Indices
			if (getGeometry().indexsize() > 0){
				glEnableVertexAttribArray(10); // Indices
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
				glVertexAttribPointer(10, 1,GL_UNSIGNED_INT,GL_FALSE,0, (GLubyte*) NULL);
			}

			unbind();

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);

		}

		/*
		 * Full Geometry
		 */


		template<>
		inline void GLAsset<GeometryFullFloat>::_gen() {
			glGenVertexArrays(1, &(this->mVAO));
			int s = getGeometry().indexsize() > 0 ? 2 : 1;
			
			handle = new unsigned int[s];
			glGenBuffers(s,handle);

			_allocate();

			bind();

			glBindBuffer(GL_ARRAY_BUFFER, handle[0]);

			glEnableVertexAttribArray(0); // Pos
			glEnableVertexAttribArray(1); // Normal
			glEnableVertexAttribArray(2); // Colour
			glEnableVertexAttribArray(3); // Texture
		
			glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNCTF,mP) );
			glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNCTF,mN) );
			glVertexAttribPointer(2,4, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNCTF,mC) );
			glVertexAttribPointer(3,2, GL_FLOAT, GL_FALSE, getGeometry().elementsize(), (GLvoid*)offsetof(VertPNCTF,mT) );

			// Indices
			if (getGeometry().indexsize() > 0){
				glEnableVertexAttribArray(4); // Indices
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
				glVertexAttribPointer(4, 1,GL_UNSIGNED_INT,GL_FALSE,0, (GLubyte*) NULL);
			}

			unbind();

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);


		}

		/*
		 * Just basic points
		 */


		template<>
		inline void GLAsset<GeometryPF>::_gen() {
			glGenVertexArrays(1, &(this->mVAO));
			int s = getGeometry().indexsize() > 0 ? 2 : 1;
			
			handle = new unsigned int[s];
			glGenBuffers(s,handle);
			_allocate();
			bind();

			glBindBuffer(GL_ARRAY_BUFFER, handle[0]);
			glEnableVertexAttribArray(0); // Pos
			glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, getGeometry().elementsize(),0 );
		
			// Indices
			if (getGeometry().indexsize() > 0){
				glEnableVertexAttribArray(1); // Indices
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
				glVertexAttribPointer(1, 1,GL_UNSIGNED_INT,GL_FALSE,0, (GLubyte*) NULL);
			}

			unbind();

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);

		}




	}
}

#endif
