/**
* @brief gl shapes classes	
* @file shapes.cpp
* @date 06/07/2012
*
*/

#include "shapes.hpp"

using namespace s9::gl;
using namespace std;

/*
 * Allocate function for updates to the Quad size
 */

void Quad::_allocate() {

	glBindBuffer(GL_ARRAY_BUFFER, handle[0]);
	glBufferData(GL_ARRAY_BUFFER, mGeom.size() * sizeof(VertPNCTF), mGeom.addr(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mGeom.indexsize() * sizeof(uint32_t), mGeom.indexaddr(), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
}


/*
 * Generate the VAO for the OpenGL Quad
 */

void Quad::_gen() {

	glGenVertexArrays(1,&mVAO);
	
	handle = new unsigned int[2];
	glGenBuffers(2,handle);

	_allocate();

	bind();

	glBindBuffer(GL_ARRAY_BUFFER, handle[0]);

	glEnableVertexAttribArray(0); // Pos
	glEnableVertexAttribArray(1); // Normal
	glEnableVertexAttribArray(2); // Colour
	glEnableVertexAttribArray(3); // TexCoord0
	glEnableVertexAttribArray(4); // Indices


	glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mP) );
	glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mN) );
	glVertexAttribPointer(2,4, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mC) );
	glVertexAttribPointer(3,2, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mT) );

	// Indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
	glVertexAttribPointer(4, 1,GL_UNSIGNED_INT,GL_FALSE,0, (GLubyte*) NULL);

	unbind();

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);

	CXGLERROR
}


void Quad::draw() {
	
	if(mVAO == 0) _gen();

	bind();
	if (mGeom.isDirty()) _allocate();

	glDrawElements(GL_TRIANGLES, mGeom.indexsize(), GL_UNSIGNED_INT, 0);

	unbind();

	CXGLERROR
}


/*
 * Allocate the actual data
 */

void Triangle::_allocate() {

	glBindBuffer(GL_ARRAY_BUFFER, handle[0]);
	glBufferData(GL_ARRAY_BUFFER, mGeom.size() * sizeof(VertPNCTF), mGeom.addr(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

/*
 * Generate the Triangle
 */

void Triangle::_gen() {
	glGenVertexArrays(1,&mVAO);
	

	handle = new unsigned int[1];
	glGenBuffers(1,handle);

	_allocate();

	bind();

	glBindBuffer(GL_ARRAY_BUFFER, handle[0]);

	glEnableVertexAttribArray(0); // Pos
	glEnableVertexAttribArray(1); // Normal
	glEnableVertexAttribArray(2); // Colour
	glEnableVertexAttribArray(3); // TexCoord0


	glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mP) );
	glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mN) );
	glVertexAttribPointer(2,4, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mC) );
	glVertexAttribPointer(3,2, GL_FLOAT, GL_FALSE, sizeof(VertPNCTF), (GLvoid*)offsetof(VertPNCTF,mT) );


	unbind();

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
}

void Triangle::draw() {
	
	if(mVAO == 0) _gen();

	bind();
	if (mGeom.isDirty()) _allocate();

	glDrawArrays(GL_TRIANGLES, 0, mGeom.size());

	unbind();
}
