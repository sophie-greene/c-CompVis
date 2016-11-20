/**
* @brief Drawer class that deals with drawing EVERYTHING
* @file drawer.cpp
* @date 28/05/2012
*
*/

#include "drawer.hpp"

using namespace std;
using namespace boost;
using namespace cv;
using namespace boost::assign; 


///\todo shift the scale factor and borders to globals or settings in mConfig


std::string textFileRead(std::string filename) {
	string line;
	string rval;
	ifstream myfile (filename.c_str());
	if (myfile.is_open()){
		while ( myfile.good() )	{
			getline (myfile,line);
			rval += line +"\n";
		}
		myfile.close();
	}

	else cerr << "Leeds - Unable to open shader file " << filename << endl;

	return rval;
}



void Shader::load(std::string vert, std::string frag) {
	
	int isCompiled_VS, isCompiled_FS;
	
	int maxLength;
	int IsLinked;
	char *vertexInfoLog;
	char *fragmentInfoLog;
	char *shaderProgramInfoLog;
	
	mVS = glCreateShader(GL_VERTEX_SHADER);
	mFS = glCreateShader(GL_FRAGMENT_SHADER);	

	string sv = textFileRead(vert);
	string sf = textFileRead(frag);

	const char * vv = sv.c_str();
	const char * ff = sf.c_str();

	glShaderSource(mVS, 1, &vv,NULL);
	glShaderSource(mFS, 1, &ff,NULL);

	glCompileShader(mVS);
	
	glGetShaderiv(mVS, GL_COMPILE_STATUS, &isCompiled_VS);
	if(isCompiled_VS == false) {
		glGetShaderiv(mVS, GL_INFO_LOG_LENGTH, &maxLength);
 
		vertexInfoLog = new char[maxLength];
		glGetShaderInfoLog(mVS, maxLength, &maxLength, vertexInfoLog);
 
		cerr << "Leeds - Vertex Shader Error " << vertexInfoLog << endl;
 
		delete [] vertexInfoLog;
		return;
	}
	
	glCompileShader(mFS);
	glGetShaderiv(mFS, GL_COMPILE_STATUS, &isCompiled_FS);
	if(isCompiled_FS == false) {
		glGetShaderiv(mFS, GL_INFO_LOG_LENGTH, &maxLength);
		fragmentInfoLog = new char[maxLength];
		glGetShaderInfoLog(mFS, maxLength, &maxLength, fragmentInfoLog);
		cerr << "Leeds - Fragment Shader Error " << fragmentInfoLog << endl;
		
		delete [] fragmentInfoLog;
		return;
	}
	
	mProgram = glCreateProgram();

	glAttachShader(mProgram,mVS);
	glAttachShader(mProgram,mFS);
	glLinkProgram(mProgram);
	
	glGetProgramiv(mProgram, GL_LINK_STATUS, (int *)&IsLinked);
	if(IsLinked == false) {
		glGetProgramiv(mProgram, GL_INFO_LOG_LENGTH, &maxLength);
		shaderProgramInfoLog = new char[maxLength];
		glGetProgramInfoLog(mProgram, maxLength, &maxLength, shaderProgramInfoLog);
		cerr << "Leeds - Shader Program Error " << shaderProgramInfoLog << endl;
		free(shaderProgramInfoLog);
		return;
	}
}

/*
 * FBO
 */

void FBO::setup(size_t w, size_t h){
	mW = w;
	mH = h;
	
	glGenFramebuffers(1, &mID);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mID);
 
	// Create depth renderbuffer
	glGenRenderbuffers(1, &mDepth);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
 
	// Create the texture
	glGenTextures(1, &mColour);
	glBindTexture(GL_TEXTURE_2D, mColour);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
 
	// Attach texture to first color attachment and the depth to the depth attachment
	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mColour, 0);
	glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, mDepth);
 
	GLenum fboStatus = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);   
 
	if (fboStatus != GL_FRAMEBUFFER_COMPLETE)  {
		std::cout << "Couldn't create frame buffer: " << fboStatus << std::endl;
		checkStatus();
	}
	else
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}


bool FBO::checkStatus() {
 GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    switch(status)
    {
    case GL_FRAMEBUFFER_COMPLETE:
        std::cout << "Framebuffer complete." << std::endl;
        return true;

    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        std::cout << "[ERROR] Framebuffer incomplete: Attachment is NOT complete." << std::endl;
        return false;

    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        std::cout << "[ERROR] Framebuffer incomplete: No image is attached to FBO." << std::endl;
        return false;
/*
    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
        std::cout << "[ERROR] Framebuffer incomplete: Attached images have different dimensions." << std::endl;
        return false;

    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
        std::cout << "[ERROR] Framebuffer incomplete: Color attached images have different internal formats." << std::endl;
        return false;
*/
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        std::cout << "[ERROR] Framebuffer incomplete: Draw buffer." << std::endl;
        return false;

    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        std::cout << "[ERROR] Framebuffer incomplete: Read buffer." << std::endl;
        return false;

    case GL_FRAMEBUFFER_UNSUPPORTED:
        std::cout << "[ERROR] Framebuffer incomplete: Unsupported by FBO implementation." << std::endl;
        return false;

    default:
        std::cout << "[ERROR] Framebuffer incomplete: Unknown error." << std::endl;
        return false;
    }
}

void FBO::printFramebufferInfo() {
    std::cout << "\n===== FBO STATUS =====\n";

    // print max # of colorbuffers supported by FBO
    int colorBufferCount = 0;
    glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &colorBufferCount);
    std::cout << "Max Number of Color Buffer Attachment Points: " << colorBufferCount << std::endl;

    int objectType;
    int objectId;

    // print info of the colorbuffer attachable image
    for(int i = 0; i < colorBufferCount; ++i)
    {
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER,
                                              GL_COLOR_ATTACHMENT0+i,
                                              GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE,
                                              &objectType);
        if(objectType != GL_NONE)
        {
            glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER,
                                                  GL_COLOR_ATTACHMENT0+i,
                                                  GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME,
                                                  &objectId);

            std::string formatName;

            std::cout << "Color Attachment " << i << ": ";
            if(objectType == GL_TEXTURE)
            {
                std::cout << "GL_TEXTURE, " << getTextureParameters(objectId) << std::endl;
            }
            else if(objectType == GL_RENDERBUFFER)
            {
                std::cout << "GL_RENDERBUFFER, " << getRenderbufferParameters(objectId) << std::endl;
            }
        }
    }

    // print info of the depthbuffer attachable image
    glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER,
                                          GL_DEPTH_ATTACHMENT,
                                          GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE,
                                          &objectType);
    if(objectType != GL_NONE)
    {
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER,
                                              GL_DEPTH_ATTACHMENT,
                                              GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME,
                                              &objectId);

        std::cout << "Depth Attachment: ";
        switch(objectType)
        {
        case GL_TEXTURE:
            std::cout << "GL_TEXTURE, " << getTextureParameters(objectId) << std::endl;
            break;
        case GL_RENDERBUFFER:
            std::cout << "GL_RENDERBUFFER, " << getRenderbufferParameters(objectId) << std::endl;
            break;
        }
    }

    // print info of the stencilbuffer attachable image
    glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER,
                                          GL_STENCIL_ATTACHMENT,
                                          GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE,
                                          &objectType);
    if(objectType != GL_NONE)
    {
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER,
                                              GL_STENCIL_ATTACHMENT,
                                              GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME,
                                              &objectId);

        std::cout << "Stencil Attachment: ";
        switch(objectType)
        {
        case GL_TEXTURE:
            std::cout << "GL_TEXTURE, " << getTextureParameters(objectId) << std::endl;
            break;
        case GL_RENDERBUFFER:
            std::cout << "GL_RENDERBUFFER, " << getRenderbufferParameters(objectId) << std::endl;
            break;
        }
    }

    std::cout << std::endl;
}


void FBO::resize(size_t w, size_t h){
	mW = w;
	mH = h;
	
	glBindTexture(GL_TEXTURE_2D, mColour);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_FLOAT, NULL);
	
	glBindRenderbuffer(GL_RENDERBUFFER, mDepth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
	
}


/*
 * VBO Constructor
 */

VBOData::VBOData() {
   
}

/*
 * VBO Destructor
 */

VBOData::~VBOData() {
	glDeleteBuffers(mNumBufs, vbo);
    glDeleteVertexArrays(1, &vao);
}

/*
 * Bind this VBO/VAO and set the shader attribs
 * Using the layout directive in shaders to map attribs to numbers
 */

void VBOData::bind() {
	glBindVertexArray(vao);
}

void VBOData::unbind() {
	glBindVertexArray(0);
	
	for (int id = 0; id < mNumBufs; ++ id){
		glDisableVertexAttribArray(id);
	}
}


void VBOData::compile(size_t buffers) {
	// Find out how many buffers we need
	int s = 0;
	mUsed = buffers;
	
	if (mUsed & VBO_VERT) s++;
	if (mUsed & VBO_IDCE) s++;
	if (mUsed & VBO_COLR) s++;
	if (mUsed & VBO_NORM) s++;
	if (mUsed & VBO_TEXC) s++;
	if (mUsed & VBO_TEXI) s++;
	
	vbo = new GLuint[s];
	glGenBuffers(s,vbo);
	s = 0;
	if (mUsed & VBO_VERT) { mVID = vbo[s]; s++;}
	if (mUsed & VBO_IDCE) { mIID = vbo[s]; s++;}
	if (mUsed & VBO_COLR) { mCID = vbo[s]; s++;}
	if (mUsed & VBO_NORM) { mNID = vbo[s]; s++;}
	if (mUsed & VBO_TEXC) { mTID = vbo[s]; s++;}
	if (mUsed & VBO_TEXI) { mTTD = vbo[s]; s++;}
	
	// Create and bind the VAO
	glGenVertexArrays(1,&vao);
	glBindVertexArray(vao);
	
	// Populate buffers
	
	allocateVertices();
	allocateIndices();
	allocateColours();
	allocateNormals();
	allocateTexCoords();
	allocateTexIDs();
	s = 0;
	
	if (mUsed & VBO_VERT){
		glEnableVertexAttribArray(s);
		glBindBuffer(GL_ARRAY_BUFFER, mVID);
		glVertexAttribPointer(s,3,GL_FLOAT,GL_FALSE,0, (GLubyte*) NULL);
		mNumElements = mVertices.size() / 3;
		s++;
	} 
	if (mUsed & VBO_IDCE){
		glEnableVertexAttribArray(s);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIID);
		glVertexAttribPointer(s,1,GL_UNSIGNED_INT,GL_FALSE,0, (GLubyte*) NULL);
		mNumIndices = mIndices.size();
		s++;
	} 
	if (mUsed & VBO_COLR){
		glEnableVertexAttribArray(s);
		glBindBuffer(GL_ARRAY_BUFFER, mCID);
		glVertexAttribPointer(s,4,GL_FLOAT,GL_FALSE,0, (GLubyte*) NULL);
		s++;
	} 
	if (mUsed & VBO_NORM){
		glEnableVertexAttribArray(s);
		glBindBuffer(GL_ARRAY_BUFFER, mNID);
		glVertexAttribPointer(s,3,GL_FLOAT,GL_FALSE,0, (GLubyte*) NULL);
		s++;
	}
	if (mUsed & VBO_TEXC){
		glEnableVertexAttribArray(s);
		glBindBuffer(GL_ARRAY_BUFFER, mTID);
		glVertexAttribPointer(s,2,GL_FLOAT,GL_FALSE,0, (GLubyte*) NULL);
		s++;
	} 
	
	if (mUsed & VBO_TEXI){
		glEnableVertexAttribArray(s);
		glBindBuffer(GL_ARRAY_BUFFER, mTTD);
		///\todo NOTE THE I in the function below!
		glVertexAttribIPointer(s,1,GL_UNSIGNED_INT,0, (GLubyte*) NULL);
		s++;
	} 
	
	mNumBufs = s;

	glBindVertexArray(0);
	
	///\todo - Not sure why the below is needed but it stops things breaking
	
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	for (int id = 0; id < mNumBufs; ++id){
		glDisableVertexAttribArray(id);
	}
	
}

/*
 * Allocate based on the current size of the arrays
 */

void VBOData::allocateVertices() {
	if (mUsed & VBO_VERT){
		glBindBuffer(GL_ARRAY_BUFFER, mVID);
		glBufferData(GL_ARRAY_BUFFER, mVertices.size() * sizeof(GLfloat), &mVertices[0], GL_DYNAMIC_DRAW);
	}
}

void VBOData::allocateIndices() {
	if (mUsed & VBO_IDCE){
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, mIndices.size() * sizeof(GLuint), &mIndices[0], GL_DYNAMIC_DRAW);
	}
}

void VBOData::allocateTexCoords() {
	if (mUsed & VBO_TEXC){
		glBindBuffer(GL_ARRAY_BUFFER, mTID);
		glBufferData(GL_ARRAY_BUFFER, mTexCoords.size()  * sizeof(GLfloat), &mTexCoords[0], GL_DYNAMIC_DRAW);
	}
}

void VBOData::allocateNormals() {
	if (mUsed & VBO_NORM){
		glBindBuffer(GL_ARRAY_BUFFER, mNID);
		glBufferData(GL_ARRAY_BUFFER, mNormals.size() * sizeof(GLfloat), &mNormals[0], GL_DYNAMIC_DRAW);
	}
}

void VBOData::allocateColours() {
	if (mUsed & VBO_COLR){
		glBindBuffer(GL_ARRAY_BUFFER, mCID);
		glBufferData(GL_ARRAY_BUFFER, mColours.size() * sizeof(GLfloat), &mColours[0], GL_DYNAMIC_DRAW);
	}
}

void VBOData::allocateTexIDs() {
	if (mUsed & VBO_TEXI){
		glBindBuffer(GL_ARRAY_BUFFER, mTTD);
		glBufferData(GL_ARRAY_BUFFER, vTexIDs.size() * sizeof(GLuint), &vTexIDs[0], GL_DYNAMIC_DRAW);
	}
}


Camera::Camera(){
	mUp = glm::vec3(0,1,0);
	mPos = glm::vec3(0,0,1.0);
	mLook = glm::vec3(0,0,-1.0);
}
	
void Camera::yaw(float_t a){
	compute(0.0,a,0.0);
}

void Camera::pitch(float_t a){
	compute(a,0.0,0.0);
}

void Camera::roll(float_t a){
	compute(0.0,0.0,a);
}
	
void Camera::move(glm::vec3 m){
	mPos += m;
	compute(0.0,0.0,0.0);
}

void Camera::setRatio(float_t r) {
	mR = r;
	compute(0.0,0.0,0.0);
}
	
void Camera::compute(float_t ax, float_t ay, float_t az) {

	glm::quat q_rotate;
	
	q_rotate = glm::rotate( q_rotate, ay, glm::vec3( 0, 1, 0 ) );
	q_rotate = glm::rotate( q_rotate, ax, glm::vec3( 1, 0, 0 ) );

	mLook = q_rotate * mLook;
	mUp = q_rotate * mUp;
	
	mViewMatrix = glm::lookAt(mPos, mPos + mLook, mUp);
	
	mProjectionMatrix = glm::perspective(55.0f,mR,1.0f, 3000.0f);
}

OrbitCamera::OrbitCamera() : Camera() {
	mPos = glm::vec3(0,0,1.0);
	mLook = glm::vec3(0,0,0);
}

void OrbitCamera::compute(float_t ax, float_t ay, float_t az) {
	
	glm::quat q_rotate;
	
	q_rotate = glm::rotate( q_rotate, ay, glm::vec3( 0, 1, 0 ) );
	q_rotate = glm::rotate( q_rotate, ax, glm::vec3( 1, 0, 0 ) );

	mPos = q_rotate * mPos;
	mUp = q_rotate * mUp;
	
	mViewMatrix = glm::lookAt(mPos, mLook, mUp);
	
	mProjectionMatrix = glm::perspective(55.0f,mR,1.0f, 3000.0f);
}

void OrbitCamera::zoom(float_t z) {
	glm::vec3 dir = mPos - mLook;
	dir = glm::normalize(dir);
	dir *= z;
	mPos += dir;
	compute(0,0,0);
}

void OrbitCamera::shift(float_t du, float_t dv) {
	glm::vec3 dir = mPos - mLook;
	dir = glm::normalize(dir);
	glm::vec3 shiftx = glm::cross(dir,mUp);
	shiftx *= du;
	glm::vec3 shifty = mUp * dv;

	mPos += shiftx + shifty;
	mLook += shiftx + shifty;
	
	compute(0,0,0);
}

/*
 * Primitive Generation
 */
 
void Primitive::generate(std::string filename, glm::vec3 scale = glm::vec3(1.0,1.0,1.0) ) {
	
	loadAsset(filename);
	cerr << "Leeds - Loaded " << filename << " with " << mVBO.mNumElements / 3 <<  " faces." <<endl;
	mScale = scale;
}

/*
 * Recompute the matrices on the primitive
 */
 
void Primitive::compute() {
	mTransMatrix = glm::translate(glm::mat4(1.0f), mPos);
	
	glm::mat4 ViewRotateX = glm::rotate( glm::mat4(1.0f), mRot.x, glm::vec3(1.0f, 0.0f, 0.0f)); 
	glm::mat4 ViewRotateY = glm::rotate( glm::mat4(1.0f), mRot.y, glm::vec3(0.0f, 1.0f, 0.0f)); 
	glm::mat4 ViewRotateZ = glm::rotate( glm::mat4(1.0f), mRot.z, glm::vec3(0.0f, 0.0f, 1.0f)); 
	
	mRotMatrix = ViewRotateZ * ViewRotateY * ViewRotateX;
	
	mScaleMatrix = glm::scale(glm::mat4(1.0f), mScale);
}


int Primitive::loadAsset (std::string filename){

	pScene = aiImportFile(filename.c_str(),aiProcessPreset_TargetRealtime_MaxQuality);
	if (pScene) {
	/*	get_bounding_box(&scene_min,&scene_max);
		scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
		scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
		scene_center.z = (scene_min.z + scene_max.z) / 2.0f;*/
		recursiveCreate(pScene, pScene->mRootNode);
		
		mVBO.mNumElements = mVBO.mVertices.size() / 3;
		mVBO.compile(VBO_VERT | VBO_NORM);
		checkError(__LINE__);
		
		return 0;
	
	}
	return 1;
}

Primitive::~Primitive() {
	aiReleaseImport(pScene);
}


/*
 * Basic setup function for shared objects
 */

 void Drawer::setup(GlobalConfig &config) {
	mObj.reset(new SharedObj(config));
	// Load Shaders
	mObj->mShaderFlatCam.load("./data/flatcam.vert", "./data/flatcam.frag");
	mObj->mShaderMeshPoints.load("./data/meshpoint.vert", "./data/meshpoint.frag");
	mObj->mShaderQuad.load("./data/quad.vert", "./data/quad.frag");
	mObj->mShaderLighting.load("./data/lighting.vert", "./data/lighting.frag");
	mObj->mShaderNormals.load("./data/meshnormal.vert", "./data/meshnormal.frag");
	mObj->mShaderGripper.load("./data/gripper.vert", "./data/gripper.frag");
	mObj->mShaderPicker.load("./data/picker.vert", "./data/picker.frag");
	
	mObj->mFBOPick.setup(640,480);
	
	// Setup VBO objects
	// Quad for Camera drawing
	mObj->mVCam.mIndices += 0,3,1,3,2,1;
	mObj->mVCam.mVertices += 0.0f,0.0f,0.0f,
		(float_t)config.camSize.width, 0.0f,0.0f, 
		(float_t)config.camSize.width, (float_t)config.camSize.height,0.0f,
		0.0f, (float_t)config.camSize.height,0.0f;
	
	mObj->mVCam.mTexCoords += 0.0, (float_t)config.camSize.height,
		(float_t)config.camSize.width, (float_t)config.camSize.height,
		(float_t)config.camSize.width, 0.0,
		0.0,0.0;
		
	mObj->mVCam.compile(VBO_VERT | VBO_IDCE | VBO_TEXC );
	
	//Quad for reference. Square that is 1 x 1 and can then be scaled
	mObj->mVQuad.mIndices += 0,3,1,1,2,3;
	mObj->mVQuad.mVertices += 0.0f,0.0f,0.0f,
		(float_t)1.0f, 0.0f,0.0f, 
		(float_t)1.0f, (float_t)1.0f,0.0f,
		0.0f, (float_t)1.0f,0.0f;
	
	mObj->mVQuad.mColours += 0.0f,0.0f,0.0f,1.0f,
		0.0f,0.0f,1.0f,1.0f,
		0.0f,1.0f,0.0f,1.0f,
		1.0f,0.0f,0.0f,1.0f;
		
	mObj->mVQuad.compile(VBO_VERT | VBO_IDCE | VBO_COLR );
	
	// Setup ASSIMP to load models
	
	mObj->mStream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT,NULL);
	aiAttachLogStream(&mObj->mStream);
	
	mObj->mGripper.generate("./data/gripper.stl", glm::vec3(0.2,0.2,0.2) );

	
	// Move camera back along the Z Axis
	mObj->mCam.move( glm::vec3(0, 0.0, 400.0) );
}

void Drawer::resize(size_t w, size_t h){
	mObj->mW = w;
	mObj->mH = h;
	
	mObj->mFBOPick.resize(w,h);
	
	mObj->mCam.setRatio( static_cast<float_t>(mObj->mW) / mObj->mH);
	
	float_t sf = mObj->mW / (mObj->mConfig.xe - mObj->mConfig.xs);
	mObj->mModelMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(sf,sf,sf));
}

/*
 * Drawer Destructor
 */
 
Drawer::~Drawer() {
	aiDetachAllLogStreams();
}


void Primitive::recursiveCreate (const struct aiScene *sc, const struct aiNode* nd) {
	unsigned int i;
	unsigned int n = 0, t;

	// draw all meshes assigned to this node
	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = pScene->mMeshes[nd->mMeshes[n]];

		for (t = 0; t < mesh->mNumFaces; ++t) {
			const struct aiFace* face = &mesh->mFaces[t];
			
			for(i = 0; i < face->mNumIndices; i++) {
				int index = face->mIndices[i];
				
			//	if(mesh->mColors[0] != NULL)
			//		glColor4fv((GLfloat*)&mesh->mColors[0][index]);
				
				if(mesh->mNormals != NULL) {
					mVBO.mNormals.push_back(mesh->mNormals[index].x);
					mVBO.mNormals.push_back(mesh->mNormals[index].y);
					mVBO.mNormals.push_back(mesh->mNormals[index].z);
				}
						
				mVBO.mVertices.push_back(mesh->mVertices[index].x);
				mVBO.mVertices.push_back(mesh->mVertices[index].y);
				mVBO.mVertices.push_back(mesh->mVertices[index].z);
			}
		
		}
		
	}

	for (n = 0; n < nd->mNumChildren; ++n) {
		recursiveCreate(sc, nd->mChildren[n]);
	}

}


/*
 * Draw Camera Flat - Set matrices to ortho and bind basic texture shader
 */
		 
void Drawer::drawCamerasFlat(std::vector<boost::shared_ptr<LeedsCam> > &cams) {
		
	// Assume nothing. Set the matrices
	size_t cami = 0;
	size_t camx = 0;
	float_t space = 20.0f;
	float_t x = space;
	float_t y = space;
	float_t sf = 0.5;

	glEnable(GL_TEXTURE_RECTANGLE);
	//glDisable(GL_CULL_FACE);

	mObj->mVCam.bind();
	BOOST_FOREACH (boost::shared_ptr<LeedsCam> cam, cams) {
	
		mObj->mShaderFlatCam.begin();
		glm::mat4 Projection = glm::ortho(0.0f,(float_t)mObj->mW,(float_t)mObj->mH,0.0f);
		glm::mat4 Model = glm::scale(glm::mat4(1.0f),glm::vec3(sf,sf,1.0)) * glm::translate(glm::mat4(1.0f), glm::vec3(x,y,0.0));
		glm::mat4 MVP = Projection * Model;
		
		GLint LocationMVP = glGetUniformLocation(mObj->mShaderFlatCam.getProgram(), "mMVPMatrix");
		glUniformMatrix4fv(	LocationMVP, 1, GL_FALSE, glm::value_ptr(MVP)); 
		
		cam->bindRectified();
		GLint LocationTex = glGetUniformLocation(mObj->mShaderFlatCam.getProgram(), "mBaseTex");
		glUniform1i(LocationTex,0);
		
		glDrawElements(GL_TRIANGLES, mObj->mVCam.mNumIndices, GL_UNSIGNED_INT, 0);
	
		mObj->mShaderFlatCam.end();
		
		// go by row and column via idx
		if ( x + space + (mObj->mConfig.camSize.width * sf)  > mObj->mW){
			camx = 0;
			x = space;
			y += mObj->mConfig.camSize.height + space;
		}
		else {
			x += space + mObj->mConfig.camSize.width;
		}
		
		cami++;
		camx++;
	}
	
	mObj->mVCam.unbind();
	
	glDisable(GL_TEXTURE_RECTANGLE);
	
}


/*
 * Draw One Camera Flat on a grid
 */
 
void Drawer::drawResultFlat(size_t cami = 0) {
	
	float_t space = 20.0f;
	float_t x = space;
	float_t y = space;
	float_t sf = 0.5;
	
	int cols = mObj->mW / (space + mObj->mConfig.camSize.width * sf);  
	
	int c = cami / cols;
	int r = cami % cols;
	
	x = space + c * (space + mObj->mConfig.camSize.width);
	y = space + r * (space + mObj->mConfig.camSize.height);
	
	glEnable(GL_TEXTURE_RECTANGLE);
	glDisable(GL_CULL_FACE);

	
	mObj->mVCam.bind();
	mObj->mShaderFlatCam.begin();
	glm::mat4 Projection = glm::ortho(0.0f,(float_t)mObj->mW,(float_t)mObj->mH,0.0f);
	glm::mat4 Model = glm::scale(glm::mat4(1.0f),glm::vec3(sf,sf,1.0)) * glm::translate(glm::mat4(1.0f), glm::vec3(x,y,0.0));
	glm::mat4 MVP = Projection * Model;
	
	GLint LocationMVP = glGetUniformLocation(mObj->mShaderFlatCam.getProgram(), "mMVPMatrix");
	glUniformMatrix4fv(	LocationMVP, 1, GL_FALSE, glm::value_ptr(MVP)); 
	
	GLint LocationTex = glGetUniformLocation(mObj->mShaderFlatCam.getProgram(), "mBaseTex");
	glUniform1i(LocationTex,0);
	
	glDrawElements(GL_TRIANGLES, mObj->mVCam.mNumIndices, GL_UNSIGNED_INT, 0);

	mObj->mShaderFlatCam.end();
				
	mObj->mVCam.unbind();
	
	glDisable(GL_TEXTURE_RECTANGLE);
	
}

/*
 * Test if the cursor picked a colour -  test against our FBO
 */
 
bool Drawer::picked(size_t u, size_t v, size_t s, glm::vec3 col){
	mObj->mFBOPick.bind();
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	int ds = s*s * 4;
	GLfloat *data = (GLfloat*) new GLfloat[ds];
	glReadPixels(u - s/2, v - s/2, s, s, GL_RGBA, GL_FLOAT, data);
	checkError(__LINE__);
	bool b = false;
	for (int i = 0; i < ds; i+=4){
		glm::vec3 c( data[i], data[i+1], data[i+2]);
		if (c == col){
			b = true;
			break;
		}
	}
	
	checkError(__LINE__);
	mObj->mFBOPick.unbind();
	delete data;
	return b;
}


void Drawer::moveGripper(float_t du, float_t dv, float_t dt) {
	
	glm::vec3 dir = mObj->mCam.getPos() - mObj->mCam.getLook();
	dir = glm::normalize(dir);
	glm::vec3 shiftx = glm::cross(dir,mObj->mCam.getUp());
	shiftx *= (du * dt * 100000.0f);
	glm::vec3 shifty = mObj->mCam.getUp() * (dv * dt * 100000.0f);

	glm::vec3 dpos = shiftx + shifty;
	mObj->mGripper.move(dpos);
}


/*
 * Draw a gripper tool
 */
 
void Drawer::drawGripper() {
	
	glm::mat4 MVP = mObj->mCam.getProjMatrix() * mObj->mCam.getViewMatrix() * mObj->mModelMatrix * mObj->mGripper.getMatrix();
	glm::mat4 MV = mObj->mCam.getProjMatrix() * mObj->mCam.getProjMatrix() * mObj->mGripper.getMatrix();
	glm::mat4 MN = glm::inverseTranspose(mObj->mCam.getViewMatrix());
	glm::mat4 MI = glm::inverse(mObj->mCam.getViewMatrix());

	// render to the picker
	
	mObj->mFBOPick.bind();
	mObj->mGripper.bind();
	mObj->mShaderPicker.begin();
	glClearBufferfv(GL_COLOR, 0, &glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)[0]);
	GLfloat depth = 1.0f;
	glClearBufferfv(GL_DEPTH, 0, &depth );
	GLint location = glGetUniformLocation(mObj->mShaderPicker.getProgram(), "uMVPMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MVP));
	location = glGetUniformLocation(mObj->mShaderPicker.getProgram(), "uColour");
	glUniform4f( location, 1.0f,0.0f,0.0f,1.0f); 
	checkError(__LINE__);
	glDrawArrays(GL_TRIANGLES, 0, mObj->mGripper.getNumElements());
	checkError(__LINE__);
	
	mObj->mShaderPicker.end();
	mObj->mGripper.unbind();
	mObj->mFBOPick.unbind();

	// Render to framebuffer as normal

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glEnable(GL_DEPTH_TEST);
	mObj->mGripper.bind();
	mObj->mShaderGripper.begin();
	checkError(__LINE__);

	location = glGetUniformLocation(mObj->mShaderGripper.getProgram(), "uMVPMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MVP)); 
	
	location = glGetUniformLocation(mObj->mShaderGripper.getProgram(), "uMVMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MV)); 
	
	location = glGetUniformLocation(mObj->mShaderGripper.getProgram(), "uNormalMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MN)); 
	
	location = glGetUniformLocation(mObj->mShaderGripper.getProgram(), "uMInverseMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MI)); 

	location = glGetUniformLocation(mObj->mShaderGripper.getProgram(), "uLight0");
	glUniform3f( location, 10.0f,5.0f,20.0f); 
	
	location = glGetUniformLocation(mObj->mShaderGripper.getProgram(), "uTangent");
	glUniform3f( location, 0.0f,0.0f,1.0f); 

	location = glGetUniformLocation(mObj->mShaderGripper.getProgram(), "uCamPos");
	glm::vec3 p = mObj->mCam.getPos();
	glUniform3f( location, p.x,p.y,p.z );
	checkError(__LINE__);

	// We ARE NOT USING DRAW ELEMENTS as we need to do per face operations so we duplicate vertices
	glDrawArrays(GL_TRIANGLES, 0, mObj->mGripper.getNumElements());

	mObj->mShaderGripper.end();
	mObj->mGripper.unbind();
	checkError(__LINE__);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);

}


/*
 * Update the internal camera from mouse
 */

static float_t sSense = 0.01;
 
void Drawer::rotateCamera(int dx, int dy, double dt) {

	if (dt <= 0)
		return;

	mObj->mCam.yaw( static_cast<float_t>(dx) * (sSense/static_cast<float_t>(dt)));
	mObj->mCam.pitch(static_cast<float_t>(dy) * (sSense/static_cast<float_t>(dt)));
	
}

/*
 * Zoom the camera
 */
 
void Drawer::zoomCamera(float_t z) {
	mObj->mCam.zoom(z);
}


/*
 * Shift the camera
 */

void Drawer::moveCamera(int dx, int dy, double dt){
	mObj->mCam.shift(static_cast<float_t>(dx), static_cast<float_t>(dy));

}

/*
 * Draw Mesh Filled in
 */
 
 void Drawer::drawMesh(VBOData &mesh, std::vector<boost::shared_ptr<LeedsCam> > &cams) {
	glm::mat4 MVP = mObj->mCam.getProjMatrix() * mObj->mCam.getViewMatrix()  * mObj->mModelMatrix;
	glm::mat4 MV = mObj->mCam.getViewMatrix() * mObj->mModelMatrix;
	glm::mat4 MN = glm::inverseTranspose(mObj->mCam.getViewMatrix());
	
	glEnable(GL_TEXTURE_RECTANGLE);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CW);
	glEnable(GL_DEPTH_TEST);
	
	// Enable Textures
	for (int i=0; i < cams.size(); i++){
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_RECTANGLE, cams[i]->getTexture());
	}
	checkError(__LINE__);

	// Draw the Points - maybe add a slight blur or alpha sprite
	mesh.bind();
	

	mObj->mShaderLighting.begin();
		
	GLint location = glGetUniformLocation(mObj->mShaderLighting.getProgram(), "mMVPMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MVP)); 
	
	location = glGetUniformLocation(mObj->mShaderLighting.getProgram(), "mMVMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MV)); 
	
	location = glGetUniformLocation(mObj->mShaderLighting.getProgram(), "mNormalMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MN)); 
	
	location = glGetUniformLocation(mObj->mShaderLighting.getProgram(), "mBaseTex");
	glUniform1i( location, 0); 
	
	location = glGetUniformLocation(mObj->mShaderLighting.getProgram(), "uShininess");
	glUniform1f( location, 20.0f); 
	
	location = glGetUniformLocation(mObj->mShaderLighting.getProgram(), "mLight0");
	glUniform3f( location, 1.0f,1.0f,1.0f); 
	
	// We ARE NOT USING DRAW ELEMENTS as we need to do per face operations so we duplicate vertices

	glDrawArrays(GL_TRIANGLES, 0, mesh.mNumElements);
	
	
	mObj->mShaderLighting.end();
	mesh.unbind();
	checkError(__LINE__);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	
	for (int i=0; i < cams.size(); i++){
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	glDisable(GL_TEXTURE_RECTANGLE);
 
 }
 
 /*
  * Draw a camera, taking up the maximum size of the screen - Assume texture is already bound
  */
  
void Drawer::drawCameraQuad() {

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	
	mObj->mVCam.bind();
	mObj->mShaderFlatCam.begin();
	glm::mat4 Projection = glm::ortho(0.0f,(float_t)mObj->mW,(float_t)mObj->mH,0.0f);
	
	float_t sf = mObj->mW > mObj->mH ? mObj->mH / mObj->mConfig.camSize.height : mObj->mW / mObj->mConfig.camSize.width;
	float_t x =  mObj->mW > mObj->mH ?( mObj->mW - (sf * mObj->mConfig.camSize.width) ) / 2.0: 0;
	float_t y =  mObj->mW > mObj->mH ? 0: ( mObj->mH - (sf * mObj->mConfig.camSize.height) ) / 2.0;
	
	glm::mat4 Model = glm::scale(glm::mat4(1.0f),glm::vec3(sf,sf,1.0)) * glm::translate(glm::mat4(1.0f), glm::vec3(x,y,0.0));
	glm::mat4 MVP = Projection * Model;
		
	GLint LocationMVP = glGetUniformLocation(mObj->mShaderFlatCam.getProgram(), "mMVPMatrix");
	glUniformMatrix4fv(	LocationMVP, 1, GL_FALSE, glm::value_ptr(MVP)); 
	
	GLint LocationTex = glGetUniformLocation(mObj->mShaderFlatCam.getProgram(), "mBaseTex");
	glUniform1i(LocationTex,0);
	
	glDrawElements(GL_TRIANGLES, mObj->mVCam.mNumIndices, GL_UNSIGNED_INT, 0);

	mObj->mShaderFlatCam.end();
	
	mObj->mVCam.unbind();
	checkError(__LINE__);
}


/*
 * Draw Mesh Points - We assume no indicies here
 */

void Drawer::drawMeshPoints(VBOData &points, float r, float g, float b){
	
	glm::mat4 MVP = mObj->mCam.getProjMatrix() * mObj->mCam.getViewMatrix()  * mObj->mModelMatrix;

	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_SRC_ALPHA);

	// Draw the Points - maybe add a slight blur or alpha sprite
	points.bind();
	mObj->mShaderMeshPoints.begin();
	glPointSize(2.0f);
		
	GLint location = glGetUniformLocation(mObj->mShaderMeshPoints.getProgram(), "mMVPMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MVP)); 
	
	location = glGetUniformLocation(mObj->mShaderMeshPoints.getProgram(), "mColour");
	glUniform4f( location, r,g,b,0.5f); 
	
	// No indicies here. Not needed
	glDrawArrays(GL_POINTS, 0, points.mNumElements);
	
	mObj->mShaderMeshPoints.end();
	points.unbind();
	checkError(__LINE__);
	glDisable(GL_BLEND);
}

/*
 * Draw Normals - pairs of points
 */
 
void Drawer::drawNormals(VBOData &lines) {
	
	glEnable(GL_DEPTH_TEST);
	
	glm::mat4 MVP = mObj->mCam.getProjMatrix() * mObj->mCam.getViewMatrix()  * mObj->mModelMatrix;

	// Draw the Points - maybe add a slight blur or alpha sprite
	lines.bind();
	mObj->mShaderNormals.begin();
		
	GLint location = glGetUniformLocation(mObj->mShaderNormals.getProgram(), "mMVPMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MVP)); 
	
	// No indicies here. Not needed
	glDrawArrays(GL_LINES, 0, lines.mNumElements);
	
	mObj->mShaderNormals.end();
	lines.unbind();
	checkError(__LINE__);
	glDisable(GL_DEPTH_TEST);
} 

void Drawer::drawCameraNormal(VBOData &line){
	glEnable(GL_DEPTH_TEST);
	
	glm::mat4 shift = glm::translate(glm::mat4(1.0f), glm::vec3(-10.0,5.0,5.0));
	glm::mat4 MVP =  mObj->mCam.getProjMatrix() * mObj->mCam.getViewMatrix() * mObj->mModelMatrix * shift;

	// Draw the Points - maybe add a slight blur or alpha sprite
	line.bind();
	mObj->mShaderNormals.begin();
		
	GLint location = glGetUniformLocation(mObj->mShaderNormals.getProgram(), "mMVPMatrix");
	glUniformMatrix4fv(	location, 1, GL_FALSE, glm::value_ptr(MVP)); 
	
	// No indicies here. Not needed
	glDrawArrays(GL_LINES, 0, line.mNumElements);
	
	mObj->mShaderNormals.end();
	line.unbind();
	checkError(__LINE__);
	glDisable(GL_DEPTH_TEST);
	
}

/*
 * Draw Reference Quad
 */
void Drawer::drawReferenceQuad(){
	
	glm::mat4 MVP =  mObj->mCam.getProjMatrix() * mObj->mCam.getViewMatrix() * mObj->mModelMatrix;

	mObj->mVQuad.bind();
	mObj->mShaderQuad.begin();
	
	GLint LocationMVP = glGetUniformLocation(mObj->mShaderQuad.getProgram(), "mMVPMatrix");

	glUniformMatrix4fv(	LocationMVP, 1, GL_FALSE, glm::value_ptr(MVP)); 
	glDrawElements(GL_TRIANGLES, mObj->mVQuad.mNumIndices, GL_UNSIGNED_INT, 0);

	mObj->mShaderQuad.end();
	mObj->mVQuad.unbind();
	checkError(__LINE__);
}




