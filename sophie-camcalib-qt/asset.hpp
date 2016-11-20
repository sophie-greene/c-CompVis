/*
* @brief GLFW Application
* @file asset.hpp
* @date 03/07/2012
*
*/

#ifndef S9_ASSET_HPP
#define S9_ASSET_HPP

#include "common.hpp"

#include "com/common.hpp"
#include "geometry.hpp"
#include "primitive.hpp"

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


namespace s9 {

	/*
	 * A class that combines a primitive with any geometry set - basic 3D building block outside of a shape
	 */

	template<class T>
	class Asset : public Primitive {
	protected:
		struct SharedObj{
			T mGeom;
		};
		boost::shared_ptr<SharedObj> mObj;

	public:
		Asset() {};
		virtual operator int() const { return mObj.use_count() > 0; };
		Asset(T geom) {mObj.reset(new SharedObj()); mObj->mGeom = geom; }
		T getGeometry() { return mObj->mGeom; };

	};

	// Handy typedefs

	typedef boost::shared_ptr<Asset<GeometryPNF> > AssetPtr;
	typedef Asset<GeometryPNF> AssetBasic;
	typedef Asset<GeometryFullFloat> AssetFull;
	
	/*
 	 * A wrapper around the Assimp library
 	 * \todo do something better with pScene
 	 */
	
	class AssetImporter {
	public:
		static AssetBasic load(std::string filename);
		
		virtual ~AssetImporter();

	protected:

		static AssetPtr _load (const struct aiScene *sc, const struct aiNode* nd, AssetPtr p);
		static const struct aiScene* pScene;

	};
}

#endif
