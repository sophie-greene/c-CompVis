/**
* @brief Winged Edge Classes
* @file wingedge.hpp
* @date 09/07/2012
*
*/


#ifndef WINGEDGE_HPP
#define WINGEDGE_HPP

#include "common.hpp"
#include "geometry.hpp"

/*
 * Given a primtive with indices, create a winged edge structure for it
 */
 
namespace s9 {
	class WE_Vertex;
	class WE_Face;

	class WE_Edge {
	public:
		boost::shared_ptr<WE_Vertex> v0, v1;
		boost::shared_ptr<WE_Face> face;
		boost::shared_ptr<WE_Edge> prev, next, sym;
	};
	
	typedef boost::shared_ptr<WE_Edge> WEP_Edge;

	class WE_Vertex {
	public:
		std::vector< boost::shared_ptr<WE_Edge> > edges;
		size_t idc; // Index into original data
	};

	class WE_Face {
	public:
		boost::shared_ptr<WE_Edge> edge;
	};
	
	typedef boost::shared_ptr<WE_Face> WEP_Face;


	class WingedEdge {
	public:
		WingedEdge(){};
		void make(DrawableGeometry geom);
		std::vector<WEP_Face> getFaces() {return mObj->mWE; };
		virtual operator int() const { return mObj.use_count() > 0; };
	
	protected:
		class SharedObj {
		public:
			std::vector<WEP_Face> mWE;
			DrawableGeometry mGeom;
		};
		
		boost::shared_ptr<SharedObj> mObj;
	};


	/*
	 * Type safe version of the WingedEdge class for returning geometry flattened
	 */

	template <class T>
	class WingedEdgeT : public WingedEdge {
			Geometry<T> flatten(); 
	};
	
}

#endif
