#include "depthMap.h"

#define  BUFSIZE 256




using namespace std;
//using namespace cv;
using namespace pcl;




mxArray * getMexArray(const std::vector<double>& v){
    mxArray * mx = mxCreateDoubleMatrix(v.size(),1, mxREAL);
    std::copy(v.begin(), v.end(), mxGetPr(mx));
    return mx;
}


/*****************************************************************************
Computes a depth map given a Normal map and some 3D constraints

input:
	normals: normal map
	mask: mask where to compute the depth map
	ep: matlab engine
	points3D: 3d constraints to fit the surface at a certain depth
	params: camera parameters
	viewer: just to viewing purposes
	weight: weight to assign to the 3D constraints term
Output:
	depthMap: the depth map

The depth map calculation is based on http://see.stanford.edu/materials/lsoeldsee263/07-ls-reg.pdf starting on pag. 7-2
in the first term of the least squares we enforce smoothness using the normals 
information as in http://pages.cs.wisc.edu/~csverma/CS766_09/Stereo/stereo.html
and in the second term we use the depth constraint in this pixel if available, 
using the points3D information as used in http://gfx.cs.princeton.edu/pubs/Vlasic_2009_DSC/perfcap.pdf (single view surface reconstruction section)
******************************************************************************/

cv::Mat calc_depthMap(double depth_factor, vector<vector<cv::Mat> >  normals, IplImage* mask,Engine *ep,vector<vector<vector<matchings_t> > > points3D, cam_params_t params, double cam_h,pcl::visualization::PCLVisualizer &viewer, double weight) {
	
	

	//all of this is basically an initial setup for the matlab variables
	mxArray *matlab_rowIndex = NULL, *matlab_colIndex = NULL, 
			*matlab_sparseMat_val = NULL,*matlab_v = NULL,*result = NULL,
			*matlab_objectPixelRow = NULL, *matlab_objectPixelCol = NULL,
			*matlab_sparseMat_valF = NULL, *matlab_rowIndexF = NULL,
			*matlab_colIndexF = NULL,*matlab_g = NULL,*matlab_dims = NULL,*matlab_weight=NULL;
	int mat_idx = 0;
	
	

	std::vector<double> weight_aux(1);
	weight_aux[0] = weight;
	cout << weight << endl;
	/*
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<PointXYZINormal>);

	for (int i = 0; i < points3D.size();i++){
		for (int j = 0; j < points3D[0].size();j++){
			if (points3D[i][j].size() > 0) {
				PointXYZINormal p;
				p.x = points3D[i][j][0].point.x;
				p.y = points3D[i][j][0].point.y;
				p.z = points3D[i][j][0].point.z;
				cloud->points.push_back(p);
			}
	
		}
	}
	viewer.removeAllPointClouds();
	viewer.addPointCloud<PointXYZINormal>(cloud,"cloud");
	*/
	float scalef = (float)mask->width/(float)640;
	float pixels_square_side = PIXEL3D*scalef;
		

	cv::Mat M = roto_translation(params.rot, params.trans);


	//we want the pixels coordinates that are inside the mask

	int nrows = mask->height;
	int ncols = mask->width;

	vector<double> objectPixelRow;
	vector<double> objectPixelCol;
	for (int i = 0; i < nrows; i++) {
		for (int j = 0; j < ncols; j++) {
			if (CV_IMAGE_ELEM(mask,uchar,i,j) > 100) {
				objectPixelRow.push_back(i);
				objectPixelCol.push_back(j);
			}
		}
	}
	vector<vector<int> >  index = vector<vector<int> > (nrows,vector<int>(ncols,-1));
	int numPixels = objectPixelRow.size();

	for (int d = 0; d < numPixels; d++) {
		index[objectPixelRow[d]][objectPixelCol[d]] = d;
	}
	
	vector<double> rowIndex;
	vector<double> colIndex;

	vector<double> rowIndexF;
	vector<double> colIndexF;

	vector<double> sparseMat_valF;
	vector<double> sparseMat_val;

	vector<double> v(numPixels*2,0);
	vector<double> g(numPixels*2,0);

	vector<double> image_dims(2);
	image_dims[0] = mask->height;
	image_dims[1] = mask->width;

	//for all the pixels that are inside the mask, we have to fill the sparse matrices

	
	for (int d = 1; d <= numPixels; d++) {

		//cout << d << endl;
		int pRow = objectPixelRow[d-1];
		int pCol = objectPixelCol[d-1];
		//cout << pRow << " " << pCol << endl;
		float nx = (float)(normals[pRow][pCol].at<double>(0));
		float ny = -(float)(normals[pRow][pCol].at<double>(1));
		float nz = (float)(normals[pRow][pCol].at<double>(2));
		double depth_val;
		double cam_height = cam_h*(1.0/depth_factor);
		bool flag = false;

		//if there is a 3D point constraint available in this pixel we convert it to depth
		if (points3D[pRow][pCol].size() > 0) {
			//if the constraint is inside what we consider feasible (not an outlier) 

			if ((points3D[pRow][pCol][0].point.x < 20 && points3D[pRow][pCol][0].point.x > -20) &&
				(points3D[pRow][pCol][0].point.y < 20 && points3D[pRow][pCol][0].point.y > -20) &&
				(points3D[pRow][pCol][0].point.z < 8 && points3D[pRow][pCol][0].point.z > 0)) {
				//activate a flag to take the depth constraint into account below in the sparse matrix calculation
				flag = true;
				cv::Mat aux(4,1,CV_64F);
				aux.at<double>(0) = points3D[pRow][pCol][0].point.x;
				aux.at<double>(1) = points3D[pRow][pCol][0].point.y;
				aux.at<double>(2) = points3D[pRow][pCol][0].point.z;
				aux.at<double>(3) = 1.0;
				aux = M*aux;
				depth_val = /*cam_height - */aux.at<double>(2)*(1.0/depth_factor);
			}
		}

		//for the first term of the equation we need to search for vectors in the surface conecting neighbouring pixels, 
		//to enforce that they remain perpendicular to the normal of the surface
		//Now there are some if-else to see in the neigbouring pixel is inside the object(mask)
		if ((index[pRow][pCol+1] >=0) && (index[pRow-1][pCol])>=0){
			//Both  (X+1, Y) and (X, Y+1) are still inside the object.
			
			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol+1]); sparseMat_val.push_back(-1);
			v[2*d-2] = nx/nz;
			
			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow-1][pCol]); sparseMat_val.push_back(-1);
			v[2*d-1] = ny/nz;

			// depth constraints
			if (flag/*points3D[pRow][pCol].size() > 0*/) {
				rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
				g[2*d-2] = depth_val;
				rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
				g[2*d-1] = depth_val;
			}
			else {
				rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
				rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
				
			}
		}
		else if (index[pRow-1][pCol] >= 0) {
			//(X, Y+1) is still inside the object, but (X+1, Y) is outside the object.
			
			if (index[pRow][pCol-1] >=0) {

				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol-1]); sparseMat_val.push_back(-1);	
				v[2*d-2] = -nx/nz;
				// depth constraints
			if (flag/*points3D[pRow][pCol].size() > 0*/) {
					
					rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
					g[2*d-2] = depth_val;
				}
				else {
					rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
				}
			}
			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow-1][pCol]); sparseMat_val.push_back(-1);
			v[2*d-1] = ny/nz;
			// depth constraints
			if (flag/*points3D[pRow][pCol].size() > 0*/) {
				
				rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
				g[2*d-1] = depth_val;
			}
			else {
				rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
			}
		}
		else if (index[pRow][pCol+1] >= 0){
			
			
			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol+1]); sparseMat_val.push_back(-1);
			v[2*d-2] = nx/nz;
			// depth constraints
			if (flag/*points3D[pRow][pCol].size() > 0*/) {
				
				rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
				g[2*d-2] = depth_val;
			}
			else {
				rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
			}
			
			if (index[pRow+1][pCol] >=0) {
				
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow+1][pCol]); sparseMat_val.push_back(-1);
				v[2*d-1] = -ny/nz;
				// depth constraints
				if (flag/*points3D[pRow][pCol].size() > 0*/) {
					
					rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
					g[2*d-1] = depth_val;
				}
				else {
					rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
				}
			}	
		}
		else {
			
			if (index[pRow][pCol-1] >=0) {
				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol-1]); sparseMat_val.push_back(-1);
				v[2*d-2] = -nx/nz;
				//depth constraints
				if (flag/*points3D[pRow][pCol].size() > 0*/) {
					
					rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
					g[2*d-2] = depth_val;
				}
				else {
					rowIndexF.push_back(2*d-2);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
				}

			}
			if (index[pRow+1][pCol] >=0) {
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow+1][pCol]); sparseMat_val.push_back(-1);
				v[2*d-1] = -ny/nz;
				//depth constraints
				if (flag/*points3D[pRow][pCol].size() > 0*/) {
					
					rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(1);
					g[2*d-1] = depth_val;
				}
				else {
					rowIndexF.push_back(2*d-1);colIndexF.push_back(index[pRow][pCol]);sparseMat_valF.push_back(0);
				}
			}
		}
	}
	//cout << numPixels*2 << "Triplet size: " << tripletList.size() << endl;


	//fill all matlab variables to calculate least squares optimization
	matlab_rowIndex = getMexArray(rowIndex);
	matlab_colIndex = getMexArray(colIndex);
	matlab_rowIndexF = getMexArray(rowIndexF);
	matlab_colIndexF = getMexArray(colIndexF);

	matlab_weight = getMexArray(weight_aux);
	matlab_sparseMat_val = getMexArray(sparseMat_val);
	matlab_sparseMat_valF = getMexArray(sparseMat_valF);
	matlab_v = getMexArray(v);
	matlab_g = getMexArray(g);

	matlab_objectPixelCol = getMexArray(objectPixelCol);
	matlab_objectPixelRow = getMexArray(objectPixelRow);
	matlab_dims = getMexArray(image_dims);
	
	engPutVariable(ep, "weight", matlab_weight);
	engPutVariable(ep, "ri", matlab_rowIndex);
	engPutVariable(ep, "ci", matlab_colIndex);
	engPutVariable(ep, "riF", matlab_rowIndexF);
	engPutVariable(ep, "ciF", matlab_colIndexF);
	
	engPutVariable(ep, "vals", matlab_sparseMat_val);
	engPutVariable(ep, "valsF", matlab_sparseMat_valF);
	engPutVariable(ep, "v", matlab_v);
	engPutVariable(ep, "g", matlab_g);

	engPutVariable(ep, "opRow", matlab_objectPixelRow);
	engPutVariable(ep, "opCol", matlab_objectPixelCol);
	engPutVariable(ep, "dims", matlab_dims);
	
	
	engEvalString(ep, "Z = LSDepthMap(ri,ci,vals,v,opRow,opCol,riF,ciF,valsF,g,dims,weight)");
	result = engGetVariable(ep,"Z");
	
	double* temp = mxGetPr(result);

	
	cv::Mat depthMap(nrows,ncols,CV_64F);
	
	for (int i = 0; i < nrows*ncols; i++) {
		depthMap.at<double>((i%nrows)*ncols + (int)((float)i/(float)nrows)) = temp[i];
	}
	
	mxDestroyArray(matlab_weight);
	mxDestroyArray(result);
	mxDestroyArray(matlab_rowIndex);
	mxDestroyArray(matlab_colIndex);
	mxDestroyArray(matlab_rowIndexF);
	mxDestroyArray(matlab_colIndexF);

	mxDestroyArray(matlab_sparseMat_val);
	mxDestroyArray(matlab_sparseMat_valF);
	mxDestroyArray(matlab_v);
	mxDestroyArray(matlab_g);
	mxDestroyArray(matlab_objectPixelCol);
	mxDestroyArray(matlab_objectPixelRow);
	mxDestroyArray(matlab_dims);

	
	engEvalString(ep, "clear");
	
	return depthMap;
}
