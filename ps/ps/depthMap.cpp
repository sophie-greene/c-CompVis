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

cv::Mat calc_depthMap(vector<vector<cv::Mat> > normals, IplImage* mask,Engine *ep) {
	
	
	mxArray *matlab_rowIndex = NULL, *matlab_colIndex = NULL, 
			*matlab_sparseMat_val = NULL,*matlab_v = NULL,*result = NULL,
			*matlab_objectPixelRow = NULL, *matlab_objectPixelCol = NULL,
			*matlab_sparseMat_valF = NULL, *matlab_rowIndexF = NULL,
			*matlab_colIndexF = NULL,*matlab_g = NULL,*matlab_dims = NULL;
	int mat_idx = 0;
	

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
        vector<vector<int> > index = vector<vector<int> >(nrows,vector<int>(ncols,-1));
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

	vector<double> v(numPixels*2);
	vector<double> g(numPixels*2,100);

	vector<double> image_dims(2);
	image_dims[0] = mask->height;
	image_dims[1] = mask->width;

	
	for (int d = 1; d <= numPixels; d++) {

		//cout << d << endl;
		int pRow = objectPixelRow[d-1];
		int pCol = objectPixelCol[d-1];
		//cout << pRow << " " << pCol << endl;
		float nx = (float)(normals[pRow][pCol].at<double>(0));
		float ny = -(float)(normals[pRow][pCol].at<double>(1));
		float nz = -(float)(normals[pRow][pCol].at<double>(2));
		rowIndexF.push_back(2*d-2);
		colIndexF.push_back(index[pRow][pCol]);
		sparseMat_valF.push_back(1);
		if ((index[pRow][pCol+1] >=0) && (index[pRow-1][pCol])>=0){
			//Both  (X+1, Y) and (X, Y+1) are still inside the object.
			
			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);

			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol+1]); sparseMat_val.push_back(-1);

			v[2*d-2] = nx/nz;
			
			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);

			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow-1][pCol]); sparseMat_val.push_back(-1);

			v[2*d-1] = ny/nz;

			
		}
		else if (index[pRow-1][pCol] >= 0) {
			//(X, Y+1) is still inside the object, but (X+1, Y) is outside the object.
			
			if (index[pRow][pCol-1] >=0) {

				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol-1]); sparseMat_val.push_back(-1);
				
				v[2*d-2] = -nx/nz;
			}
			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
			rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow-1][pCol]); sparseMat_val.push_back(-1);
			v[2*d-1] = ny/nz;
		}
		else if (index[pRow][pCol+1] >= 0){
			//cout << d << ": 3" << endl;
			
			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
			rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol+1]); sparseMat_val.push_back(-1);
			
			v[2*d-2] = nx/nz;
			
			if (index[pRow+1][pCol] >=0) {
				
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow+1][pCol]); sparseMat_val.push_back(-1);
				
				v[2*d-1] = -ny/nz;
			}
			
		}
		else {
			//cout << d << ": 4" << endl;
			if (index[pRow][pCol-1] >=0) {
				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-2); colIndex.push_back(index[pRow][pCol-1]); sparseMat_val.push_back(-1);

				v[2*d-2] = -nx/nz;
			}
			if (index[pRow+1][pCol] >=0) {
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow][pCol]); sparseMat_val.push_back(1);
				rowIndex.push_back(2*d-1); colIndex.push_back(index[pRow+1][pCol]); sparseMat_val.push_back(-1);

				v[2*d-1] = -ny/nz;
			}
		}
	}
	//cout << numPixels*2 << "Triplet size: " << tripletList.size() << endl;

	matlab_rowIndex = getMexArray(rowIndex);
	matlab_colIndex = getMexArray(colIndex);
	matlab_rowIndexF = getMexArray(rowIndexF);
	matlab_colIndexF = getMexArray(colIndexF);

	matlab_sparseMat_val = getMexArray(sparseMat_val);
	matlab_sparseMat_valF = getMexArray(sparseMat_valF);
	matlab_v = getMexArray(v);
	matlab_g = getMexArray(g);

	matlab_objectPixelCol = getMexArray(objectPixelCol);
	matlab_objectPixelRow = getMexArray(objectPixelRow);
	matlab_dims = getMexArray(image_dims);
	

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
	
	
	engEvalString(ep, "Z = LSDepthMap(ri,ci,vals,v,opRow,opCol,riF,ciF,valsF,g,dims)");
	result = engGetVariable(ep,"Z");
	
	double* temp = mxGetPr(result);

	
	cv::Mat depthMap(nrows,ncols,CV_64F);
	
	for (int i = 0; i < nrows*ncols; i++) {
		depthMap.at<double>((i%nrows)*ncols + (int)((float)i/(float)nrows)) = temp[i];
	}
	
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
	/*
	
	//M.setFromTriplets(tripletList.begin(),tripletList.end());

	//cout << "non zeros in M: " << M.nonZeros() << endl;
	
	//Eigen::ConjugateGradient<Eigen::SparseMatrix<double> > cg;
	
        //Eigen::SparseQR<Eigen::SparseMatrix<double>,Eigen::NaturalOrdering<int> > cg;
	
	
	//cg.compute(M);

	//if(cg.info()!=Eigen::Success) {
	//	cout <<  "decomposition failed" << endl;
	//	exit(0);
	//}

	cout << "solving..."<<endl;
	
	x = cg.solve(v);

	Mat aux(x.size(),1,CV_64F);

	for (int i = 0; i < x.size(); i++) {
		aux.at<double>(i) = x[i];
		//cout << v[i] << endl;
	}
	
	FileStorage fs("test.yml", FileStorage::WRITE);
	fs << "solution" << aux;

	fs.release();

	//std::cout << "#iterations: " << cg.iterations() << std::endl;
	//std::cout << "estimated error: " << cg.error() << std::endl;

	//Mat *x;
	//cvSolve(&M,&v,x,DECOMP_CHOLESKY);

	//cout << x->rows << " " << x->cols << endl;

	*/
	
	
}



cv::Mat calc_depthMap(double depth_factor, vector<vector<cv::Mat> > normals, IplImage* mask,Engine *ep,vector<vector<vector<matchings_t> > > points3D, cam_params_t params, double cam_h,pcl::visualization::PCLVisualizer &viewer, double weight) {
	
	
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
        vector<vector<int> > index = vector<vector<int> >(nrows,vector<int>(ncols,-1));
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
		if (points3D[pRow][pCol].size() > 0) {
			if ((points3D[pRow][pCol][0].point.x < 20 && points3D[pRow][pCol][0].point.x > -20) &&
				(points3D[pRow][pCol][0].point.y < 20 && points3D[pRow][pCol][0].point.y > -20) &&
				(points3D[pRow][pCol][0].point.z < 8 && points3D[pRow][pCol][0].point.z > 0)) {
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
			//cout << d << ": 3" << endl;
			
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
			//cout << d << ": 4" << endl;
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
