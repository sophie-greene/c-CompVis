#include "../include/bundle_adjustment.h"

BundleAdjust::BundleAdjust()
{

}

cv::Mat _K;
cv::Mat _dist;

void my_fjac(int /*i*/, int /*j*/, CvMat *point_params, CvMat* cam_params, CvMat* A, CvMat* B, void* /*data*/) {
  //compute jacobian per camera parameters (i.e. Aij)
  //take i-th point 3D current coordinates

  CvMat _Mi;
  cvReshape(point_params, &_Mi, 3, 1 );

  CvMat* _mp = cvCreateMat(1, 1, CV_64FC2 ); //projection of the point

  //split camera params into different matrices
  CvMat _ri, _ti, _k;
  cvGetRows( cam_params, &_ri, 0, 3 );
  cvGetRows( cam_params, &_ti, 3, 6 );

  /*double intr_data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 1};
    intr_data[0] = cam_params->data.db[6];
    intr_data[4] = cam_params->data.db[7];
    intr_data[2] = cam_params->data.db[8];
    intr_data[5] = cam_params->data.db[9];
    CvMat _A = cvMat(3,3, CV_64F, intr_data );*/

  CvMat _A = _K;
  CvMat _d = _dist;

  CvMat _dpdr, _dpdt;

  cvGetCols( A, &_dpdr, 0, 3 );
  cvGetCols( A, &_dpdt, 3, 6 );
  //cvGetCols( A, &_dpdf, 6, 8 );
  //cvGetCols( A, &_dpdc, 8, 10 );

  cvProjectPoints2(&_Mi, &_ri, &_ti, &_A, &_d, _mp, &_dpdr, &_dpdt,
                   NULL, NULL, NULL, 0);

  cvReleaseMat( &_mp );


  //get rotation matrix
  double R[9], t[3];
  double fx = _K.at<double>(0,0);
  double fy = _K.at<double>(1,1);
  //intr_data[0], fy = intr_data[4];

  CvMat _R = cvMat( 3, 3, CV_64F, R );
  cvRodrigues2(&_ri, &_R);

  double X,Y,Z;
  X = point_params->data.db[0];
  Y = point_params->data.db[1];
  Z = point_params->data.db[2];

  t[0] = _ti.data.db[0];
  t[1] = _ti.data.db[1];
  t[2] = _ti.data.db[2];

  //compute x,y,z
  double x = R[0] * X + R[1] * Y + R[2] * Z + t[0];
  double y = R[3] * X + R[4] * Y + R[5] * Z + t[1];
  double z = R[6] * X + R[7] * Y + R[8] * Z + t[2];

#if 1
  double coeff[6] = { z, 0, -x,
                      0, z, -y };
  CvMat coeffmat = cvMat( 2, 3, CV_64F, coeff );

  CvMat* dstrike_dbig = cvCreateMat(2,3,CV_64F);
  cvMatMul(&coeffmat, &_R, dstrike_dbig);
  cvScale(dstrike_dbig, dstrike_dbig, 1/(z*z) );

  cvCopy(dstrike_dbig, B);
  cvReleaseMat(&dstrike_dbig);

  //multiply by fx, fy
  CvMat row;
  cvGetRows( B, &row, 0, 1 );
  cvScale( &row, &row, fx );

  cvGetRows( B, &row, 1, 2 );
  cvScale( &row, &row, fy );

#else

  double k = fx/(z*z);

  cvmSet( B, 0, 0, k*(R[0]*z-x*R[6]));
  cvmSet( B, 0, 1, k*(R[1]*z-x*R[7]));
  cvmSet( B, 0, 2, k*(R[2]*z-x*R[8]));

  k = fy/(z*z);

  cvmSet( B, 1, 0, k*(R[3]*z-y*R[6]));
  cvmSet( B, 1, 1, k*(R[4]*z-y*R[7]));
  cvmSet( B, 1, 2, k*(R[5]*z-y*R[8]));

#endif
}

void my_func(int /*i*/, int /*j*/, CvMat *point_params, CvMat* cam_params, CvMat* estim, void* /*data*/) {
  //just do projections
  CvMat _Mi;
  cvReshape( point_params, &_Mi, 3, 1 );

  CvMat* _mp = cvCreateMat(1, 1, CV_64FC2 ); //projection of the point
  CvMat* _mp2 = cvCreateMat(1, 2, CV_64F ); //projection of the point

  //split camera params into different matrices
  CvMat _ri, _ti;

  cvGetRows( cam_params, &_ri, 0, 3 );
  cvGetRows( cam_params, &_ti, 3, 6 );

  /*double intr_data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 1};
    intr_data[0] = cam_params->data.db[6];
    intr_data[4] = cam_params->data.db[7];
    intr_data[2] = cam_params->data.db[8];
    intr_data[5] = cam_params->data.db[9];
    CvMat _A = cvMat(3,3, CV_64F, intr_data );*/

  CvMat _A = _K;
  CvMat _d = _dist;


  cvProjectPoints2( &_Mi, &_ri, &_ti, &_A, &_d, _mp, NULL, NULL,
                    NULL, NULL, NULL, 0);

  _mp2->data.db[0] = _mp->data.db[0];
  _mp2->data.db[1] = _mp->data.db[1];
  cvTranspose( _mp2, estim );
  cvReleaseMat( &_mp );
  cvReleaseMat( &_mp2 );
}

void my_fjac_ba(int i, int j, cv::Mat& point_params, cv::Mat& cam_params, cv::Mat& A, cv::Mat& B, void* data) {
  CvMat _point_params = point_params, _cam_params = cam_params, _Al = A, _Bl = B;
  my_fjac(i,j, &_point_params, &_cam_params, &_Al, &_Bl, data);
}

void my_func_ba(int i, int j, cv::Mat& point_params, cv::Mat& cam_params, cv::Mat& estim, void* data)  {
  CvMat _point_params = point_params, _cam_params = cam_params, _estim = estim;
  my_func(i,j,&_point_params,&_cam_params,&_estim,data);
}

void BundleAdjust::bundleAdjustExtrinsic( std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                                           cv::Mat &K, cv::Mat& distCoeffs, std::vector<cv::Mat>& Rvec, std::vector<cv::Mat>& tvec, int max_iter)
{
  _K = K.clone();
  _dist = distCoeffs.clone();
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, max_iter, DBL_EPSILON);

  int num_cameras = (int)imagePoints.size();
  int num_points = (int)points.size();
  int num_cam_param = 3 /* rotation vector */ + 3 /* translation vector */;
  int num_point_param = 3;

  assert(num_cameras==Rvec.size());
  assert(num_cameras==tvec.size());
  assert(num_cameras==visibility.size());
  for(int i=0;i<num_cameras;i++){
    assert( imagePoints.at(i).size() == num_points );
    assert( visibility.at(i).size() == num_points );
  }

  double error = 0.0;
  for(int i=0;i<num_cameras;i++){
    std::vector<cv::Point2d> pts = Tools::projPoints(Rvec.at(i),tvec.at(i),K,distCoeffs,points);
    for(int j=0;j<num_points;j++){
      if(visibility.at(i).at(j)){
        error += cv::norm(pts.at(j)-imagePoints.at(i).at(j));
      }
    }
  }
  std::cout << "Initial error : " << error << std::endl;


  cv::Mat params( num_cameras * num_cam_param + num_points * num_point_param, 1, CV_64F );

  //fill camera params
  for( int i = 0; i < num_cameras; i++ ) {
    //rotation
    cv::Mat rot_vec;
    cv::Rodrigues( Rvec[i].t(), rot_vec );
    cv::Mat dst = params.rowRange(i*num_cam_param, i*num_cam_param+3);
    rot_vec.copyTo(dst);

    //translation
    dst = params.rowRange(i*num_cam_param + 3, i*num_cam_param+6);
    cv::Mat _t = -Rvec[i].t()*tvec[i];
    _t.copyTo(dst);
    //tvec[i].copyTo(dst);
  }

  //fill point params
  cv::Mat ptparams(num_points, 1, CV_64FC3, params.data + num_cameras*num_cam_param*params.step);
  cv::Mat _points(points);
  CV_Assert(_points.size() == ptparams.size() && _points.type() == ptparams.type());
  _points.copyTo(ptparams);

  //convert visibility vectors to visibility matrix
  cv::Mat vismat(num_points, num_cameras, CV_32S);
  for( int i = 0; i < num_cameras; i++ ) {
    //get row
    cv::Mat col = vismat.col(i);
    cv::Mat((int)visibility[i].size(), 1, vismat.type(), (void*)&visibility[i][0]).copyTo( col );
  }

  int num_proj = countNonZero(vismat); //total number of points projections

  //collect measurements
  cv::Mat X(num_proj*2,1,CV_64F); //measurement vector

  int counter = 0;
  for(int i = 0; i < num_points; i++ ) {
    for(int j = 0; j < num_cameras; j++ ) {
      //check visibility
      if( visibility[j][i] ) {
        //extract point and put tu vector
        cv::Point2d p = imagePoints[j][i];
        ((double*)(X.data))[counter] = p.x;
        ((double*)(X.data))[counter+1] = p.y;
        assert(p.x != -1 || p.y != -1);
        counter+=2;
      }
    }
  }

  cv::LevMarqSparse levmar( num_points, num_cameras, num_point_param, num_cam_param, 2, vismat, params, X,
                            cv::TermCriteria(criteria), my_fjac_ba, my_func_ba, NULL,
                            NULL, NULL);

  std::cout << "Error norm: " << levmar.errNorm << std::endl;




  //extract results
  //fill point params
  /*Mat final_points(num_points, 1, CV_64FC3,
    levmar.P->data.db + num_cameras*num_cam_param *levmar.P->step);
    CV_Assert(_points.size() == final_points.size() && _points.type() == final_points.type());
    final_points.copyTo(_points);*/

  points.clear();
  CvMat *point_mat = cvCreateMat(3,1,CV_64F);
  for( int i = 0; i < num_points; i++ ) {
    cvGetSubRect( levmar.P, point_mat, cvRect( 0, levmar.num_cams * levmar.num_cam_param+ levmar.num_point_param * i, 1, levmar.num_point_param ));
    CvScalar x = cvGet2D(point_mat,0,0); CvScalar y = cvGet2D(point_mat,1,0); CvScalar z = cvGet2D(point_mat,2,0);
    points.push_back(cv::Point3d(x.val[0],y.val[0],z.val[0]));
    //std::cerr<<"point"<<points[points.size()-1].x<<","<<points[points.size()-1].y<<","<<points[points.size()-1].z<<std::endl;
  }
  cvReleaseMat(&point_mat);
  //fill camera params
  //R.clear();T.clear();cameraMatrix.clear();
  for( int i = 0; i < num_cameras; i++ ) {
    //rotation
    cv::Mat rot_vec = cv::Mat(levmar.P).rowRange(i*num_cam_param, i*num_cam_param+3);
    cv::Rodrigues( rot_vec, Rvec[i] );
    //translation
    tvec[i] = cv::Mat(levmar.P).rowRange(i*num_cam_param + 3, i*num_cam_param+6);

    Rvec[i] =  Rvec[i].t();
    tvec[i] = -Rvec[i]*tvec[i];
  }

  error = 0.0;
  for(int i=0;i<num_cameras;i++){
      std::vector<cv::Point2d> pts = Tools::projPoints(Rvec.at(i),tvec.at(i),K,distCoeffs,points);
    for(int j=0;j<num_points;j++){
      if(visibility.at(i).at(j)){
        error += cv::norm(pts.at(j)-imagePoints.at(i).at(j));
      }
    }
  }
  std::cout << "Final error : " << error << std::endl;

}

void BundleAdjust::bundleAdjust( std::vector<cv::Point3d>& points, std::vector<PointData>& imagePoints, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter)
{
  int nbPoints = points.size();
  int nbPoses = R.size();
  std::vector<std::vector<cv::Point2d> > newImagePoints;
  for(int i=0;i<nbPoses;i++)
    newImagePoints.push_back( std::vector<cv::Point2d>(nbPoints,cv::Point2d(0,0)) );
  std::vector<std::vector<int> > visibility;
  for(int i=0;i<nbPoses;i++)
    visibility.push_back( std::vector<int>(nbPoints,0) );

  for(int i=0;i<imagePoints.size();i++){
    int pt3D_id = imagePoints.at(i).getPtID();
    int pose_id = imagePoints.at(i).getPoseID();
    newImagePoints[pose_id][pt3D_id] = imagePoints.at(i).getPoint();
    visibility[pose_id][pt3D_id] = 1;
  }

  bundleAdjust(points,newImagePoints,visibility,cameraMatrix,distCoeffs,R,t,max_iter);

  /*cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, max_iter, DBL_EPSILON);

  int nbPoints = points.size();
  int nbCorrespondances = imagePoints.size();
  int nbImages = R.size();
  assert(R.size()==t.size());

  std::cout << "nbPoints : " << nbPoints << std::endl;
  std::cout << "nbCorrespondances : " << nbCorrespondances << std::endl;
  std::cout << "nbImages : " << nbImages << std::endl;

  std::vector<cv::Mat> cams, dist;
  for(int i=0;i<nbImages;i++){
    cams.push_back(cameraMatrix);
    dist.push_back(distCoeffs);
  }

  double error = 0.0;
  for(int i=0;i<nbImages;i++){
    std::vector<cv::Point2d> pts = Tools::projPoints(R.at(i),t.at(i),cameraMatrix,points);
    for(int j=0;j<nbCorrespondances;j++){
      if(imagePoints.at(j).camIndex==i){
        error += cv::norm(pts.at(imagePoints.at(j).ptIndex)-imagePoints.at(j).pt2d);
      }
    }
  }
  std::cout << "Initial error : " << error << std::endl;

  std::vector<cv::Mat> Rvec,tvec;
  for(int i=0;i<R.size();i++){
      Rvec.push_back( Tools::copy( R.at(i) ) );
      tvec.push_back( Tools::copy( t.at(i) ) );
  }

  for(int i=0;i<Rvec.size();i++){
      cv::Mat Rtemp = Rvec.at(i);
      cv::Mat ttemp = tvec.at(i);
      Rvec[i] = Rtemp.t();
      tvec[i] = -Rtemp.t()*ttemp;
  }

  cv::LevMarqSparse solver;
  solver.bundleAdjust(points,imagePoints,visibility,cams,Rvec,tvec,dist,criteria);

  for(int i=0;i<Rvec.size();i++){
      cv::Mat Rtemp = Rvec.at(i);
      cv::Mat ttemp = tvec.at(i);
      Rvec[i] = Rtemp.t();
      tvec[i] = -Rtemp.t()*ttemp;
  }

  error = 0.0;
  for(int i=0;i<nbImages;i++){
    std::vector<cv::Point2d> pts = Tools::projPoints(R.at(i),t.at(i),cameraMatrix,points);
    for(int j=0;j<nbCorrespondances;j++){
      if(imagePoints.at(j).camIndex==i){
        error += cv::norm(pts.at(imagePoints.at(j).ptIndex)-imagePoints.at(j).pt2d);
      }
    }
  }
  std::cout << "Final error : " << error << std::endl;

  R.clear();;
  t.clear();
  for(int i=0;i<Rvec.size();i++){
      R.push_back( Tools::copy( Rvec.at(i) ) );
      t.push_back( Tools::copy( tvec.at(i) ) );
  }
*/

}

void BundleAdjust::bundleAdjust( std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                   cv::Mat &cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter)
{
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, max_iter, DBL_EPSILON);

  int nbPoints = points.size();
  int nbImages = imagePoints.size();
  assert(visibility.size()==imagePoints.size());

  for(int i=0;i<nbImages;i++){
      assert(imagePoints.at(i).size()==nbPoints);
      assert(visibility.at(i).size()==nbPoints);
  }

  assert(R.size()==nbImages);
  assert(t.size()==nbImages);

  std::cout << "nbPoints : " << nbPoints << std::endl;
  std::cout << "nbImages : " << nbImages << std::endl;

  std::vector<cv::Mat> cams, dist;
  for(int i=0;i<nbImages;i++){
    cams.push_back(cameraMatrix);
    if(distCoeffs.cols>0 && distCoeffs.rows>0)
      dist.push_back(distCoeffs);
  }

  double error = 0.0;
  for(int i=0;i<nbImages;i++){
    std::vector<cv::Point2d> pts = Tools::projPoints(R.at(i),t.at(i),cameraMatrix,distCoeffs,points);
    for(int j=0;j<nbPoints;j++){
      if(visibility.at(i).at(j)){
        error += cv::norm(pts.at(j)-imagePoints.at(i).at(j));
      }
    }
  }
  std::cout << "Initial error : " << error << std::endl;

  std::vector<cv::Mat> Rvec,tvec;
  for(int i=0;i<R.size();i++){
      Rvec.push_back( Tools::copy( R.at(i) ) );
      tvec.push_back( Tools::copy( t.at(i) ) );
  }

  for(int i=0;i<Rvec.size();i++){
      cv::Mat Rtemp = Rvec.at(i);
      cv::Mat ttemp = tvec.at(i);
      Rvec[i] = Rtemp.t();
      tvec[i] = -Rtemp.t()*ttemp;
  }

  cv::LevMarqSparse solver;
  solver.bundleAdjust(points,imagePoints,visibility,cams,Rvec,tvec,dist,criteria);

  for(int i=0;i<Rvec.size();i++){
      cv::Mat Rtemp = Rvec.at(i);
      cv::Mat ttemp = tvec.at(i);
      Rvec[i] = Rtemp.t();
      tvec[i] = -Rtemp.t()*ttemp;
  }

  error = 0.0;
  for(int i=0;i<nbImages;i++){
      std::vector<cv::Point2d> pts = Tools::projPoints(Rvec.at(i),tvec.at(i),cameraMatrix,distCoeffs,points);
    for(int j=0;j<nbPoints;j++){
      if(visibility.at(i).at(j)){
        error += cv::norm(pts.at(j)-imagePoints.at(i).at(j));
      }
    }
  }
  std::cout << "Final error : " << error << std::endl;

  R.clear();;
  t.clear();
  for(int i=0;i<Rvec.size();i++){
      R.push_back( Tools::copy( Rvec.at(i) ) );
      t.push_back( Tools::copy( tvec.at(i) ) );
  }

}


void BundleAdjust::bundleAdjustOpenCV( std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                         cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter)
{
  assert(R.size()==t.size());
  int num_images = R.size();
  std::vector<cv::detail::ImageFeatures> features(num_images);
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> cameras;


  std::cout << "Set 3D points ... " << std::flush;
  for(int i=0;i<points.size();i++){
  }
  std::cout << "done" << std::endl;

  std::cout << "Set 2D points ... " << std::flush;
  for(int i=0;i<imagePoints.size();i++){
    for(int j=0;j<imagePoints.size();j++){
      cv::KeyPoint keyPoint;
      cv::Point2d pt = imagePoints.at(i).at(j);
      keyPoint.pt = cv::Point2f(pt.x,pt.y);
      features[i].keypoints.push_back( keyPoint );
    }
  }
  std::cout << "done" << std::endl;

  std::cout << "Set " << R.size() << " pose ... " << std::flush;
  for(int i=0;i<R.size();i++){
    cv::detail::CameraParams cam;
    cam.focal = cameraMatrix.at<double>(0,0);
    cam.aspect = cameraMatrix.at<double>(1,1)/cameraMatrix.at<double>(0,0);
    cam.ppx = cameraMatrix.at<double>(0,2);
    cam.ppy = cameraMatrix.at<double>(1,2);
    cam.R = R.at(i);
    cam.t = t.at(i);
    cameras.push_back(cam);
  }
  std::cout << "done" << std::endl;


  cv::Ptr<cv::detail::BundleAdjusterBase> adjuster;
  if(1)  adjuster = new cv::detail::BundleAdjusterReproj();
  else   adjuster = new cv::detail::BundleAdjusterRay();

  float conf_thresh = 1.f;
  adjuster->setConfThresh(conf_thresh);
  cv::Mat_<uchar> refine_mask = cv::Mat::zeros(3, 3, CV_8U);
  /*if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
  if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
  if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
  if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
  if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;*/
  adjuster->setRefinementMask(refine_mask);

  std::cout << "Start Bundle Adjustment ... " << std::flush;
  (*adjuster)(features, pairwise_matches, cameras);
  std::cout << "done" << std::endl;


}

void BundleAdjust::bundleAdjustLM(std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                                  cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter)
{

  float threshold2D;
  int maxLifeTime;

  MML *lm = new MML(0,0.9999,0.001);
  TmpMemory mem(lm, threshold2D, maxLifeTime);

  int nbPoints = points.size();
  int nbImages = imagePoints.size();

  lm->Init(nbPoints, nbImages);

  for(int i=0; i<R.size(); i++){
    tvmet::Matrix<flt,3,3> Rs = toTvMet33(R.at(i));
    tvmet::Matrix<flt,3,1> Ts = toTvMet31(t.at(i));
    tvmet::Matrix<flt,3,3> Ks = toTvMet33(cameraMatrix);

    lm->SetCameraInitValues(i, -Ts(0,0),-Ts(1,0), -Ts(2,0));
    lm->SetR0(i, Rs);
    lm->SetK(i, Ks);
    lm->Yji[i].clear();
  }
  std::vector< std::vector < std::pair<int,cv::Point2d> > > temp;

  int nCapPix = 0;
  for(int i=0; i<nbPoints; i++){
    lm->SetPointInitValues(i, points[i].x, points[i].y, points[i].z, 1.0);
    lm->SetNumProjCameras(i, temp[i].size());

    for(int j=0; j<temp[i].size(); j++){
      VectorCam<flt, 2> pt;
      pt.vector(0) = temp[i][j].second.x;
      pt.vector(1) = temp[i][j].second.y;
      lm->AddCapturedPixel(i, pt);
      lm->Yji[temp[i][j].first].push_back(std::pair<int,int>(i,j));

      nCapPix++;
    }
  }
  lm->SetNumCapPix(nCapPix);

  size_t it_interne;
  int prevNum2DPoints = 0;
    for (size_t it=1; it<=max_iter; it++) {
      std::cout<<std::endl<<"::iteration "<<it<<std::endl;
      if (it != 1) {
        mem.UpdateML();
      }
      if (lm->GetNum2DPoint() <= prevNum2DPoints && it >= 3) {
        std::cout<<"Number of inlier 2D points did not change."<<std::endl;
        break;
      }
      prevNum2DPoints = lm->GetNum2DPoint();
      std::cout<<"Number of inlier 3D points: "<<lm->GetNum3DPoint()<<std::endl;
      std::cout<<"Number of inlier 2D points: "<<lm->GetNum2DPoint()<<std::endl;
      lm->Run(it_interne);
      std::cout<<"Updating inliers."<<std::endl;
      mem.UpdatePoints();
    }
}
