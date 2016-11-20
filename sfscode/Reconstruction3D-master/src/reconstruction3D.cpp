#include "../include/reconstruction3D.h"

/** Create Reconstruction3D object
 */
Reconstruction3D::Reconstruction3D()
{
  srand(time(NULL));
  estimator = new Estimator(7);
}

/** Destroy Reconstruction3D object
 */
Reconstruction3D::~Reconstruction3D()
{
  delete estimator;
}

/** Get error
 * @param error error code to translate in comprehensible string
 * @return Comprehensible String Error
 */
std::string Reconstruction3D::getError(int error)
{
  switch(error){
  case NO_ERROR : return "NO_ERROR";
  case MAT_FUNDAMENTAL_ERROR: return "MAT_FUNDAMENTAL_ERROR";
  case MAT_ESSENTIAL_ERROR: return "MAT_ESSENTIAL_ERROR";
  case EXTRACT_EXTRINSIC_ERROR: return "EXTRACT_EXTRINSIC_ERROR";
  case FIND_EXTRINSIC_ERROR: return "FIND_EXTRINSIC_ERROR";
  case MAT_CONCAT_ERROR: return "MAT_CONCAT_ERROR";
  case TRIANGULATION_ERROR: return "TRIANGULATION_ERROR";
  case REPROJECTION_ERROR: return "REPROJECTION_ERROR";
  case NO_GOOD_POINTS_ERROR: return "NO_GOOD_POINTS_ERROR";
  case EXTRINSIC_NOT_FOUND: return "EXTRINSIC_NOT_FOUND";
  case UNKNOWN_CAMERA_PAIR: return "UNKNOWN_CAMERA_PAIR";
  case NOT_ENOUGH_POINTS: return "NOT_ENOUGH_POINTS";
  case UNNAMED_ERROR: return "UNNAMED_ERROR";
  default:
    return "UNKNOWN_ERROR";
  }
}

/** Get default parameters\n
 *   maxIter = 2000\n
 *   useRANSAC = true\n
 *   reprojThreshold = 1.0\n
 *   confidence = 0.99\n
 */
Reconstruction3D::Param Reconstruction3D::getDefaultParam()
{
  Param p;
  p.maxIter = 2000;
  p.useRANSAC = true;
  p.reprojThreshold = 1.0;
  p.confidence = 0.99;
  return p;
}

/** Compute 3D points estimation using two 2D points vector
 * @param pts1,pts2 are inputs 2D points vectors
 * @param cam1,cam2 are input camera description
 * @param pts3D is output 3D points vector
 * @param R,t are output rotation and translation matrices
 * @param error is the output error matrix, square of the real distance
 * @param p is input estimator param, use getDefaultParam() to initalize
 * @retval Error code
 */
int Reconstruction3D::compute(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param p)
{
  assert(pts1.size()==pts2.size());
  if(cam1.type==PERSP && cam2.type==PERSP)
    return computeWith2Persp(pts1,pts2,cam1,cam2,pts3D,R,t,error,p);
  if(cam1.type==OMNI && cam2.type==OMNI)
    return computeWith2Omni(pts1,pts2,cam1,cam2,pts3D,R,t,error,p);
  if(cam1.type==PERSP && cam2.type==OMNI){
    return computeWithPerspAndOmni(pts1,pts2,cam1,cam2,pts3D,R,t,error,p);
  }
  if(cam1.type==OMNI && cam2.type==PERSP){
    int res = computeWithPerspAndOmni(pts2,pts1,cam1,cam2,pts3D,R,t,error,p);
    //FIXME, rebuild 3D points
    t = -t;
    R = R.t();
    return res;
  }
  return UNKNOWN_CAMERA_PAIR;
}

/** Compute 3D points estimation using two 2D points vector using two perspective cameras
 * @param pts1, pts2 are inputs 2D points vectors
 * @param cam1,cam2 are input camera parameters
 * @param pts3D is output 3D points vector
 * @param R,t are output rotation and translation matrices
 * @param error is the output error matrix, square of the real distance
 * @param p is input estimator param, use getDefaultParam() to initalize
 * @retval Error code
 */
int Reconstruction3D::computeWith2Persp(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param p)
{
  cv::Mat E;
#if USE_ESSENTIAL_MATRIX
  std::cout << "Compute Essential Matrix" << std::endl;
  if(computeEssentialMatrix(pts1,pts2,cam1,cam2,E,error,p))
    return MAT_ESSENTIAL_ERROR;
#else
  std::cout << "Compute Fundamental Matrix" << std::endl;
  cv::Mat F;
  if(findFundamentalMat(pts1,pts2,F,error,p))
    return MAT_FUNDAMENTAL_ERROR;
  if(computeEssentialMatrix(F,cam1.K,cam2.K,E))
    return MAT_ESSENTIAL_ERROR;
#endif

  int nb_good=0;
  std::vector<bool> good = getMask(error,p.reprojThreshold*p.reprojThreshold);

  for(int i=0;i<good.size();i++)
    if(good.at(i))
      nb_good++;

  std::cout << "Use " << nb_good << " points" << std::endl;

  if(nb_good==0){
    return NO_GOOD_POINTS_ERROR;
  }

  std::vector<cv::Mat> Rvec,tvec;
  if(extractExtrinsicParametersFromEssential(E,Rvec,tvec))
    return EXTRACT_EXTRINSIC_ERROR;

  if(selectExtrinsicParameters(pts1,pts2,cam1,cam2,Rvec,tvec,R,t,pts3D,good))
    return EXTRINSIC_NOT_FOUND;

  return NO_ERROR;
}

/** Compute 3D points estimation using two 2D points vector using two omni cameras
 * @param pts1, pts2 are inputs 2D points vectors
 * @param cam1,cam2 are input camera parameters
 * @param pts3D is output 3D points vector
 * @param R,t are output rotation and translation matrices
 * @param error is the output error matrix, square of the real distance
 * @param p is input estimator param, use getDefaultParam() to initalize
 * @retval Error code
 */
int Reconstruction3D::computeWith2Omni(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param p)
{
  std::cout << "computeWith2Omni()" << std::endl;
  //Compute fundamental matrix (if uncalibred)
  //Compute essential matrix
  cv::Mat E;
  std::vector<bool> good = getMask(error,p.reprojThreshold*p.reprojThreshold);

  std::vector<cv::Point2d> pts1n, pts2n;
  normalize(pts1,cam1.K,pts1n);
  normalize(pts2,cam2.K,pts2n);

  std::vector<cv::Point3d> m1b,m2b;
  for(int i=0;i<pts1n.size();i++){
    double x = pts1n.at(i).x;
    double y = pts1n.at(i).y;
    double gamma = sqrt(1+(1-cam1.epsilon*cam1.epsilon)*(x*x+y*y));
    double nu = (-gamma-cam1.epsilon*(x*x+y*y))/(cam1.epsilon*cam1.epsilon*(x*x+y*y)-1);

    double z = 1/(1+cam1.epsilon*nu);
    m1b.push_back( cv::Point3d(x,y,z) );
  }
  for(int i=0;i<pts2n.size();i++){
    double x = pts2n.at(i).x;
    double y = pts2n.at(i).y;
    double gamma = sqrt(1+(1-cam2.epsilon*cam2.epsilon)*(x*x+y*y));
    double nu = (-gamma-cam2.epsilon*(x*x+y*y))/(cam2.epsilon*cam2.epsilon*(x*x+y*y)-1);

    double z = 1/(1+cam2.epsilon*nu);
    m2b.push_back( cv::Point3d(x,y,z) );
  }
  cv::Mat m1 = toMat(m1b);
  cv::Mat m2 = toMat(m2b);

  //std::cout << m1 << std::endl;
  //cv::Mat m1 = toHomogeneous(m1b);
  //cv::Mat m2 = toHomogeneous(m2b);

  std::vector<cv::Mat> vE;
  int res = estimator->runNPoint2(m1,m2,E);
  std::cout << res << " solutions" << std::endl;

  for(int i=0;i<res;i++){
    //E = vE.at(i);
    display(E,"E");

    cv::SVD svd(E);
    display(svd.w,"W");

    //Compute extrinsic parameters
    std::vector<cv::Mat> Rvec,tvec;
    if(extractExtrinsicParametersFromEssential(E,Rvec,tvec))
      return EXTRACT_EXTRINSIC_ERROR;

    /*for(int i=0;i<Rvec.size();i++){
      for(int j=0;j<tvec.size();j++){
        cv::Mat Rt;
        cv::Rodrigues(Rvec.at(i),Rt);
        std::cout << "Pose " << i<<"-"<<j << std::endl;
        display(Rt,"R");
        display(tvec.at(j),"t");
      }
    }*/

    //Compute triangulation
    if(selectExtrinsicParameters(pts1,pts2,cam1,cam2,Rvec,tvec,R,t,pts3D,good))
      return EXTRINSIC_NOT_FOUND;

    std::cout << std::endl;
  }
  return NO_ERROR;
}

/** Compute 3D points estimation using two 2D points vector using one perspective and one omnidirectional
 * @param pts1, pts2 are inputs 2D points vectors
 * @param cam1,cam2 are input camera parameters
 * @param pts3D is output 3D points vector
 * @param R,t are output rotation and translation matrices
 * @param error is the output error matrix, square of the real distance
 * @param p is input estimator param, use getDefaultParam() to initalize
 * @retval Error code
 */
int Reconstruction3D::computeWithPerspAndOmni(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param p)
{
  std::cout << "computeWithPerspAndOmni()" << std::endl;
  int count = MIN(pts1.size(),pts2.size());

  //Compute fundamental matrix : qp.t() * Fpc * q^c = 0, q^c = [x²+y²;x;y;1], Fpc = Matrix 3x4
  cv::Mat A(count,12,CV_64F);
  for(int i=0;i<count;i++){
    cv::Mat p1 = toHomogeneous(pts1.at(i));
    cv::Mat p2 = liftCoordinate(pts2.at(i));
    cv::Mat p = kron(p1,p2);
    p.copyTo(A.row(i));
  }
  cv::Mat F = rightNullVector(A);
  F = F.reshape(0,3);
  display(F,"F");

  //Compute essential matrix : DIRECT-E is better
  //Compute extrinsic parameters
  //Compute triangulation
  return UNNAMED_ERROR;
}

/** Compute the Essential matrix from 2D points
 * @param pts1,pts2 are input 2D points lists
 * @param cam1,cam2 are input camera model
 * @param E is the output Essential matrix
 * @param error is the output error matrix, square of the real distance
 * @param p is input estimator param, use getDefaultParam() to initalize
 * @retval Error code
 */
int Reconstruction3D::computeEssentialMatrix( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, Camera &cam1, Camera &cam2, cv::Mat &E, cv::Mat &error, Param p)
{
  assert(pts1.size()==pts2.size());
  std::vector<cv::Point2d> pts1n, pts2n;
  normalize(pts1,cam1.K,pts1n);
  normalize(pts2,cam2.K,pts2n);
#if 0
  int itermax = p.maxIter;

  for(int iter=0;iter<itermax;iter++){
    std::vector<cv::Point2d> pts1_t, pts2_t;
    getSubset(pts1,pts1_t,pts2,pts2_t,5);

    cv::Mat Etemp;

    std::vector<cv::Mat> Rvectemp,tvectemp;
    extractExtrinsicParametersFromEssential(Etemp,Rvectemp,tvectemp);

    //TODO
  }
#endif
  return findFundamentalMat(pts1n,pts2n,E,error,p);
}

/** Compute the Essential matrix from Fundamental matrix and projection ones
 * @param F is the input Fundamental matrix
 * @param K1,K2 are the input projection matrices
 * @param E is the output Essential matrix
 * @retval Error code
 */
int Reconstruction3D::computeEssentialMatrix(cv::Mat &F, cv::Mat &K1, cv::Mat &K2, cv::Mat &E)
{
  E = K1.t()*F*K2;
  return NO_ERROR;
}

/** Compute the Essential matrix from rotation and translation matrices
 * @param R,t are the input matrices
 * @param E is the output Essential matrix
 * @retval Error code
 */
int Reconstruction3D::computeEssentialMatrix(cv::Mat &R, cv::Mat &t, cv::Mat &E)
{
  E = skewSymetric(t) * R;
  return NO_ERROR;
}

/** Extract Rotation matrix and Translation vector from Fondamental matrix
 * @param F is the input fondamental matrix
 * @param R is the output rotation matrix
 * @param t is the output translation matrix
 * @retval Error code
 */
int Reconstruction3D::extractExtrinsicParametersFromFondamental(cv::Mat &F, cv::Mat &R, cv::Mat &t)
{
  cv::SVD svd(F);
#if 1
  t = svd.u.col(2);
#else
  t = svd.vt.col(2);
#endif
  cv::Mat ex;
  skewSymetric(t,ex);
  multiply(ex,F,R);
  return NO_ERROR;
}

/** Extract Rotation matrix and Translation vector from Essential matrix
 * @param E is the input essential matrix
 * @param R is the output rotation matrix
 * @param t is the output translation matrix
 * @retval Error code
 */
int Reconstruction3D::extractExtrinsicParametersFromEssential(cv::Mat &E, cv::Mat &R, cv::Mat &t)
{
  //E = u*w*vt = U*D*Vt
  cv::SVD svd(E);
  double w_[9] = {0,1,0, -1,0,0, 0,0,1};
  cv::Mat W = cv::Mat(3,3,CV_64F,w_);

  //Compute R
#if 0
  multiply(svd.u,W,svd.vt,R);
#else
  cv::Mat Wt = W.t();
  multiply(svd.u,Wt,svd.vt,R);
#endif
  if(cv::determinant(R)<0) R = -R;

  //Compute t
  t = svd.u.col(2).clone();
#if 0
  t = -t;
#endif

  return NO_ERROR;
}

/** Extract Rotation matrix and Translation vector from Essential matrix
 * @param E is the input essential matrix
 * @param Rvec is an output vector of rotation matrix
 * @param tvec is an output vector of translation matrix
 * @retval Error code
 */
int Reconstruction3D::extractExtrinsicParametersFromEssential(cv::Mat &E, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec)
{
  //E = [t]x R
  //E = u*w*vt = U*D*Vt
  cv::SVD svd(E);
  //std::cout << svd.w << std::endl;
  double w1_[9] = {0,1,0, -1,0,0, 0,0,1};
  double w2_[9] = {0,-1,0, 1,0,0, 0,0,1};
  cv::Mat W1 = cv::Mat(3,3,CV_64F,w1_);
  cv::Mat W2 = cv::Mat(3,3,CV_64F,w2_);

  Rvec.clear();
  tvec.clear();

  cv::Mat R1,R2,t1,t2;
  //Compute R
  R1 = svd.u*W1*svd.vt;
  R2 = svd.u*W1.t()*svd.vt;
  //multiply(svd.u,Wt,svd.vt,R2);
  if(cv::determinant(R1)<0) {R1 = -svd.u*W2*svd.vt;     /*std::cout << "Correct R1" << std::endl;*/}
  if(cv::determinant(R2)<0) {R2 = -svd.u*W2.t()*svd.vt; /*std::cout << "Correct R2" << std::endl;*/}
  if(cv::determinant(R1)<0) {std::cout << "Bad R1 " << cv::determinant(R1) << std::endl;}
  if(cv::determinant(R2)<0) {std::cout << "Bad R2 " << cv::determinant(R2) << std::endl;}
  //display(R1,"R1");
  //display(R2,"R2");
  Rvec.push_back(R1);
  Rvec.push_back(R2);

  //Compute t
#if 1
  t1 = svd.u.col(2).clone();
#else
  double z_[9] = {0,-1,0, 1,0,0, 0,0,0};
  cv::Mat Z = cv::Mat(3,3,CV_64F,z_);
  cv::Mat temp = svd.vt.t()*Z*svd.vt;
  t1 = cv::Mat(3,1,CV_64F);
  Tools::set(t1,0,0, get(temp,2,1));
  Tools::set(t1,1,0, get(temp,0,2));
  Tools::set(t1,2,0, get(temp,1,0));
#endif
  t2 = -t1;
  tvec.push_back(t1);
  tvec.push_back(t2);
  return NO_ERROR;
}

/** Select the good R and t pair between the four solutions
 * @param pts1,pts2 are input 2D points lists
 * @param cam1,cam2 are input camera models
 * @param Rvec,tvec are input extrinsic parameters lists
 * @param R,t are output selected extrinsic parameters
 * @param pts3D is output 3D points list
 * @param mask is the input mask
 */
int Reconstruction3D::selectExtrinsicParameters(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D, std::vector<bool> mask)
{
  cv::Mat I3 = cv::Mat::eye(3,3,CV_64F);
  cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F);
  //cv::Mat Rx = toRotationMatrix(-M_PI/2,0,0);

  cv::Mat R0 = I3.clone();;
  //if(cam1.type==OMNI){
  //  R0 = Rx.clone();
  //}else{
  //  R0 = I3.clone();
  //}

  //Compute retroproj error
  bool found = false;
  double errormin = 1e20;
  for(unsigned int i=0;i<tvec.size();i++){
    for(unsigned int j=0;j<Rvec.size();j++){
      std::vector<cv::Point3d> p3;
      cv::Mat Rtemp = Rvec.at(j).clone().t();
      cv::Mat ttemp = -Rtemp*tvec.at(i).clone();
      std::vector<cv::Point2d> ptsr1,ptsr2;

      display(Rodrigues(Rtemp),"Rtest");
      display(ttemp,"ttest");

      if(cam1.type==OMNI){
        Rtemp = R0 * Rtemp;
        ttemp = R0 * ttemp;
      }

      //Compute triangulation
      triangulate(pts1,pts2,cam1,cam2,Rtemp,ttemp,p3,R0,t0);

      ptsr1 = getRetroprojPoints(p3,cam1,R0,t0);
      ptsr2 = getRetroprojPoints(p3,cam2,Rtemp,ttemp);
      //ptsr2 = getRetroprojPoints(p3,cam2,Rtemp.t(),-Rtemp.t()*ttemp);
      std::cout << "Error 1: " << reprojError(pts1,ptsr1,mask) << std::endl;
      std::cout << "Error 2: " << reprojError(pts2,ptsr2,mask) << std::endl;
      double temp_error = reprojError(pts1,ptsr1,mask) + reprojError(pts2,ptsr2,mask);

      bool test_point = false;
      int k=0;
      if(mask.size()==pts1.size() && cam1.type==PERSP && cam2.type==PERSP){
        test_point = true;
        while(mask.at(k)==0) k++;
      }

      if(test_point){
        cv::Point3d P = p3.at(k);
        if( P.z>0 && temp_error<errormin ){
          cv::Point3d Q = projPoint(Rtemp.t(),-Rtemp.t()*ttemp,P);
          //cv::Point3d Q = projPoint(Rtemp,ttemp,P);
          if( Q.z>0 ){
            errormin = temp_error;
            R = Rtemp.clone();
            t = ttemp.clone();
            pts3D = p3;
            found = true;
            std::cout << "Good ("<<errormin<<") after testing" << std::endl;
          }
        }
      }else{
        if(temp_error<errormin){
          errormin = temp_error;
          R = Rtemp.clone();
          t = ttemp.clone();
          pts3D = p3;
          found = true;
          std::cout << "Good ("<<errormin<<")" << std::endl;
        }
      }
      std::cout << std::endl;
    }
  }

  if(!found)
    return EXTRINSIC_NOT_FOUND;

  return NO_ERROR;

}

/** Compute the position of 3D points
 * @param pts1, pts2 are inputs 2D points vectors
 * @param cam1,cam2 are input camera models
 * @param R is the input rotation matrix
 * @param t is the input translation matrix
 * @param R0 is the optional input rotation matrix of first camera
 * @param t0 is the optional input translation matrix of first camera
 * @param pts3D is output 3D points vector
 * @retval Error code
 */
int Reconstruction3D::triangulate(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera cam1, Camera cam2, cv::Mat R, cv::Mat t, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0)
{
  assert(pts1.size()!=0 && pts2.size()!=0);

  if(cam1.type==PERSP && cam2.type==PERSP) {
    return triangulate(pts1,pts2,cam1.K,cam2.K,R,t,pts3D,R0,t0);
  }
  if(cam1.type==OMNI && cam2.type==OMNI){
    std::cout << "Triangulate with two omni" << std::endl;
    pts3D.clear();
    std::vector<cv::Point2d> pts1n, pts2n;
    normalize(pts1,pts2,cam1.K,cam2.K,pts1n,pts2n);

    //FILE *fsphere1 = fopen("PTS_SPHERE1.txt","w");
    //FILE *fsphere2 = fopen("PTS_SPHERE2.txt","w");


    for(int i=0;i<pts1.size();i++){
      //Projection center
      cv::Point3d O1 = toPoint3d(t0);
      cv::Point3d O2 = toPoint3d(t);

      //Point one sphere
      cv::Point3d X1 = projectOnSphere(pts1n.at(i),R0,t0,cam1);
      cv::Point3d X2 = projectOnSphere(pts2n.at(i),R ,t ,cam2);

      //fprintf(fsphere1,"%f %f %f\n",X1.x,X1.y,X1.z);
      //fprintf(fsphere2,"%f %f %f\n",X2.x,X2.y,X2.z);

      //std::cout << "X1 : " << cv::norm(X1-O1) << std::endl;
      //std::cout << "X2 : " << cv::norm(X2-O2) << std::endl;
      //Line intersection
      cv::Point3d M = linesIntersection(O1,X1,O2,X2);
      pts3D.push_back(M);
      //exit(9);
    }

    //fclose(fsphere1);
    //fclose(fsphere2);
    return NO_ERROR;
  }
  if(cam1.type==PERSP && cam2.type==OMNI){
    //TODO, triangulate with 1 persp and 1 omni
  }
  if(cam1.type==OMNI && cam2.type==PERSP){
    //TODO, triangulate with 1 omni and 1 persp
  }
  return UNKNOWN_CAMERA_PAIR;
}

/** Compute the position of 3D points
 * @param pts1, pts2 are inputs 2D points vectors
 * @param K1,K2 are input projection matrix
 * @param R is the input rotation matrix
 * @param t is the input translation matrix
 * @param R0 is the optional input rotation matrix of first camera
 * @param t0 is the optional input translation matrix of first camera
 * @param pts3D is output 3D points vector
 * @retval Error code
 */
int Reconstruction3D::triangulate(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat K1, cv::Mat K2, cv::Mat R, cv::Mat t, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0)
{
  std::vector<cv::Point2d> pts1n, pts2n;
  normalize(pts1,pts2,K1,K2,pts1n,pts2n);
  return triangulate(pts1n,pts2n,R,t,pts3D,R0,t0);
}

/** Compute the position of 3D points
 * @param pt1, pt2 are inputs normalized 2D points vectors
 * @param K1,K2 are input projection matrix
 * @param R is the input rotation matrix
 * @param t is the input translation matrix
 * @param R0 is the optional input rotation matrix of first camera
 * @param t0 is the optional input translation matrix of first camera
 * @param Q is output 3D points vector
 * @retval Error code
 */
int Reconstruction3D::triangulate(cv::Point2d pt1, cv::Point2d pt2, cv::Mat K1, cv::Mat K2, cv::Mat R, cv::Mat t, cv::Point3d &Q, cv::Mat R0, cv::Mat t0)
{
  std::vector<cv::Point2d> pts1,pts2;
  pts1.push_back(pt1);
  pts2.push_back(pt2);
  std::vector<cv::Point3d> pts3D;
  int result = triangulate(pts1,pts2,K1,K2,R,t,pts3D,R0,t0);
  Q = pts3D.at(0);
  return result;
}

/** Compute the position of 3D points
 * @param pt1n, pt2n are inputs normalized 2D points vectors
 * @param R is the input rotation matrix
 * @param t is the input translation matrix
 * @param R0 is the optional input rotation matrix of first camera
 * @param t0 is the optional input translation matrix of first camera
 * @param Q is output 3D points vector
 * @retval Error code
 */
int Reconstruction3D::triangulate(cv::Point2d pt1n, cv::Point2d pt2n, cv::Mat R, cv::Mat t, cv::Point3d &Q, cv::Mat R0, cv::Mat t0)
{
  std::vector<cv::Point2d> pts1,pts2;
  pts1.push_back(pt1n);
  pts2.push_back(pt2n);
  std::vector<cv::Point3d> pts3D;
  int result = triangulate(pts1,pts2,R,t,pts3D,R0,t0);
  Q = pts3D.at(0);
  return result;
}

/** Compute the position of 3D points
 * @param pts1n, pts2n are inputs normalized 2D points vectors
 * @param R is the input rotation matrix
 * @param t is the input translation matrix
 * @param R0 is the optional input rotation matrix of first camera
 * @param t0 is the optional input translation matrix of first camera
 * @param pts3D is output 3D points vector
 * @retval Error code
 */
int Reconstruction3D::triangulate(std::vector<cv::Point2d> &pts1n, std::vector<cv::Point2d> &pts2n, cv::Mat R, cv::Mat t, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0)
{   
  //std::cout << "triangulate" << std::endl;
  int numPoints = pts1n.size();
#if 0
  cv::Mat P1 = concatH( R0, t0 );
  cv::Mat P2 = concatH( R, t );
#else
  cv::Mat P1 = concatH( R0.t(), -R0.t()*t0 );
  cv::Mat P2 = concatH( R.t(), -R.t()*t );
#endif

#if 1
  pts3D.clear();
  cv::Mat pts4D;
  cv::Mat pts1mat = toMat(pts1n).t();
  cv::Mat pts2mat = toMat(pts2n).t();
  cv::triangulatePoints(P1,P2,pts1mat,pts2mat,pts4D);

  for(int i=0;i<numPoints;i++){
    cv::Point3d Q( get(pts4D,0,i), get(pts4D,1,i), get(pts4D,2,i));
    double scale = get(pts4D,3,i);
    Q *= 1.0/scale;
    pts3D.push_back(Q);
  }
  return NO_ERROR;
#else
  cv::Mat A = cv::Mat(6,4,CV_64F);

  // Solve system for each point
  int i,j;
  for( i = 0; i < numPoints; i++ ){
    //Fill matrix for current point
    for( j = 0; j < 2; j++ ){
      double x,y;
      if(j==0){
        x = pts1n.at(i).x;
        y = pts1n.at(i).y;
        for( int k = 0; k < 4; k++ ){
          A.at<double>(j*3+0, k) = x * P1.at<double>(2,k) -     P1.at<double>(0,k) ;
          A.at<double>(j*3+1, k) = y * P1.at<double>(2,k) -     P1.at<double>(1,k) ;
          A.at<double>(j*3+2, k) = x * P1.at<double>(1,k) - y * P1.at<double>(0,k) ;
        }
      }else{
        x = pts2n.at(i).x;
        y = pts2n.at(i).y;
        for( int k = 0; k < 4; k++ ){
          A.at<double>(j*3+0, k) = x * P2.at<double>(2,k) -     P2.at<double>(0,k) ;
          A.at<double>(j*3+1, k) = y * P2.at<double>(2,k) -     P2.at<double>(1,k) ;
          A.at<double>(j*3+2, k) = x * P2.at<double>(1,k) - y * P2.at<double>(0,k) ;
        }
      }
    }
    // Solve system for current point
    cv::SVD svd(A);
    double scale = svd.vt.at<double>(3,3);
    cv::Point3d Q(svd.vt.at<double>(3,0)/scale,svd.vt.at<double>(3,1)/scale,svd.vt.at<double>(3,2)/scale);
    pts3D.push_back(Q);
  }
  return NO_ERROR;
#endif
}

/** Compute the position of 3D points
 * @param ptsp, ptsc are inputs normalized 2D points vectors
 * @param Rp,Rc are the input rotation matrices
 * @param tp,tc are the input translation matrices
 * @param Kp is the input calibration matrix of perspective camera
 * @param Bc is the inpur define the omnidirectional camera ray projection
 * @param pts3D is output 3D points vector
 * @retval Error code
 */
int Reconstruction3D::triangulate(std::vector<cv::Point2d> &ptsp, std::vector<cv::Point2d> &ptsc, cv::Mat &Rp, cv::Mat &tp, cv::Mat &Rc, cv::Mat &tc, cv::Mat &Kp, cv::Mat &Bc, std::vector<cv::Point3d> &pts3D)
{
  pts3D.clear();
  int count = MIN(ptsp.size(),ptsc.size());
  cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F);
  cv::Mat I3 = cv::Mat::eye(3,3,CV_64F);
  for(int i=0;i<count;i++){
    cv::Mat A = tp.clone();
    cv::Mat B = tc.clone();
    cv::Mat temp1,temp2;

    cv::Mat qp(3,1,CV_64F);
    qp.at<double>(0,0) = ptsp.at(i).x;
    qp.at<double>(1,0) = ptsp.at(i).y;
    qp.at<double>(2,0) = 1.0;

    temp1 = Rp.t() * Kp.inv() * qp;
    //multiply(Rpt,Ki,qp,temp);

    concatHorizontal(A,temp1,A);
    concatHorizontal(A,t0,A);
    concatHorizontal(A,I3,A);

    cv::Mat qc(4,1,CV_64F);
    qc.at<double>(0,0) = ptsc.at(i).x*ptsc.at(i).x + ptsc.at(i).y*ptsc.at(i).y;
    qc.at<double>(1,0) = ptsc.at(i).x;
    qc.at<double>(2,0) = ptsc.at(i).y;
    qc.at<double>(3,0) = 1.0;

    temp2 = Rc.t() * Bc * qc;
    //multiply(Rct,Bc,qc,temp2);

#if 0 //TODO, test if ok <= no change
    double s = get(temp2,2,0);
    temp2.at<double>(0,0) /= s;
    temp2.at<double>(1,0) /= s;
    temp2.at<double>(2,0) /= s;
#endif
    concatHorizontal(B,t0,B);
    concatHorizontal(B,temp2,B);
    concatHorizontal(B,I3,B);

    cv::Mat C;
    concatVertical(A,B,C);
    //display(C,"C");

    cv::SVD svd(C/*,cv::SVD::FULL_UV*/);

    cv::Mat result = svd.vt.row(svd.vt.rows-1).clone().t();
    cv::Point3d Q;
#if 1
    double scale = 1.0 * sign( get(result,5,0) );
#else
    double scale = -1.0;
#endif
    Q.x = scale*result.at<double>(3,0);
    Q.y = scale*result.at<double>(4,0);
    Q.z = scale*result.at<double>(5,0);
    pts3D.push_back(Q);
  }
  return NO_ERROR;
}

/** Compute the retroprojection error
 * @param pts1,pts2 are the inputs points vectors
 * @param mask is an optional input to select points to compute
 * @retval sum( sqrt( (x1-x2)²+(y1-y2)² ) )
 */
double Reconstruction3D::reprojError(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, std::vector<bool> mask)
{
  assert(mask.size()==0 || (mask.size()==pts1.size() && mask.size()==pts2.size()));
  bool useMask = true;
  if(mask.size()!=pts1.size()) useMask=false;
  double error = 0.0;
  for(unsigned int i=0;i<pts1.size();i++){
    bool check = true;
    if(useMask)
      if(!mask.at(i))
	check = false;
    if(check){
      double x = pts1.at(i).x - pts2.at(i).x;
      double y = pts1.at(i).y - pts2.at(i).y;
      error += sqrt(x*x+y*y);
    }
  }
  return error;
}

/** Compute the retroprojection error
 * @param pt1,pt2 are the inputs points
 * @retval sqrt( (x1-x2)²+(y1-y2)² )
 */
double Reconstruction3D::reprojError(cv::Point2d pt1, cv::Point2d pt2)
{
  double x = pt1.x - pt2.x;
  double y = pt1.y - pt2.y;
  return sqrt(x*x+y*y);
}

/** Compute Bundle Adjustment\n
 * \f$m=P.M\f$
 */
int Reconstruction3D::bundleAdjustment(std::vector< std::vector<cv::Point2d> > &imagesPoints, std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec, cv::Mat K, int max_iter)
{

  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, max_iter, DBL_EPSILON);

  std::vector< std::vector<cv::Point2d> > imagesPoints_bak = imagesPoints;
  int nbPoints = points3D.size();
  int nbImages = imagesPoints.size();

  std::cout << "nbPoints : " << nbPoints << std::endl;
  std::cout << "nbImages : " << nbImages << std::endl;

  std::vector<cv::Mat> cams, dist;
  for(int i=0;i<nbImages;i++){
    cams.push_back(K);
    //cv::Mat d = cv::Mat::zeros(5,1,CV_64F);
    //dist.push_back(d);
  }

  std::vector< std::vector<int> > visibility;
  for(int i=0;i<nbImages;i++){
    std::vector<int>  visibility_temp;
    for(int j=0;j<nbPoints;j++){
      visibility_temp.push_back(1);
    }
    visibility.push_back(visibility_temp);
  }

  std::vector<cv::Mat> Rtemp, ttemp;
  for(int i=0;i<nbImages;i++){
    Rtemp.push_back(Rvec.at(i).t());
    ttemp.push_back(-tvec.at(i));
  }

  cv::LevMarqSparse solver;
  solver.bundleAdjust(points3D,imagesPoints_bak,visibility,cams,Rtemp,ttemp,dist,criteria);

  for(int i=0;i<nbImages;i++){
    Rvec[i] = Rtemp.at(i).t();
    tvec[i] = -ttemp.at(i);
  }

  /*for(int i=0;i<Rvec.size();i++){
    std::cout << Rvec.at(i) << std::endl;
    }
    for(int i=0;i<tvec.size();i++){
    std::cout << tvec.at(i) << std::endl;
    }
    for(int i=0;i<cams.size();i++){
    std::cout << cams.at(i) << std::endl;
    }*/

}

/** Compute Bundle Adjustment\n
 * \f$m=P.M\f$
 */
int Reconstruction3D::bundleAdjustment_bak(std::vector< std::vector<cv::Point2d> > &imagesPoints, std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> /*Rvec*/, std::vector<cv::Mat> /*tvec*/, cv::Mat K)
{
  int nbPoints = points3D.size();
  int nbImages = imagesPoints.size();

  std::cout << "nbPoints : " << nbPoints << std::endl;
  std::cout << "nbImages : " << nbImages << std::endl;

  cv::Mat m = cv::Mat::zeros(3*nbImages,nbPoints,CV_64F);

  //Initialize lambdas
  std::vector< std::vector<double> > lambdas;
  for(int nim=0;nim<nbImages;nim++){
    std::vector<double> l;
    for(int npt=0;npt<nbPoints;npt++){
      l.push_back(1.0);
    }
    lambdas.push_back(l);
  }

  double lasterror = 1e20;
  double error_max = 1.0;
  cv::Mat P,M;

  for(int k=0;k<100;k++){
    //Create matrix m
    for(int nim=0;nim<nbImages;nim++){
      for(int npt=0;npt<nbPoints;npt++){
	cv::Point2d pt = imagesPoints.at(nim).at(npt);
	double lambda = lambdas.at(nim).at(npt);
	Tools::set(m,3*nim  ,npt,   lambda*pt.x   );
	Tools::set(m,3*nim+1,npt,   lambda*pt.y   );
	Tools::set(m,3*nim+2,npt,   lambda        );
      }
    }

    //display(m,"m");
    cv::SVD PM(m);
    //display(PM.w,"W");
    double error = get(PM.w,4,0);


    cv::Mat UW = PM.u * cv::Mat::diag(PM.w);
    P = concatH(UW.col(0),UW.col(1),UW.col(2),UW.col(3));
    M = concatV(PM.vt.row(0),PM.vt.row(1),PM.vt.row(2),PM.vt.row(3));

    for(int i=0;i<M.cols;i++)
      M.col(i) /= get(M,3,i);

    std::cout << "Error : " << error << std::endl;
    if(error<error_max)
      break;
    if(lasterror-error<1e-5)
      break;
    lasterror = error;

    for(int nim=0;nim<nbImages;nim++){
      cv::Mat Pi(3,4,CV_64F);
      for(int i=0;i<3;i++)
        for(int j=0;j<4;j++)
          Tools::set(Pi,i,j, get(P,nim*3+i,j));

      for(int npt=0;npt<nbPoints;npt++){
	cv::Mat mi(3,1,CV_64F);
	Tools::set(mi,0,0, imagesPoints.at(nim).at(npt).x );
	Tools::set(mi,1,0, imagesPoints.at(nim).at(npt).y );
	Tools::set(mi,2,0, 1.0 );
	cv::Mat Mi = M.col(npt);
	lambdas[nim][npt] = getLambda(mi,Pi,Mi);
      }
    }
  }

  for(int nim=0;nim<nbImages;nim++){
    cv::Mat Pi(3,4,CV_64F);
    for(int i=0;i<3;i++)
      for(int j=0;j<4;j++)
        Tools::set(Pi,i,j, get(P,nim*3+i,j));
    display(K.inv()*Pi,"Pi");
  }

  return NO_ERROR;
}

double Reconstruction3D::getLambda(cv::Mat m, cv::Mat P, cv::Mat M)
{
  cv::Mat res = P*M;
  res /= get(res,2,0);
  /*display(res,"res");
    display(m,"m");
    std::cout << get(m,0,0)/get(res,0,0) << std::endl;
    std::cout << get(m,1,0)/get(res,1,0) << std::endl;*/
  return (get(res,0,0)/get(m,0,0) + get(res,1,0)/get(m,1,0))/2.0;
}


cv::Mat _K;

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

  CvMat _dpdr, _dpdt;

  cvGetCols( A, &_dpdr, 0, 3 );
  cvGetCols( A, &_dpdt, 3, 6 );
  //cvGetCols( A, &_dpdf, 6, 8 );
  //cvGetCols( A, &_dpdc, 8, 10 );

  cvProjectPoints2(&_Mi, &_ri, &_ti, &_A, NULL, _mp, &_dpdr, &_dpdt,
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


  cvProjectPoints2( &_Mi, &_ri, &_ti, &_A, NULL, _mp, NULL, NULL,
                    NULL, NULL, NULL, 0);

  _mp2->data.db[0] = _mp->data.db[0];
  _mp2->data.db[1] = _mp->data.db[1];
  cvTranspose( _mp2, estim );
  cvReleaseMat( &_mp );
  cvReleaseMat( &_mp2 );
}

void my_fjac_new(int i, int j, cv::Mat& point_params, cv::Mat& cam_params, cv::Mat& A, cv::Mat& B, void* data) {
  CvMat _point_params = point_params, _cam_params = cam_params, _Al = A, _Bl = B;
  my_fjac(i,j, &_point_params, &_cam_params, &_Al, &_Bl, data);
}

void my_func_new(int i, int j, cv::Mat& point_params, cv::Mat& cam_params, cv::Mat& estim, void* data)  {
  CvMat _point_params = point_params, _cam_params = cam_params, _estim = estim;
  my_func(i,j,&_point_params,&_cam_params,&_estim,data);
}

/** Compute Bundle Adjustment\n
 * \f$m=P.M\f$
 */
int Reconstruction3D::bundleAdjustment2(std::vector< std::vector<cv::Point2d> > &imagesPoints, std::vector<cv::Point3d> &points3D, std::vector< std::vector<int> > visibility, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec, cv::Mat K, int max_iter)
{
  _K = K.clone();
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, max_iter, DBL_EPSILON);

  int num_cameras = (int)imagesPoints.size();
  int num_points = (int)points3D.size();
  int num_cam_param = 3 /* rotation vector */ + 3 /* translation vector */;
  int num_point_param = 3;

  assert(num_cameras==Rvec.size());
  assert(num_cameras==tvec.size());
  assert(num_cameras==visibility.size());
  for(int i=0;i<num_cameras;i++){
    assert( imagesPoints.at(i).size() == num_points );
    assert( visibility.at(i).size() == num_points );
  }

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
  cv::Mat _points(points3D);
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
        cv::Point2d p = imagesPoints[j][i];
        ((double*)(X.data))[counter] = p.x;
        ((double*)(X.data))[counter+1] = p.y;
        assert(p.x != -1 || p.y != -1);
        counter+=2;
      }
    }
  }

  cv::LevMarqSparse levmar( num_points, num_cameras, num_point_param, num_cam_param, 2, vismat, params, X,
			    cv::TermCriteria(criteria), my_fjac_new, my_func_new, NULL,
			    NULL, NULL);

  std::cout << "Error norm: " << levmar.errNorm << std::endl;
  //extract results
  //fill point params
  /*Mat final_points(num_points, 1, CV_64FC3,
    levmar.P->data.db + num_cameras*num_cam_param *levmar.P->step);
    CV_Assert(_points.size() == final_points.size() && _points.type() == final_points.type());
    final_points.copyTo(_points);*/

  points3D.clear();
  CvMat *point_mat = cvCreateMat(3,1,CV_64F);
  for( int i = 0; i < num_points; i++ ) {
    cvGetSubRect( levmar.P, point_mat, cvRect( 0, levmar.num_cams * levmar.num_cam_param+ levmar.num_point_param * i, 1, levmar.num_point_param ));
    CvScalar x = cvGet2D(point_mat,0,0); CvScalar y = cvGet2D(point_mat,1,0); CvScalar z = cvGet2D(point_mat,2,0);
    points3D.push_back(cv::Point3d(x.val[0],y.val[0],z.val[0]));
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

  return NO_ERROR;
}

/** Compute extrinsic parameters R and t\n
 * \f$x=K[R~t]X\f$
 * @param objectPoints is input 3D points vector
 * @param imagePoints is input 2D points vector
 * @param R,t are out rotation and translation matrix
 * @param cam is input camera model
 * @param useExtrinsicGuess is an optional option, set to true to use R and t as guess
 */
int Reconstruction3D::computeExtrinsicParametersOpenCV(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, Camera cam, bool useExtrinsicGuess)
{
  assert(objectPoints.size()==imagePoints.size());
  R = cv::Mat::eye(3,3,CV_64F);
  t = cv::Mat::zeros(3,1,CV_64F);
  CvMat rvec = Tools::Rodrigues(R);
  CvMat tvec = t;
  CvMat object = Tools::toMat(objectPoints);
  CvMat image = Tools::toMat(imagePoints);
  CvMat K = cam.K;
  CvMat dist = cam.dist;
  cvFindExtrinsicCameraParams2(&object,&image,&K,NULL,&rvec,&tvec);
  cv::Mat Rvec(&rvec);
  cv::Mat Tvec(&tvec);
  R = Tools::Rodrigues(Rvec).clone();
  t = Tvec.clone();

  std::cout << "Release mat" << std::endl;
  return 1;
}

/** Compute extrinsic parameters R and t\n
 * \f$x=K[R~t]X\f$
 * @param objectPoints is input 3D points vector
 * @param imagePoints is input 2D points vector
 * @param R,t are out rotation and translation matrix
 * @param cam is input camera model
 * @param useExtrinsicGuess is an optional option, set to true to use R and t as guess
 */
int Reconstruction3D::computeExtrinsicParameters(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, Camera cam, bool useExtrinsicGuess)
{
  assert(objectPoints.size()==imagePoints.size());

  if(t.rows!=3 || t.cols!=1)
    t = cv::Mat::zeros(3,1,CV_64F);


  if(R.rows!=3 || R.cols!=3)
    R = cv::Mat::eye(3,3,CV_64F);

  int count  = objectPoints.size();
  int subsetSize = cam.type==OMNI?20:5; //20 if omni, 5 if persp
  assert(count>=subsetSize);
  if(count<subsetSize){
     R = cv::Mat::eye(3,3,CV_64F);
     t = cv::Mat::zeros(3,1,CV_64F);
     return NOT_ENOUGH_POINTS;
  }

  int maxIter = subsetSize==count?1:500;

  int nbGood = 0;
  int max_k = 0;
  int k;

  /*std::vector<cv::Point3f> objectPointsF = Tools::toFloatVector(objectPoints);
  std::vector<cv::Point2f> imagePointsF = Tools::toFloatVector(imagePoints);

  cv::solvePnPRansac(objectPointsF,imagePointsF,cam.K,cam.dist,R,t);
  Tools::display(R,"R0");

  R = Tools::Rodrigues(-R);
  R = R.t();
  t = -t;

  Tools::display(R,"R0");
  Tools::display(t,"t0");

  return NO_ERROR;*/

#pragma omp parallel private(k) shared(max_k,nbGood,R,t)
  {
    cv::Mat ttemp = cv::Mat(3,1,CV_64F), Rtemp;
    int tid = omp_get_thread_num();
    if (tid == 0) {
      int nthreads = omp_get_num_threads();
      printf("Number of threads = %d\n", nthreads);
    }
    std::vector<cv::Point3d> objectPoints_temp;
    std::vector<cv::Point2d> imagePoints_temp;
    for(k=0;k<maxIter && nbGood<0.9*count;k++){

      getSubset(objectPoints,objectPoints_temp,imagePoints,imagePoints_temp,subsetSize);

      if(cam.type==PERSP){
#if 0
        computeExtrinsicParametersWithMatlabMethod(objectPoints_temp,imagePoints_temp,Rtemp,ttemp,cam);
#else
        double pts3D[subsetSize*3];
        double pts2D[subsetSize*2];
        double _r[3], _t[3], _a[9];
        CvMat obj = cvMat(subsetSize,3,CV_64F,pts3D);
        CvMat img = cvMat(subsetSize,2,CV_64F,pts2D);
        CvMat Rvec = cvMat(3,1,CV_64F,_r);
        CvMat tvec = cvMat(3,1,CV_64F,_t);
        CvMat A = cvMat(3,3,CV_64F,_a);
        for(int i=0;i<3;i++)
          for(int j=0;j<3;j++)
            _a[3*i+j] = cam.K.at<double>(i,j);

        for(int i=0;i<subsetSize;i++){
          pts3D[3*i+0] =  objectPoints_temp.at(i).x;
          pts3D[3*i+1] =  objectPoints_temp.at(i).y;
          pts3D[3*i+2] =  objectPoints_temp.at(i).z;
          pts2D[2*i+0] =  imagePoints_temp.at(i).x;
          pts2D[2*i+1] =  imagePoints_temp.at(i).y;
        }

        if(useExtrinsicGuess){
          cv::Mat Rtemp;
          cv::Rodrigues(R,Rtemp);
          for(int i=0;i<3;i++){
            _r[i] = Rtemp.at<double>(i,0);
            _t[i] = t.at<double>(i,0);
          }
        }else{
          for(int i=0;i<3;i++){
            _r[i] = 0.0;
            _t[i] = 0.0;
          }
        }

        cvFindExtrinsicCameraParams2(&obj,&img,&A,0,&Rvec,&tvec,useExtrinsicGuess);

        ttemp.at<double>(0,0) = _t[0];
        ttemp.at<double>(1,0) = _t[1];
        ttemp.at<double>(2,0) = _t[2];
        cv::Mat Rutheta(3,1,CV_64F);
        Rutheta.at<double>(0,0) = _r[0];
        Rutheta.at<double>(1,0) = _r[1];
        Rutheta.at<double>(2,0) = _r[2];
        cv::Rodrigues(Rutheta,Rtemp);
        Rtemp = Rtemp.t();
        ttemp = -Rtemp*ttemp;
#endif
      }

      if(cam.type==OMNI){         
        //PuigPhD p24
        cv::Mat A(6*subsetSize,60,CV_64F),q,Q,K,D;
        for(int i=0;i<subsetSize;i++){
          //cv::Mat q = liftCoordinate(imagePoints_temp.at(i),6).t();
          //cv::Mat Q = liftCoordinate(objectPoints_temp.at(i)).t();
          q = liftMatrix( skewSymetric( toHomogeneous( imagePoints_temp.at(i) ) ) );
          Q = liftCoordinate(objectPoints_temp.at(i)).t();
          K = kron(q,Q);
          D = A(cv::Rect( 0, i*6, 60, 6) );
          K.copyTo(D);
          //k.copyTo(A.row(i));
        }
        cv::Mat Pcata = rightNullVector(A).reshape(0,6);
        //displaySize(Pcata,"Pcata");

        double epsilon = cam.epsilon;

        //M = λ (K^) (X_ξ) D−1 (X^T_ξ) (K^)^T

        //Compute X_ξ
        cv::Mat Xe = cv::Mat::eye(6,6,CV_64F);
        Xe.at<double>(5,5) = 1.0 - epsilon*epsilon;
        Xe.at<double>(5,0) =  -epsilon*epsilon;
        Xe.at<double>(5,2) =  -epsilon*epsilon;

        //Compute D
        /*cv::Mat D = cv::Mat::eye(6,6,CV_64F);
        D.at<double>(1,1) = 2.0;
        D.at<double>(3,3) = 2.0;
        D.at<double>(4,4) = 2.0;*/

        //M = P_s D−1 P_s^T
        /*cv::Mat Ps = Pcata.colRange(0,6);
        cv::Mat M = Ps * D.inv() * Ps.t();

        double M66 = M.at<double>(5,5);
        double M16 = M.at<double>(0,5);
        double M46 = M.at<double>(3,5);
        double M56 = M.at<double>(4,5);
        double M44 = M.at<double>(3,3);

        double cx = M46/M66;
        double cy = M56/M66;
        double e = sqrt( ((M16/M66)-cx*cx) / (-2*(M44/M66) -cx*cx));
        double f = sqrt(2*(2*pow(e,4)+pow((1+e*e),2))*((M44/M66)-cx*cx));*/

        //Compute A_cata = (K^) X_ξ
        cv::Mat Acata = liftMatrix(cam.K) * Xe;

        //Compute T_cata
        cv::Mat Tcata = Acata.inv()*Pcata;
        cv::Mat Rest = Tcata.colRange(0,6).clone();
        cv::Mat Test = Tcata.colRange(6,10).clone();

        //Debug
        //std::cout << cv::determinant(Rest) << std::endl;
        //display(Rest,"Rest");
        //display(Test,"Test");
        //display(Rest.inv(),"Rest.inv()");

        Test = Rest.inv() * Test;

        //Debug
        //display(Test,"Test");

        double tx = -Test.at<double>(1,1);
        double ty = -Test.at<double>(1,0);
        double tz = -Test.at<double>(3,0);

        ttemp = toTranslationMatrix(tx,ty,tz);

        double gamma = atan2( Rest.at<double>(5,1) , Rest.at<double>(4,1) );
        cv::Mat Rzgamma = liftMatrix(toRotationMatrixZ(gamma));
        Rest = Rzgamma.inv() * Rest;
        double beta = atan2( -Rest.at<double>(5,2) , Rest.at<double>(2,2) );
        cv::Mat Rybeta = liftMatrix(toRotationMatrixY(beta));
        Rest = Rybeta.inv() * Rest;
        double alpha = atan2( Rest.at<double>(4,2) , Rest.at<double>(2,2) );

        //std::cout << "Angle x: " << alpha <<" / Angle y: " << beta << " / Angle z: " << gamma << std::endl;
        //Rtemp = toRotationMatrix(0,0,gamma);
        //Rtemp = toRotationMatrix(alpha,beta,gamma);
        Rtemp = toRotationMatrixZ(gamma) * toRotationMatrixY(beta) * toRotationMatrixX(alpha);

        //Debug
        //display(Rtemp,"Rtemp");
        //display(ttemp,"ttemp");
      }

      std::vector<cv::Point2d> reproj = getRetroprojPoints(objectPoints, cam, Rtemp, ttemp);

      int nb = 1;
      double limit = 10.0;
      for(int i=0;i<imagePoints.size();i++){
        if(cv::norm( reproj.at(i) - imagePoints.at(i) ) < limit){
          nb++;
        }
      }

      if(nb>nbGood){
        nbGood = nb;
        R = Rtemp.clone();
        t = ttemp.clone();
      }
      //printf("%d points in %d iterations\r",nbGood,k);
      max_k++;
    }
  } //end #pragma
  std::cout << nbGood << " points in " << max_k << " iterations" << std::endl;

}

/** Compute extrinsic parameters R and t\n
 * \f$x=K[R~t]X\f$
 * @param objectPoints is input 3D points vector
 * @param imagePoints is input 2D points vector
 * @param R,t are out rotation and translation matrix
 * @param K is input camera matrix
 */
int Reconstruction3D::computeExtrinsicParametersWithPseudoInverse(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, cv::Mat K)
{
  int count = MIN(objectPoints.size(),imagePoints.size());
  cv::Mat A = cv::Mat::zeros(2*count,11,CV_64F);
  cv::Mat b(2*count,1,CV_64F);

  cv::Mat pts = toMat(imagePoints);

  for(int i=0;i<count;i++){
    double X = objectPoints.at(i).x;
    double Y = objectPoints.at(i).y;
    double Z = objectPoints.at(i).z;
    double u = pts.at<double>(i,0);
    double v = pts.at<double>(i,1);
    A.at<double>(2*i,0) = A.at<double>(2*i+1,4) = X;
    A.at<double>(2*i,1) = A.at<double>(2*i+1,5) = Y;
    A.at<double>(2*i,2) = A.at<double>(2*i+1,6) = Z;
    A.at<double>(2*i,3) = A.at<double>(2*i+1,7) = 1.0;
    A.at<double>(2*i, 8) = - u*X;
    A.at<double>(2*i, 9) = - u*Y;
    A.at<double>(2*i,10) = - u*Z;
    A.at<double>(2*i+1, 8) = - v*X;
    A.at<double>(2*i+1, 9) = - v*Y;
    A.at<double>(2*i+1,10) = - v*Z;
    b.at<double>(2*i  ,0) = u;
    b.at<double>(2*i+1,0) = v;
  }

  cv::Mat x = solvePseudoInverse(A,b);
  cv::Mat one = cv::Mat::ones(1,1,CV_64F);
  concatVertical(x,one,x);
  cv::Mat P = x.reshape(0,3);

  P = K.inv() * P;

  R = P.colRange(0,3);
  t = P.col(3);

  double norm = cv::norm(R);

  R *= 1.0/norm;
  t *= 1.0/norm;

  return NO_ERROR;
}

/** Compute extrinsic parameters R and t\n
 * \f$x=K[R~t]X\f$
 * @param objectPoints is input 3D points vector
 * @param imagePoints is input 2D points vector
 * @param R,t are out rotation and translation matrix
 * @param cam is input camera model
 */
int Reconstruction3D::computeExtrinsicParametersWithMatlabMethod(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, Camera cam)
{
  int count = MIN(objectPoints.size(),imagePoints.size());
  R = cv::Mat(3,3,CV_64F);
  t = cv::Mat(3,1,CV_64F);

  //Omar's matlab method

  cv::Mat B = cv::Mat::zeros(2*count,9,CV_64F);
  cv::Mat C(2*count,3,CV_64F);

  cv::Mat pts3D = toMat(objectPoints);
  cv::Mat pts2D = toMat(imagePoints);


  for(int i=0;i<count;i++){
    double X = pts3D.at<double>(i,0);
    double Y = pts3D.at<double>(i,1);
    double Z = pts3D.at<double>(i,2);
    double u = pts2D.at<double>(i,0);
    double v = pts2D.at<double>(i,1);
    B.at<double>(2*i,0) = B.at<double>(2*i+1,4) = X;
    B.at<double>(2*i,1) = B.at<double>(2*i+1,5) = Y;
    B.at<double>(2*i,2) = B.at<double>(2*i+1,6) = Z;
    B.at<double>(2*i,3) = B.at<double>(2*i+1,7) = 1.0;
    B.at<double>(2*i, 8)   = - u;
    B.at<double>(2*i+1, 8) = - v;
    C.at<double>(2*i  ,0) = -u*X;
    C.at<double>(2*i  ,1) = -u*Y;
    C.at<double>(2*i  ,2) = -u*Z;
    C.at<double>(2*i+1,0) = -v*X;
    C.at<double>(2*i+1,1) = -v*Y;
    C.at<double>(2*i+1,2) = -v*Z;
  }
  //display(B,"B");
  //display(C,"C");
  cv::Mat pB = pseudoInv(B);
  cv::Mat D = C.t()*C - C.t()*B* pB *C;
  //display(D,"D");

  //cv::Mat val,vec;
  //cv::eigen(D,val,vec);
  //display(val,"val");
  cv::Mat x3 = minEigenVector(D);
  x3 /= cv::norm(x3);

  //display(x3,"x3");

  cv::Mat x9 = - pB * C * x3;

  if(get(x9,8,0)<0){
    x3 = -x3;
    x9 = -x9;
  }

  //display(x9,"x9");

  cv::Mat M(3,4,CV_64F);
  for(int i=0;i<4;i++)
    for(int j=0;j<2;j++)
      Tools::set(M,i,j, get(x9,2*j+i,0) );
  for(int i=0;i<3;i++)
    Tools::set(M,2,i, get(x3,i,0) );
  Tools::set(M,2,3, get(x9,8,0));

  cv::Mat P = cam.K.inv()*M;
  //P /= cv::norm(P.col(0));
  //display(P,"P");

  R = cv::Mat(3,3,CV_64F);
  t = cv::Mat(3,1,CV_64F);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      Tools::set(R,i,j,get(P,i,j));

  for(int i=0;i<3;i++)
    Tools::set(t,i,0,get(P,i,3));
  return NO_ERROR;
}

/**
 * @warning BAD RESULT
 */
int Reconstruction3D::computeExtrinsicParametersWithSVD(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, cv::Mat K)
{
  std::cout << "computeExtrinsicParametersWithSVD" << std::endl;

  int count = MIN(objectPoints.size(),imagePoints.size());
  R = cv::Mat(3,3,CV_64F);
  t = cv::Mat(3,1,CV_64F);

  cv::Mat A = cv::Mat::zeros(2*count,12,CV_64F);

  for(int i=0;i<count;i++){
    double X = objectPoints.at(i).x;
    double Y = objectPoints.at(i).y;
    double Z = objectPoints.at(i).z;
    double u = imagePoints.at(i).x;
    double v = imagePoints.at(i).y;
    A.at<double>(2*i,0) = A.at<double>(2*i+1,4) = X;
    A.at<double>(2*i,1) = A.at<double>(2*i+1,5) = Y;
    A.at<double>(2*i,2) = A.at<double>(2*i+1,6) = Z;
    A.at<double>(2*i,3) = A.at<double>(2*i+1,7) = 1.0;
    A.at<double>(2*i, 8) = - u*X;
    A.at<double>(2*i, 9) = - u*Y;
    A.at<double>(2*i,10) = - u*Z;
    A.at<double>(2*i,10) = - u;
    A.at<double>(2*i+1, 8) = - v*X;
    A.at<double>(2*i+1, 9) = - v*Y;
    A.at<double>(2*i+1,10) = - v*Z;
    A.at<double>(2*i+1,10) = - v;
  }

  cv::Mat LL = A.t()*A;
  cv::SVD svd(LL,cv::SVD::MODIFY_A);
  cv::Mat P = svd.vt.row(11).reshape(0,3).clone();
  display(P);
  cv::Mat RR = P.colRange(0,3);
  cv::Mat tt = P.col(3);
  display(RR);
  if(cv::determinant(RR)<0){
    RR = -1.0*RR;
    tt = -1.0*tt;
  }
  double n = cv::norm(RR);
  cv::SVD svdRR(RR,cv::SVD::MODIFY_A);
  R = svdRR.u*svdRR.vt;
  t = tt * cv::norm(R)/n;

  return NO_ERROR;
}

/**
 * @warning BAD RESULT
 */
int Reconstruction3D::computeExtrinsicParametersWithLM(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, cv::Mat K)
{
  int count = MIN(objectPoints.size(),imagePoints.size());
  bool computeSVD = false;
  if(R.rows!=3 || R.cols!=3){
    R = cv::Mat(3,3,CV_64F);
    computeSVD= true;
  }
  if(t.rows!=3 ||t.cols!=1){
    t = cv::Mat(3,1,CV_64F);
    computeSVD= true;
  }

  //Initialize parameters if needed
  if(computeSVD)
    computeExtrinsicParametersWithSVD(objectPoints,imagePoints,R,t,K);

  cv::Mat Rvec;
  cv::Rodrigues(R,Rvec);


  CvLevMarq solver(6,count*2, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,50,FLT_EPSILON), true);

  CvMat *matM = cvCreateMat( 1, count, CV_64FC3 );
  CvMat *_m = cvCreateMat( 1, count, CV_64FC2 );
  CvMat matA = K;

  double *_obj[count*3], _img[count*2];
  CvMat _objectPoints = cvMat(count,3,CV_64F,_obj);
  CvMat _imagePoints = cvMat(count,2,CV_64F,_img);
  for(unsigned int i=0;i<count;i++){
    cvmSet(&_objectPoints,i,0,objectPoints.at(i).x);
    cvmSet(&_objectPoints,i,1,objectPoints.at(i).y);
    cvmSet(&_objectPoints,i,2,objectPoints.at(i).z);
    cvmSet(&_imagePoints,i,0,imagePoints.at(i).x);
    cvmSet(&_imagePoints,i,1,imagePoints.at(i).y);
  }

  cvConvertPointsHomogeneous( &_objectPoints, matM );
  cvConvertPointsHomogeneous( &_imagePoints, _m );

  double param[6];
  param[0] = get(Rvec,0,0);
  param[1] = get(Rvec,1,0);
  param[2] = get(Rvec,2,0);
  param[3] = get(t,0,0);
  param[4] = get(t,1,0);
  param[5] = get(t,2,0);

  CvMat _param = cvMat( 6, 1, CV_64F, param );
  CvMat _r = cvMat( 3, 1, CV_64F, param );
  CvMat _t = cvMat( 3, 1, CV_64F, param + 3 );

  cvCopy( &_param, solver.param );

  CvMat _dpdr, _dpdt;
  CvMat *distCoeffs = 0;

  for(;;){
    CvMat *matJ = 0, *_err = 0;
    const CvMat *__param = 0;
    bool proceed = solver.update( __param, matJ, _err );
    cvCopy( __param, &_param );
    if( !proceed || !_err )
      break;
    cvReshape( _err, _err, 2, 1 );
    if( matJ ){
      cvGetCols( matJ, &_dpdr, 0, 3 );
      cvGetCols( matJ, &_dpdt, 3, 6 );
      cvProjectPoints2( matM, &_r, &_t, &matA, distCoeffs,_err, &_dpdr, &_dpdt, 0, 0, 0 );
    }else{
      cvProjectPoints2( matM, &_r, &_t, &matA, distCoeffs,_err, 0, 0, 0, 0, 0 );
    }
    cvSub(_err, _m, _err);
    cvReshape( _err, _err, 1, 2*count );
  }
  cvCopy( solver.param, &_param );

  Tools::set(Rvec,0,0,param[0]);
  Tools::set(Rvec,1,0,param[1]);
  Tools::set(Rvec,2,0,param[2]);
  Tools::set(t,0,0,param[3]);
  Tools::set(t,1,0,param[4]);
  Tools::set(t,2,0,param[5]);

  cv::Rodrigues(Rvec,R);

}

/** Compute extrinsic parameters R and t\n
 * \f$x=K[R~t]X\f$
 * @param objectPoints is input 3D points vector
 * @param imagePoints is input 2D points vector
 * @param Rmat,tmat are out rotation and translation matrix
 * @param K is input camera matrix
 */
int Reconstruction3D::computeExtrinsicParameters2(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &Rmat, cv::Mat &tmat, cv::Mat K)
{
  const int max_iter = 20;
  cv::Ptr<CvMat> matM, _Mxy, _m, _mn, matL, matJ;

  CvMat A = K;
  cv::Ptr<CvMat> distCoeffs = 0;
  double r[3],t[3];
  CvMat rvec = cvMat( 3, 1, CV_64F, r );
  CvMat tvec = cvMat( 3, 1, CV_64F, t );

  int i, count;
  double a[9], ar[9]={1,0,0,0,1,0,0,0,1}, R[9];
  double MM[9], U[9], V[9], W[3];
  CvScalar Mc;
  double param[6];
  CvMat matA = cvMat( 3, 3, CV_64F, a );
  CvMat _Ar = cvMat( 3, 3, CV_64F, ar );
  CvMat matR = cvMat( 3, 3, CV_64F, R );
  CvMat _r = cvMat( 3, 1, CV_64F, param );
  CvMat _t = cvMat( 3, 1, CV_64F, param + 3 );
  CvMat _Mc = cvMat( 1, 3, CV_64F, Mc.val );
  CvMat _MM = cvMat( 3, 3, CV_64F, MM );
  CvMat matU = cvMat( 3, 3, CV_64F, U );
  CvMat matV = cvMat( 3, 3, CV_64F, V );
  CvMat matW = cvMat( 3, 1, CV_64F, W );
  CvMat _param = cvMat( 6, 1, CV_64F, param );
  CvMat _dpdr, _dpdt;

  CV_Assert( CV_IS_MAT(&A) && CV_IS_MAT(&rvec) && CV_IS_MAT(&tvec) );

  count = MAX(objectPoints.size(), imagePoints.size());
  //std::cout << count << " points" << std::endl;
  matM = cvCreateMat( 1, count, CV_64FC3 );
  _m = cvCreateMat( 1, count, CV_64FC2 );

  cv::Mat obj = toMat(objectPoints);
  cv::Mat img = toMat(imagePoints);

  double *test1[count*3], test2[count*2];
  CvMat _objectPoints = cvMat(count,3,CV_64F,test1);
  CvMat _imagePoints = cvMat(count,2,CV_64F,test2);
  for(unsigned int i=0;i<count;i++){
    cvmSet(&_objectPoints,i,0,objectPoints.at(i).x);
    cvmSet(&_objectPoints,i,1,objectPoints.at(i).y);
    cvmSet(&_objectPoints,i,2,objectPoints.at(i).z);
    cvmSet(&_imagePoints,i,0,imagePoints.at(i).x);
    cvmSet(&_imagePoints,i,1,imagePoints.at(i).y);
  }

  cvConvertPointsHomogeneous( &_objectPoints, matM );
  cvConvertPointsHomogeneous( &_imagePoints, _m );

  //display(*matM,"matM");
  //display(obj,"obj");

  cvConvert( &A, &matA );

  CV_Assert( (CV_MAT_DEPTH(rvec.type) == CV_64F || CV_MAT_DEPTH(rvec.type) == CV_32F) &&
	     (rvec.rows == 1 || rvec.cols == 1) && rvec.rows*rvec.cols*CV_MAT_CN(rvec.type) == 3 );

  CV_Assert( (CV_MAT_DEPTH(tvec.type) == CV_64F || CV_MAT_DEPTH(tvec.type) == CV_32F) &&
	     (tvec.rows == 1 || tvec.cols == 1) && tvec.rows*tvec.cols*CV_MAT_CN(tvec.type) == 3 );

  _mn = cvCreateMat( 1, count, CV_64FC2 );
  _Mxy = cvCreateMat( 1, count, CV_64FC2 );

  //CV_Assert(CV_IS_MAT(_m) && CV_IS_MAT(_mn) && (_m->rows == 1 || _m->cols == 1));
  //CV_Assert((_mn->rows == 1 || _mn->cols == 1) && _m->cols + _m->rows - 1 == _mn->rows + _mn->cols - 1);
  //CV_Assert((CV_MAT_TYPE(_m->type) == CV_32FC2 || CV_MAT_TYPE(_m->type) == CV_64FC2));
  //CV_Assert((CV_MAT_TYPE(_mn->type) == CV_32FC2 || CV_MAT_TYPE(_mn->type) == CV_64FC2)) ;
  // normalize image points
  // (unapply the intrinsic matrix transformation and distortion)
  cvUndistortPoints( _m, _mn, &matA, distCoeffs, 0, &_Ar );

  bool useExtrinsicGuess = false;
  if( useExtrinsicGuess ){
    CvMat _r_temp = cvMat(rvec.rows, rvec.cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(rvec.type)), param );
    CvMat _t_temp = cvMat(tvec.rows, tvec.cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(tvec.type)), param + 3);
    cvConvert( &rvec, &_r_temp );
    cvConvert( &tvec, &_t_temp );
  }else{
    Mc = cvAvg(matM);
    cvReshape( matM, matM, 1, count );
    cvMulTransposed( matM, &_MM, 1, &_Mc );
    //display(_MM,"_MM");
    cvSVD( &_MM, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T );
    //display(matW,"matW");

    //cv::Scalar s = toScalar(mean(obj));
    //cv::Mat obj_temp = obj - s;
    //cv::Mat _MM_new = (obj_temp.t())*(obj_temp);
    //display(_MM_new,"_MM_new");
    //cv::SVD svd(_MM_new);
    //display(svd.w,"w");

    // initialize extrinsic parameters
    if( W[2]/W[1] < 1e-3 || count < 4 ) {
      //std::cout << "planar structure" << std::endl;
      // a planar structure case (all M's lie in the same plane)
      double tt[3], h[9], h1_norm, h2_norm;
      CvMat* R_transform = &matV;
      CvMat T_transform = cvMat( 3, 1, CV_64F, tt );
      CvMat matH = cvMat( 3, 3, CV_64F, h );
      CvMat _h1, _h2, _h3;

      if( V[2]*V[2] + V[5]*V[5] < 1e-10 )
        cvSetIdentity( R_transform );

      if( cvDet(R_transform) < 0 )
        cvScale( R_transform, R_transform, -1 );

      cvGEMM( R_transform, &_Mc, -1, 0, 0, &T_transform, CV_GEMM_B_T );

      for( i = 0; i < count; i++ ){
        const double* Rp = R_transform->data.db;
        const double* Tp = T_transform.data.db;
        const double* src = matM->data.db + i*3;
        double* dst = _Mxy->data.db + i*2;

        dst[0] = Rp[0]*src[0] + Rp[1]*src[1] + Rp[2]*src[2] + Tp[0];
        dst[1] = Rp[3]*src[0] + Rp[4]*src[1] + Rp[5]*src[2] + Tp[1];
      }

      cvFindHomography( _Mxy, _mn, &matH );

      if( cvCheckArr(&matH, CV_CHECK_QUIET) ){
        cvGetCol( &matH, &_h1, 0 );
        _h2 = _h1; _h2.data.db++;
        _h3 = _h2; _h3.data.db++;
        h1_norm = sqrt(h[0]*h[0] + h[3]*h[3] + h[6]*h[6]);
        h2_norm = sqrt(h[1]*h[1] + h[4]*h[4] + h[7]*h[7]);

        cvScale( &_h1, &_h1, 1./MAX(h1_norm, DBL_EPSILON) );
        cvScale( &_h2, &_h2, 1./MAX(h2_norm, DBL_EPSILON) );
        cvScale( &_h3, &_t, 2./MAX(h1_norm + h2_norm, DBL_EPSILON));
        cvCrossProduct( &_h1, &_h2, &_h3 );

        cvRodrigues2( &matH, &_r );
        cvRodrigues2( &_r, &matH );
        cvMatMulAdd( &matH, &T_transform, &_t, &_t );
        cvMatMul( &matH, R_transform, &matR );
      }else{
        cvSetIdentity( &matR );
        cvZero( &_t );
      }

      cvRodrigues2( &matR, &_r );
    }else{
      //std::cout << "non-planar structure. Use DLT method" << std::endl;
      // non-planar structure. Use DLT method
      double* L;
      double LL[12*12], LW[12], LV[12*12], sc;
      CvMat _LL = cvMat( 12, 12, CV_64F, LL );
      CvMat _LW = cvMat( 12, 1, CV_64F, LW );
      CvMat _LV = cvMat( 12, 12, CV_64F, LV );
      CvMat _RRt, _RR, _tt;
      CvPoint3D64f* M = (CvPoint3D64f*)matM->data.db;
      CvPoint2D64f* mn = (CvPoint2D64f*)_mn->data.db;

      matL = cvCreateMat( 2*count, 12, CV_64F );
      L = matL->data.db;

      for( i = 0; i < count; i++, L += 24 ){
        double x = -mn[i].x, y = -mn[i].y;
        L[0] = L[16] = M[i].x;
        L[1] = L[17] = M[i].y;
        L[2] = L[18] = M[i].z;
        L[3] = L[19] = 1.;
        L[4] = L[5] = L[6] = L[7] = 0.;
        L[12] = L[13] = L[14] = L[15] = 0.;
        L[8] = x*M[i].x;
        L[9] = x*M[i].y;
        L[10] = x*M[i].z;
        L[11] = x;
        L[20] = y*M[i].x;
        L[21] = y*M[i].y;
        L[22] = y*M[i].z;
        L[23] = y;
      }

      cvMulTransposed( matL, &_LL, 1 );
      cvSVD( &_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T );
      _RRt = cvMat( 3, 4, CV_64F, LV + 11*12 );
      cvGetCols( &_RRt, &_RR, 0, 3 );
      cvGetCol( &_RRt, &_tt, 3 );
      if( cvDet(&_RR) < 0 )
        cvScale( &_RRt, &_RRt, -1 );
      sc = cvNorm(&_RR);
      cvSVD( &_RR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
      cvGEMM( &matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T );
      cvScale( &_tt, &_t, cvNorm(&matR)/sc );
      cvRodrigues2( &matR, &_r );
    }
  }

  cvReshape( matM, matM, 3, 1 );
  cvReshape( _mn, _mn, 2, 1 ); // refine extrinsic parameters using iterative algorithm
  CvLevMarq solver( 6, count*2, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,max_iter,FLT_EPSILON), true);
  cvCopy( &_param, solver.param );

  for(;;){
    CvMat *matJ = 0, *_err = 0;
    const CvMat *__param = 0;
    bool proceed = solver.update( __param, matJ, _err );
    cvCopy( __param, &_param );
    if( !proceed || !_err )
      break;
    cvReshape( _err, _err, 2, 1 );
    if( matJ ){
      cvGetCols( matJ, &_dpdr, 0, 3 );
      cvGetCols( matJ, &_dpdt, 3, 6 );
      cvProjectPoints2( matM, &_r, &_t, &matA, distCoeffs,_err, &_dpdr, &_dpdt, 0, 0, 0 );
    }else{
      cvProjectPoints2( matM, &_r, &_t, &matA, distCoeffs,_err, 0, 0, 0, 0, 0 );
    }
    cvSub(_err, _m, _err);
    cvReshape( _err, _err, 1, 2*count );
  }
  cvCopy( solver.param, &_param );

  _r = cvMat( rvec.rows, rvec.cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(rvec.type)), param );
  _t = cvMat( tvec.rows, tvec.cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(tvec.type)), param + 3 );

  cvConvert( &_r, &rvec );
  cvConvert( &_t, &tvec );

  Rmat = cv::Mat(&matR);
  tmat = cv::Mat(&tvec);

  //display(Rmat,"Rmat");
  //display(tmat,"tmat");
  //std::cout << cv::norm(tmat) << std::endl;


  return NO_ERROR;
}

/** Project image point on shpere
 * @param pt is input 2D point
 * @param R,t are input rotation and translation matrix
 * @param cam is the camera model
 */
cv::Point3d Reconstruction3D::projectOnSphere(cv::Point2d pt, cv::Mat R, cv::Mat t, Camera cam)
{
  double e = cam.epsilon;
  double x = pt.x;
  double y = pt.y;
  double gamma = sqrt(1.0+(1.0-e*e)*(x*x+y*y));
  double nu = (-gamma-e*(x*x+y*y))/(e*e*(x*x+y*y)-1);
  cv::Point3d xbar(x,y,1.0/(1.0+e*nu));
  cv::Point3d xf = (1.0/nu + e)*xbar;
  return projPoint(R,t,xf);
}

/** Project image point on shpere
 * @param pt is input 2D point
 * @param epsilon is the camera model parameter
 */
cv::Point3d Reconstruction3D::projectOnSphere(cv::Point2d pt, double epsilon)
{
  double x = pt.x;
  double y = pt.y;
  double gamma = sqrt(1+(1-epsilon*epsilon)*(x*x+y*y));
  double nu = (-gamma-epsilon*(x*x+y*y))/(epsilon*epsilon*(x*x+y*y)-1);
  cv::Point3d xbar(x,y,1/(epsilon*nu));
  return (1/nu + epsilon)*xbar;
}

/** Project image point on shpere
 * @param pts is input points matrix
 * @param epsilon is the camera model parameter
 */
cv::Mat Reconstruction3D::projectOnSphere(cv::Mat pts, double epsilon)
{
  int count = pts.rows;
  cv::Mat temp = cv::Mat(count,3,CV_64F);
  for(int i=0;i<count;i++){
    double x = pts.at<double>(i,0);
    double y = pts.at<double>(i,1);
    double gamma = sqrt(1+(1-epsilon*epsilon)*(x*x+y*y));
    double nu = (-gamma-epsilon*(x*x+y*y))/(epsilon*epsilon*(x*x+y*y)-1);
    double s = epsilon + 1.0/nu;
    temp.at<double>(i,0) = x*s;
    temp.at<double>(i,1) = y*s;
    temp.at<double>(i,2) = s/(1.0 + epsilon*nu);
    //temp.at<double>(i,3) = 1.0;

    //temp /= temp.at<double>(i,2);
    double scale = temp.at<double>(i,2);
    for(int j=0;j<3;j++)
      temp.at<double>(i,j) /= scale;
  }
  return temp;
}

/** Compute fundamental matrix
 * @param pts1,pts2 are input vector points
 * @param F is the output Fundamental matrix
 * @param p is input parameters
 */
int Reconstruction3D::findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, Param p)
{
  cv::Mat error;
  return findFundamentalMat(pts1,pts2,F,error,p);
}

/** Compute fundamental matrix
 * @param pts1,pts2 are input vector points
 * @param F is the output Fundamental matrix
 * @param error is the output error matrix, square of the real distance
 * @param p is input parameters
 */
int Reconstruction3D::findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, cv::Mat &error, Param p)
{
  return findFundamentalMat(pts1,pts2,F,error, p.useRANSAC,p.reprojThreshold,p.confidence,p.maxIter);
}

/** Compute fundamental matrix
 * @param pts1,pts2 are input vector points
 * @param F is the output Fundamental matrix
 * @param useRANSAC set to true to use RANSAC
 * @param maxIter is the maximum number of iteration for RANSAC algorithm
 * @param reprojThreshold,confidence are two parameters
 */
int Reconstruction3D::findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, bool useRANSAC, double reprojThreshold, double confidence, int maxIter)
{
  cv::Mat error;
  return findFundamentalMat(pts1,pts2,F,error, useRANSAC,reprojThreshold,confidence,maxIter);
}

/** Compute fundamental matrix
 * @param pts1,pts2 are input vector points
 * @param F is the output Fundamental matrix
 * @param error is the output error matrix, square of the real distance
 * @param useRANSAC set to true to use RANSAC
 * @param maxIter is the maximum number of iteration for RANSAC algorithm
 * @param reprojThreshold,confidence are two parameters
 */
int Reconstruction3D::findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, cv::Mat &error, bool useRANSAC, double reprojThreshold, double confidence, int maxIter)
{
  /*cv::Mat m1(pts1.size(),3,CV_64F);
    cv::Mat m2(pts2.size(),3,CV_64F);
    for(unsigned int i=0;i<pts1.size();i++){
    m1.at<double>(i,0) = pts1.at(i).x;
    m1.at<double>(i,1) = pts1.at(i).y;
    m1.at<double>(i,2) = 1.0;
    m2.at<double>(i,0) = pts2.at(i).x;
    m2.at<double>(i,1) = pts2.at(i).y;
    m2.at<double>(i,2) = 1.0;
    }*/
  cv::Mat m1 = toMat(pts1);
  cv::Mat m2 = toMat(pts2);
  return findFundamentalMat(m1,m2,F,error,useRANSAC,reprojThreshold,confidence,maxIter);
}

/** Compute fundamental matrix
 * @param pts1,pts2 are input matrix points
 * @param F is the output Fundamental matrix
 * @param error is the output error matrix, square of the real distance
 * @param useRANSAC set to true to use RANSAC
 * @param maxIter is the maximum number of iteration for RANSAC algorithm
 * @param reprojThreshold,confidence are two parameters
 */
int Reconstruction3D::findFundamentalMat( cv::Mat &pts1, cv::Mat &pts2, cv::Mat &F, cv::Mat &error, bool useRANSAC, double reprojThreshold, double confidence, int maxIter)
{
  int count = MIN(pts1.rows,pts2.rows);
  bool result = false;
  if( reprojThreshold <= 0 )
    reprojThreshold = 3;
  if( confidence < DBL_EPSILON || confidence > 1 - DBL_EPSILON )
    confidence = 0.99;

#if 1
  if(pts1.cols<3){
    cv::Mat pts1_temp(count,3,CV_64F);
    for(int i=0;i<count;i++){
      pts1_temp.at<double>(i,0) = pts1.at<double>(i,0);
      pts1_temp.at<double>(i,1) = pts1.at<double>(i,1);
      pts1_temp.at<double>(i,2) = 1.0;
    }
    pts1 = pts1_temp.clone();
  }
  if(pts2.cols<3){
    cv::Mat pts2_temp(count,3,CV_64F);
    for(int i=0;i<count;i++){
      pts2_temp.at<double>(i,0) = pts2.at<double>(i,0);
      pts2_temp.at<double>(i,1) = pts2.at<double>(i,1);
      pts2_temp.at<double>(i,2) = 1.0;
    }
    pts2 = pts2_temp.clone();
  }

  if( count >= 15 ){
    //  std::cout << "Compute Fundamental Matrix with parameters" << std::endl <<
    //          "RANSAC="<<useRANSAC<<", reprojThreshold="<<reprojThreshold<<", confidence="<<confidence<<", maxIter="<<maxIter<<std::endl;
    if(useRANSAC){
      result = estimator->runRANSAC(pts1, pts2, F, error, reprojThreshold, confidence, maxIter);
    }else{
      std::vector<cv::Mat> M;
      result = estimator->runNPoint(pts1, pts2, M, count);
      if(result) F = M.at(0).clone();
    }
    if(!result)
      std::cout << "F not found" << std::endl;
  }else{
    std::cout << "Not enough points (" <<count<<")"<< std::endl;
  }

#else
  cv::Mat pts1C2 = pts1.reshape(2).clone();
  cv::Mat pts2C2 = pts2.reshape(2).clone();

  cv::Mat mask(count,1,CV_8U);
  F = cv::findFundamentalMat(pts1C2,pts2C2,mask);


  error = cv::Mat(count,1,CV_64F);
  for(int i=0;i<mask.rows;i++){
    if(mask.at<bool>(i,0)==1)
      Tools::set(error,i,0,0.0);
    else
      Tools::set(error,i,0,1.0);
  }
  result = true;
#endif

  if( !result )
    return MAT_FUNDAMENTAL_ERROR;
  return NO_ERROR;
}

/** Compute the trifocal tensor
 * @param pts1,pts2,pts3 are input points vectors
 * @param T1,T2,T3 are output Trifocal Tensors
 * @param K is input camera matrix
 */
int Reconstruction3D::findTrifocalTensor(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point2d> pts3, cv::Mat &T1, cv::Mat &T2, cv::Mat &T3, cv::Mat &K)
{
  int count = MIN(pts1.size(),MIN(pts2.size(),pts3.size()));
  cv::Mat m1(count,3,CV_64F);
  cv::Mat m2(count,3,CV_64F);
  cv::Mat m3(count,3,CV_64F);
#if 0
  normalize(pts1,K,m1);
  normalize(pts2,K,m2);
  normalize(pts3,K,m3);
#else
  m1 = toHomogeneous(pts1);
  m2 = toHomogeneous(pts2);
  m3 = toHomogeneous(pts3);
#endif
  cv::Mat A(count,27,CV_64F);
  for(int p=0;p<count;p++){
    /*   for(int i=0;i<3;i++)
	 for(int j=0;j<3;j++)
	 for(int k=0;k<3;k++){
	 double val = get(m1,p,i);
	 Tools::set(A,p,i*j*k,val);
	 }*/
    double x0 = get(m1,p,0);
    double x1 = get(m1,p,1);
    double x2 = get(m1,p,2);
    double x_0 = get(m2,p,0);
    double x_1 = get(m2,p,1);
    double x_2 = get(m2,p,2);
    double x__0 = get(m3,p,0);
    double x__1 = get(m3,p,1);
    double x__2 = get(m3,p,2);

    Tools::set(A,p,26,+x2*(x_1-x_0)*(x__1-x__0));//*T222
    Tools::set(A,p,25,-x2*(x_1-x_0)*(x__2-x__0));//*T221
    Tools::set(A,p,24,+x2*(x_1-x_0)*(x__2-x__1));//*T220
    Tools::set(A,p,23,-x2*(x_2-x_0)*(x__1-x__0));//*T212
    Tools::set(A,p,22,+x2*(x_2-x_0)*(x__2-x__0));//*T211
    Tools::set(A,p,21,-x2*(x_2-x_0)*(x__2-x__1));//*T210
    Tools::set(A,p,20,+x2*(x_2-x_1)*(x__1-x__0));//*T202
    Tools::set(A,p,19,-x2*(x_2-x_1)*(x__2-x__0));//*T201
    Tools::set(A,p,18,+x2*(x_2-x_1)*(x__2-x__1));//*T200
    Tools::set(A,p,17,+x1*(x_1-x_0)*(x__1-x__0));//*T122
    Tools::set(A,p,16,-x1*(x_1-x_0)*(x__2-x__0));//*T121
    Tools::set(A,p,15,+x1*(x_1-x_0)*(x__2-x__1));//*T120
    Tools::set(A,p,14,-x1*(x_2-x_0)*(x__1-x__0));//*T112
    Tools::set(A,p,13,+x1*(x_2-x_0)*(x__2-x__0));//*T111
    Tools::set(A,p,12,-x1*(x_2-x_0)*(x__2-x__1));//*T110
    Tools::set(A,p,11,+x1*(x_2-x_1)*(x__1-x__0));//*T102
    Tools::set(A,p,10,-x1*(x_2-x_1)*(x__2-x__0));//*T101
    Tools::set(A,p, 9,+x1*(x_2-x_1)*(x__2-x__1));//*T100
    Tools::set(A,p, 8,+x0*(x_1-x_0)*(x__1-x__0));//*T022
    Tools::set(A,p, 7,-x0*(x_1-x_0)*(x__2-x__0));//*T021
    Tools::set(A,p, 6,+x0*(x_1-x_0)*(x__2-x__1));//*T020
    Tools::set(A,p, 5,-x0*(x_2-x_0)*(x__1-x__0));//*T012
    Tools::set(A,p, 4,+x0*(x_2-x_0)*(x__2-x__0));//*T011
    Tools::set(A,p, 3,-x0*(x_2-x_0)*(x__2-x__1));//*T010
    Tools::set(A,p, 2,+x0*(x_2-x_1)*(x__1-x__0));//*T002
    Tools::set(A,p, 1,-x0*(x_2-x_1)*(x__2-x__0));//*T001
    Tools::set(A,p, 0,+x0*(x_2-x_1)*(x__2-x__1));//*T000
  }
  T1 = cv::Mat(3,3,CV_64F);
  T2 = cv::Mat(3,3,CV_64F);
  T3 = cv::Mat(3,3,CV_64F);

  cv::Mat t = rightNullVector(A);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      Tools::set(T1,i,j, get(t,3*i+j,0));
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      Tools::set(T2,i,j, get(t,3*i+j + 9,0));
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      Tools::set(T3,i,j, get(t,3*i+j + 18,0));
  display(T1,"T1");
  display(T2,"T2");
  display(T3,"T3");

  cv::Mat u1 = rightNullVector(T1);
  cv::Mat u2 = rightNullVector(T2);
  cv::Mat u3 = rightNullVector(T3);
  cv::Mat v1 = leftNullVector(T1);
  cv::Mat v2 = leftNullVector(T2);
  cv::Mat v3 = leftNullVector(T3);

  cv::Mat U = concatH(u1,u2,u3);
  cv::Mat V = concatH(v1,v2,v3);

  cv::Mat ep = rightNullVector(U);
  cv::Mat epp = rightNullVector(V);

  display(ep,"ep");
  display(epp,"epp");

  cv::Mat Pp1 = T1*epp;
  cv::Mat Pp2 = T2*epp;
  cv::Mat Pp3 = T3*epp;
  cv::Mat Pp = concatH(Pp1,Pp2,Pp3,epp);
  display(Pp,"Pp");

  cv::Mat I3 = cv::Mat::eye(3,3,CV_64F);
  cv::Mat Ppp1 = (epp*epp.t() - I3)*T1.t()*ep;
  cv::Mat Ppp2 = (epp*epp.t() - I3)*T2.t()*ep;
  cv::Mat Ppp3 = (epp*epp.t() - I3)*T3.t()*ep;
  cv::Mat Ppp = concatH(Ppp1,Ppp2,Ppp3,ep);
  display(Ppp,"Ppp");
}

/** Compute the retroprojection error
 * @param pts1,pts2 are the inputs points vectors
 * @param mask is an optional input to select points to compute
 */
std::vector<double> Reconstruction3D::getRetroprojError(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, std::vector<bool> mask)
{
  assert(mask.size()==0 || (mask.size()==pts1.size() && mask.size()==pts2.size()));
  std::vector<double> out;
  bool useMask = true;
  if(mask.size()!=pts1.size()) useMask=false;
  for(unsigned int i=0;i<pts1.size();i++){
    bool check = true;
    if(useMask)
      if(!mask.at(i)){
        out.push_back(0.0);
        check = false;
    }
    if(check){
      double x = pts1.at(i).x - pts2.at(i).x;
      double y = pts1.at(i).y - pts2.at(i).y;
      out.push_back(sqrt(x*x+y*y));
    }
  }
  return out;
}

/** Get the reprojection points
 * @param pts3D is the input points3D vector
 * @param cam is the input camera definition
 * @param R,t are the rotation and translation matrices
 * @return retroprojected points
 * @warning Bad result with catadioptric cameras
 */
std::vector<cv::Point2d> Reconstruction3D::getRetroprojPoints(std::vector<cv::Point3d> pts3D, Camera cam, cv::Mat R, cv::Mat t)
{
  cv::Mat Rtemp = R.clone();
  cv::Mat ttemp = t.clone();
  std::vector<cv::Point2d> reproj;
  reproj.clear();

  if(cam.type == PERSP){
    cv::Mat p3mat = toMat(pts3D);
    cv::Mat p3matbis(p3mat.rows,1,CV_64FC3);
    p3matbis = p3mat.reshape(3).clone();
#if 1
    Rtemp = Rtemp.t();
    ttemp = -Rtemp*ttemp;
#endif
    cv::Mat Rv1;
    cv::Rodrigues(Rtemp,Rv1);
    cv::Mat ptsr1mat;
    cv::projectPoints(p3matbis,Rv1,ttemp,cam.K,cam.dist,ptsr1mat);
    ptsr1mat = ptsr1mat.reshape(1).clone();
    reproj = toVect2D(ptsr1mat);
  }
  if(cam.type==OMNI){
    for(unsigned int i=0;i<pts3D.size();i++){
      cv::Mat point3D = toMat(pts3D.at(i));
      //Move point in [I3,t0]
      cv::Mat reprojPts = R.t()*(point3D - t);
      cv::Point3d Q = toPoint3d(reprojPts);
      //Project to sphere
      double norm = cv::norm(Q);
      Q = Q * (1.0/norm);
      //Project to plan
      cv::Point3d S(Q.x/(Q.z+cam.epsilon), Q.y/(Q.z+cam.epsilon), 1);
      cv::Mat pt2 = toMat(S);
      pt2 = cam.K * pt2;
      cv::Point2d p = toPoint2d(pt2);
      reproj.push_back(p);
    }
  }
  return reproj;
}

/** Get the reprojection image
 * @param img is the input image
 * @param pts_input is the input points2D vector
 * @param pts_reproj is the input points2D vector of retroprojected points
 * @param good_matching is the optional mask
 * @return The image with 2D points, retroprojected points, lines between this points
 */
cv::Mat Reconstruction3D::getRetropjImage(cv::Mat img, std::vector<cv::Point2d> pts_input, std::vector<cv::Point2d> pts_reproj, std::vector<bool> good_matching)
{
  cv::Mat out = img.clone();
  int count = MIN(pts_input.size(),pts_reproj.size());
  for(int i=0; i<count;i++){
    int linesize = 2;
    cv::Scalar linecolor = CV_RGB(255,255,255);
    if(i<good_matching.size()){
      if(good_matching.at(i)){
	linecolor = CV_RGB(0,255,0);
      }else{
	linecolor = CV_RGB(255,0,0);
	linesize = 1;
      }
    }
    cv::circle(out,pts_input.at(i), 3,CV_RGB(0,0,255),-1);
    cv::circle(out,pts_reproj.at(i),3,CV_RGB(255,0,255),-1);
    cv::line(out,pts_input.at(i),pts_reproj.at(i),linecolor,linesize);
  }
  return out;
}

/** Get the reprojection image
 * @param img is the input image
 * @param pts2D is the input points2D vector
 * @param pts3D is the input points3D vector
 * @param cam is the input camera definition
 * @param R,t are the rotation and translation matrices
 * @return The image with 2D points, retroprojected points, lines between this points
 */
cv::Mat Reconstruction3D::getRetropjImage(cv::Mat &img, std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, Camera cam, cv::Mat R, cv::Mat t)
{
  std::vector<bool> empty;
  return getRetropjImage(img,pts2D,pts3D,cam,R,t,empty);
}

/** Get the reprojection image
 * @param img is the input image
 * @param pts2D is the input points2D vector
 * @param pts3D is the input points3D vector
 * @param cam is the input camera definition
 * @param R,t are input rotation and translation matrices
 * @param good_matching is input vector of bools (true = inlier, false = outlier)
 * @return The image with 2D points, retroprojected points, lines between this points
 */
cv::Mat Reconstruction3D::getRetropjImage(cv::Mat &img, std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, Camera cam, cv::Mat R, cv::Mat t, std::vector<bool> &good_matching)
{
  std::vector<cv::Point2d> reproj = getRetroprojPoints(pts3D,cam,R,t);
  cv::Mat image = img.clone();

  int count = MIN(pts2D.size(),reproj.size());
  for(int i=0; i<count;i++){
    cv::Scalar linecolor = CV_RGB(255,255,255);

    if(i<good_matching.size()){
      if(good_matching.at(i)){
        linecolor = CV_RGB(0,255,0);
        cv::circle(image,pts2D.at(i), 3,CV_RGB(0,0,255),-1);
        cv::circle(image,reproj.at(i),3,linecolor,-1);
        cv::line(image,pts2D.at(i),reproj.at(i),linecolor,1);
      }else{
        linecolor = CV_RGB(255,0,0);
        cv::circle(image,pts2D.at(i), 3,CV_RGB(0,0,255),-1);
        cv::circle(image,reproj.at(i),3,linecolor,-1);
        cv::line(image,pts2D.at(i),reproj.at(i),linecolor,1);
      }
    }
  }
  return image;
}

std::vector<bool> Reconstruction3D::getMask(cv::Mat error, double threshold)
{
  std::vector<bool> mask;
  for(int i=0;i<error.rows;i++){
    if(error.at<double>(i,0)<threshold)
      mask.push_back(true);
    else
      mask.push_back(false);
  }
  return mask;
}

cv::Mat Reconstruction3D::backprojectionOmni(double r, double x0, double y0)
{
  cv::Mat B = cv::Mat::zeros(3,4,CV_64F);
  B.at<double>(0,1) = 2.0*r;
  B.at<double>(0,3) = -2.0*r*x0;
  B.at<double>(1,2) = 2.0*r;
  B.at<double>(1,3) = -2.0*r*y0;
  B.at<double>(2,0) = 1.0;
  B.at<double>(2,1) = -2.0*x0;
  B.at<double>(2,2) = -2.0*y0;
  B.at<double>(2,3) = x0*x0 + y0*y0 - r*r;
  return B;
}
