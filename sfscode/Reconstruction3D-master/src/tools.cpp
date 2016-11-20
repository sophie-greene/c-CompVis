//#include "../include/reconstruction3D.h"
#include "../include/tools.h"

cv::Mat Tools::rescale(cv::Mat &M, double scale)
{
  cv::Mat m = cv::Mat(M.cols*scale,M.rows*scale,M.type());
  cv::resize(M,m,cv::Size(M.cols*scale,M.rows*scale));
  return m;
}

cv::Mat Tools::flip(cv::Mat &M, FLIPMODE mode)
{
  if(mode==NONE) return M;
  cv::Mat m;
  cv::flip(M,m,mode);
  return m;
}

cv::Mat Tools::copy(cv::Mat M)
{
    cv::Mat m = cv::Mat(M.rows,M.cols,CV_64F);
    for(int i=0;i<M.rows;i++)
        for(int j=0;j<M.cols;j++){
            double val = M.at<double>(i,j);
            m.at<double>(i,j) = val;
        }
    return m;
}

/** Convert angle from radians to degrees
 * @param rad is input angle
 */
double Tools::toDeg(double rad)
{
  return rad*180.0/M_PI;
}

/** Convert angle from degrees to radians
 * @param deg is input angle
 */
double Tools::toRad(double deg)
{
  return deg*M_PI/180.0;
}

/** Display an value
 * @param value is input value
 * @param name is the optional value name
 * @param jump is the optional option, set true to add a white line after name
 */
void Tools::display(double value, std::string name, bool jump)
{
  if(name.length()>0)
    printf("%s = ",name.c_str());
  if(jump)
    printf("\n");
  std::cout << value << std::endl;
}

/** Display an opencv matrix expression cv::MatExpr
 * @param m is input matrix
 * @param name is the optional matrix name
 * @param jump is the optional option, set true to add a white line after name
 */
void Tools::display(cv::MatExpr m, std::string name, bool jump)
{
  cv::Mat M = m;
  display(M,name,jump);
}

/** Display an opencv matrix cvMat
 * @param m is input matrix
 * @param name is the optional matrix name
 * @param jump is the optional option, set true to add a white line after name
 */
void Tools::display(CvMat m, std::string name, bool jump)
{
  cv::Mat M(&m);
  display(M,name,jump);
}

/** Display an opencv matrix cv::Mat
 * @param m is input matrix
 * @param name is the optional matrix name
 * @param jump is the optional option, set true to add a white line after name
 */
void Tools::display(cv::Mat m, std::string name, bool jump)
{
  if(name.length()>0)
    printf("%s = ",name.c_str());
  if(jump)
    printf("\n");
  std::cout << m << std::endl;
}

/** Display an opencv matrix cv::Mat size
 * @param m is input matrix
 * @param name is the optional matrix name
 * @param jump is the optional option, set true to add a white line after name
 */
void Tools::displaySize(cv::Mat m, std::string name)
{
  if(name.length()>0)
    printf("%s = ",name.c_str());
  std::cout << m.rows <<"x"<< m.cols << std::endl;
}

/** Display an opencv matrix cv::Mat size
 * @param m is input matrix
 * @param name is the optional matrix name
 * @param jump is the optional option, set true to add a white line after name
 */
void Tools::displaySize(cv::MatExpr m, std::string name)
{
  cv::Mat M = m;
  displaySize(M,name);
}

/** Display an opencv image cv::Mat
 * @param image is input image
 * @param waitKey is the optional wait for key option
 * @param windowTitle is the optional title for the window
 */
void Tools::displayImage(cv::Mat image, bool waitKey, std::string windowTitle, bool closeWindow)
{
  if(image.rows*image.cols==0){
    std::cout << "Invalid image" << std::endl;
    return;
  }
  cv::imshow(windowTitle,image);
  if(waitKey)
    cv::waitKey(0);
  if(closeWindow){
    cv::destroyWindow(windowTitle);
    cv::waitKey(5);
  }
}

/** Multiply 2 matrices using cv::gemm
 * @param M1,M2 are input matrices
 * @param Mout is output matrix
 */
void Tools::multiply(cv::Mat &M1, cv::Mat &M2, cv::Mat &Mout)
{
  cv::gemm(M1,M2,1,0,0,Mout);
}

/** Multiply 3 matrices using cv::gemm
 * @param M1,M2,M3 are input matrices
 * @param Mout is output matrix
 */
void Tools::multiply(cv::Mat &M1, cv::Mat &M2, cv::Mat &M3, cv::Mat &Mout)
{
  cv::Mat temp;
  multiply(M1,M2,temp);
  multiply(temp,M3,Mout);
}

/** Build diagonal matrix from vector
 * @param d is input matrix
 * @param D is output matrix
 */
void Tools::diag(cv::Mat &d, cv::Mat &D)
{
  diag(d.at<double>(0,0),d.at<double>(0,1),d.at<double>(0,2),D);
}

/** Build diagonal matrix from doubles
 * @param v1,v2,v3 are input doubles
 * @param D is output matrix
 */
void Tools::diag(double v1, double v2, double v3, cv::Mat &D)
{
  D.create(3,3,CV_64F);
  double *d_ = (double*)D.data;
  double v_[9] = {v1,0,0, 0,v2,0, 0,0,v3};
  for(int i=0;i<9;i++) d_[i] = v_[i];
}

/** Build diagonal matrix from vector
 * @param d is input matrix
 */
cv::Mat Tools::diag(cv::Mat &d)
{
  assert(d.rows==1 || d.cols==1);
  if(d.rows==1)
    return diag(d.at<double>(0,0),d.at<double>(0,1),d.at<double>(0,2));
  else
    return diag(d.at<double>(0,0),d.at<double>(1,0),d.at<double>(2,0));
}

/** Build diagonal matrix from doubles
 * @param v1,v2,v3 are input doubles
 */
cv::Mat Tools::diag(double v1, double v2, double v3)
{
  cv::Mat D = cv::Mat::zeros(3,3,CV_64F);
  D.at<double>(0,0) = v1;
  D.at<double>(1,1) = v2;
  D.at<double>(2,2) = v3;
  return D;
}

/** Compute left null vector of matrix
 * @param A is the input matrix
 */
cv::Mat Tools::leftNullVector(cv::Mat A)
{
  cv::SVD svd(A);
  int index = svd.u.cols-1;
  cv::Mat u = svd.u.col(index).clone();
  return u;
}

/** Compute right null vector of matrix
 * @param A is the input matrix
 */
cv::Mat Tools::rightNullVector(cv::Mat A)
{
  cv::SVD svd(A);
  int index = svd.vt.rows-1;
  cv::Mat v = svd.vt.row(index).t();
  return v;
}

/** Compute the minimal eigen vector of a matrix
 * @param M is the input matrix
 */
cv::Mat Tools::minEigenVector(cv::Mat &M)
{
  cv::Mat val,vec;
  cv::eigen(M,val,vec);
  int index = 0;
  double min = 1e20;
  for(int i=0;i<val.rows;i++){
    if(get(val,i,0)<min){
      min = get(val,i,0);
      index = i;
    }
  }
  return vec.row(index).t();
}

/** Project point
 * @param R,t are input rotation and translation matrix
 * @param Q is the input 3D point
 * @retval 3D point
 */
cv::Point3d Tools::projPoint(cv::MatExpr Re, cv::MatExpr te, cv::Point3d Q)
{
  cv::Mat R = Re, t = te;
  cv::Mat Qm = toMat(Q);
  cv::Mat p = R*Qm + t;
  cv::Point3d pt = toPoint3d(p);
  return pt;
}

/** Project point
 * @param R,t are input rotation and translation matrix
 * @param Q is the input 3D point
 * @retval 3D point
 */
cv::Point3d Tools::projPoint(cv::Mat &R, cv::Mat &t, cv::Point3d Q)
{
  cv::Mat Qm = toMat(Q);
  cv::Mat p = R*Qm + t;
  cv::Point3d pt = toPoint3d(p);
  return pt;
}

/** Project point
 * @param R,t are input rotation and translation matrix
 * @param Q is the input 3D point
 * @retval 3D point
 */
cv::Point2d Tools::projPoint(cv::Mat &R, cv::Mat &t, cv::Mat K, cv::Mat dist, cv::Point3d Q)
{
  std::vector<cv::Point3d> in;
  in.push_back(Q);
  std::vector<cv::Point2d> out = projPoints(R,t,K,dist,in);
  return out.at(0);
}

/** Project point
 * @param R,t are input rotation and translation matrix
 * @param Q is the input 3D point
 * @retval 3D point
 */
std::vector<cv::Point2d> Tools::projPoints(cv::Mat &R, cv::Mat &t, cv::Mat K, cv::Mat dist, std::vector<cv::Point3d> pts)
{
  if(pts.size()==0)
    return std::vector<cv::Point2d>();

  cv::Mat Rtemp = R.t();
  cv::Mat ttemp = -R.t()*t;
  std::vector<cv::Point2d> out;
  cv::projectPoints(pts,Rtemp,ttemp,K,dist,out);
  return out;
}

/** Project points
 * @param R,t are input rotation and translation matrix
 * @param pts in input 3D point vector
 * @retval 3D point vector
 */
std::vector<cv::Point3d> Tools::projPoints3D(cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> pts, double scale)
{
  if(scale!=1.0)
    for(int i=0;i<pts.size();i++)
      pts[i] *= scale;
  cv::Mat points = toHomogeneous(pts);
  cv::Mat P = concatH(R,t);
  cv::Mat Q = P*points.t();
  Q = Q.t();
  return toVect3D(Q);
}

/** lift coordinate for omnidirectional point
 * @param pt is the input 2D point
 */
cv::Mat Tools::liftCoordinate(cv::Point2d pt, int size)
{
  //See [Puig12]
  assert(size==4 || size==6);
  if(size==4){
    cv::Mat m(4,1,CV_64F);
    m.at<double>(0,0) = pt.x*pt.x + pt.y*pt.y;
    m.at<double>(1,0) = pt.x;
    m.at<double>(2,0) = pt.y;
    m.at<double>(3,0) = 1.0;
    return m;
  }else{
      cv::Mat m(6,1,CV_64F);
      m.at<double>(0,0) = pt.x*pt.x + pt.y*pt.y;
      m.at<double>(1,0) = pt.x;
      m.at<double>(2,0) = pt.y;
      m.at<double>(3,0) = 1.0;
      m.at<double>(4,0) = 1.0;
      m.at<double>(5,0) = 1.0;
      return m;
  }
}

/** lift coordinate for omnidirectional point
 * @param pt is the input 3D point
 */
cv::Mat Tools::liftCoordinate(cv::Point3d pt, int size)
{
  //See [Puig12]
  assert(size==10);
  if(size==10){
    cv::Mat m(10,1,CV_64F);
    m.at<double>(0,0) = pt.x*pt.x;
    m.at<double>(1,0) = pt.x*pt.y;
    m.at<double>(2,0) = pt.y*pt.y;
    m.at<double>(3,0) = pt.x*pt.z;
    m.at<double>(4,0) = pt.y*pt.z;
    m.at<double>(5,0) = pt.z*pt.z;
    m.at<double>(6,0) = pt.x;
    m.at<double>(7,0) = pt.y;
    m.at<double>(8,0) = pt.z;
    m.at<double>(9,0) = 1.0;
    return m;
  }
}

/** lift matrix
 * @param M is the input matrix
 */
cv::Mat Tools::liftMatrix(cv::Mat M)
{
  // [BastanlarPhD] p18
  cv::Mat P = cv::Mat::zeros(6,9,CV_64F);
  P.at<double>(0,0) = P.at<double>(1,1) = P.at<double>(1,3) = P.at<double>(2,4) = 1.0;
  P.at<double>(3,2) = P.at<double>(3,6) = P.at<double>(4,5) = P.at<double>(4,7) = P.at<double>(5,8) = 1.0;
  cv::Mat D = cv::Mat::eye(6,6,CV_64F);
  D.at<double>(1,1) = D.at<double>(3,3) = D.at<double>(4,4) = 2.0;

  cv::Mat A = D.inv()*P*kron(M,M)*P.t();
  return A;
}

/** Test if a matrix is a rotation matrix
 * @param M is the matrix to test
 * @return True if M is a rotation matrix
 */
bool Tools::isRotationMatrix(cv::Mat M)
{
  double det = cv::determinant(M);
  if( det < 0.99 || det > 1.01){
    std::cout << "Det(R)="<<det<<std::endl;
    return false;
  }
  cv::Mat Mt = M.t();
  cv::Mat Mi = M.inv();

  return areEqual(Mt,Mi,0.01);
}

/** Test if two matrices are equal
 * @param M1,M2 are matrices to test
 * @param threshold is tolerance : M1(i,j)-M2(i,j) < threshold
 * @return True if M1 and M2 are equal
 */
bool Tools::areEqual(cv::Mat &M1, cv::Mat &M2, double threshold)
{
  if(M1.rows!=M2.rows || M1.cols!=M2.cols)
    return false;

  for(int i=0;i<M1.rows;i++)
    for(int j=0;j<M2.cols;j++)
      if(fabs(get(M1,i,j)-get(M1,i,j))>threshold)
        return false;
  return true;
}

/** Flip point along X axis
 * @param pts is input/ouput points vector
 * @param size is the width of the image
 * pts(i).x = size - pts(i).x
 */
void Tools::flipX(std::vector<cv::Point2d> &pts, double size)
{
  for(unsigned int i=0;i<pts.size();i++)
    pts[i].x = size - pts[i].x;
}

/** Flip point along Y axis
 * @param pts is input/ouput points vector
 * @param size is the height of the image
 * pts(i).y = size - pts(i).y
 */
void Tools::flipY(std::vector<cv::Point2d> &pts, double size)
{
  for(unsigned int i=0;i<pts.size();i++)
    pts[i].y = size - pts[i].y;
}

/** Get the sign of a double
 * @param x is the double to test
 * @retval -1 if x<0, 1 else
 */
int Tools::sign(double x)
{
  if(x<0)
    return -1;
  else
    return 1;
}

/** Get the trace of a matrix
 * @param M is the input matrix
 * @retval sum( M(i,i) )
 */
double Tools::trace(cv::Mat &M)
{
  double res = 0.0;
  for(int i=0;i<MIN(M.rows,M.cols);i++)
    res += get(M,i,i);
  return res;
}

/** Compute de kronecker product of 3 first element of 2 vectors
 * @param M1,M2 are the input matrices 1*n (n>=3)
 */
cv::Mat Tools::kron(cv::Mat M1, cv::Mat M2)
{
  /*assert( (M1.rows==1 && M2.rows==1) || (M1.cols==1 && M2. cols==1));
  if(M1.cols==1 && M1.rows!=1)
    return kron(M1.t(),M2.t());
  cv::Mat M = cv::Mat(1,M1.cols*M2.cols,CV_64F);
  for(int i=0;i<M1.cols;i++)
    for(int j=0;j<M2.cols;j++)
      M.at<double>(0,M2.cols*i+j) = M1.at<double>(0,i) * M2.at<double>(0,j);
  return M;*/
  int m=M1.rows, n=M1.cols, p=M2.rows, q=M2.cols;
  cv::Mat M = cv::Mat::zeros(m*p,n*q,CV_64F);
  for(int i=0;i<m;i++){
    for(int j=0;j<n;j++){
      cv::Mat D = M(cv::Rect( j*q, i*p, q, p) );
      D = M1.at<double>(i,j) * M2;
    }
  }
  return M;
}

/** Build a skew symetrix matrix from doubles
 * @param v1,v2,v3 are input doubles
 * @param S is output matrix
 */
void Tools::skewSymetric(double v1, double v2, double v3, cv::Mat &S)
{
  S.create(3,3,CV_64F);
  double *s_ = (double*)S.data;
  double s[9] = {0,-v3,v2, v3,0,-v1, -v2,v1,0};
  for(int i=0;i<9;i++) s_[i] = s[i];
}

/** Build a skew symetrix matrix from doubles
 * @param v1,v2,v3 are input doubles
 */
cv::Mat Tools::skewSymetric(double v1, double v2, double v3)
{
  cv::Mat S = cv::Mat::zeros(3,3,CV_64F);
  S.at<double>(0,1) = -v3;
  S.at<double>(0,2) =  v2;
  S.at<double>(1,0) =  v3;
  S.at<double>(1,2) = -v1;
  S.at<double>(2,0) = -v2;
  S.at<double>(2,1) =  v1;
  return S;
}

/** Build a skew symetrix matrix from vector
 * @param v is input vector
 */
cv::Mat Tools::skewSymetric(cv::Mat v)
{
  double v1,v2,v3;
  if(v.rows<v.cols){
    v1 = v.at<double>(0,0);
    v2 = v.at<double>(0,1);
    v3 = v.at<double>(0,2);
  }else{
    v1 = v.at<double>(0,0);
    v2 = v.at<double>(1,0);
    v3 = v.at<double>(2,0);
  }
  return skewSymetric(v1,v2,v3);
}

/** Build a skew symetrix matrix from vector
 * @param v is input vector
 * @param S is output matrix
 */
void Tools::skewSymetric(cv::Mat &v, cv::Mat &S)
{
  double v1,v2,v3;
  if(v.rows<v.cols){
    v1 = v.at<double>(0,0);
    v2 = v.at<double>(0,1);
    v3 = v.at<double>(0,2);
  }else{
    v1 = v.at<double>(0,0);
    v2 = v.at<double>(1,0);
    v3 = v.at<double>(2,0);
  }
  skewSymetric(v1,v2,v3,S);
}

/** To Rank2 matrix
 * @param M Input Matrix
 */
void Tools::toRank2(cv::Mat &M)
{
  double val = M.at<double>(2,2);
  M *= 1.0/val;
}

/** Contact 2 matrices verticaly
 * @param M1,M2 are input matrices
 * @param Mout is output matrix
 */
void Tools::concatVertical(cv::Mat &M1, cv::Mat &M2, cv::Mat &Mout)
{
  cv::Mat M1copy,M2copy;
  M1copy = M1.clone();
  M2copy = M2.clone();
  assert(M1copy.cols == M2copy.cols);
  Mout.create(M1copy.rows+M2copy.rows,M1copy.cols,CV_64F);
  double *m_ = (double*)Mout.data;
  int index = 0;
  for(int i=0;i<M1copy.rows;i++){
    for(int j=0;j<M1copy.cols;j++){
      m_[index] = M1copy.at<double>(i,j);
      index++;
    }
  }
  for(int i=0;i<M2copy.rows;i++){
    for(int j=0;j<M2copy.cols;j++){
      m_[index] = M2copy.at<double>(i,j);
      index++;
    }
  }
}

/** Contact 2 matrices horizontaly
 * @param M1,M2 are input matrices
 * @param Mout is output matrix
 */
void Tools::concatHorizontal(cv::Mat &M1, cv::Mat &M2, cv::Mat &Mout)
{
  cv::Mat M1copy,M2copy;
  M1copy = M1.clone();
  M2copy = M2.clone();
  assert(M1copy.rows == M2copy.rows);
  int tot = M1copy.cols+M2copy.cols;
  Mout.create(M1copy.rows,tot,CV_64F);
  double *m_ = (double*)Mout.data;
  int index = 0;
  for(int i=0;i<M1copy.rows;i++){
    for(int j=0;j<M1copy.cols;j++){
      m_[index] = M1copy.at<double>(i,j);
      index++;
    }
    for(int j=0;j<M2copy.cols;j++){
      m_[index] = M2copy.at<double>(i,j);
      index++;
    }
  }
}

/** Concat horizontal matrices
 * @param M1,M2 are input matrices
 * @param M3,M4 are optional input matrices
 */
cv::Mat Tools::concatH(cv::Mat M1, cv::Mat M2, cv::Mat M3, cv::Mat M4)
{
  assert(M1.rows == M2.rows && M1.rows>0 && M2.rows>0);
  int rows = M1.rows;
  int cols = M1.cols + M2.cols;
  if(M3.rows == rows) cols += M3.cols;
  if(M4.rows == rows) cols += M4.cols;
  cv::Mat M(rows,cols,CV_64F);
  for(int i=0;i<rows;i++){
    for(int j=0;j<M1.cols;j++){
      M.at<double>(i,j) = M1.at<double>(i,j);
    }
    for(int j=0;j<M2.cols;j++){
      M.at<double>(i,j+M1.cols) = M2.at<double>(i,j);
    }
    if(M3.rows == rows){
      for(int j=0;j<M3.cols;j++){
        M.at<double>(i,j+M1.cols+M2.cols) = M3.at<double>(i,j);
      }
    }
    if(M4.rows == rows){
      for(int j=0;j<M4.cols;j++){
        M.at<double>(i,j+M1.cols+M2.cols+M3.cols) = M4.at<double>(i,j);
      }
    }
  }
  return M;
}

/** Concat vertical matrices
 * @param M1,M2 are input matrices
 * @param M3,M4 are optional input matrices
 */
cv::Mat Tools::concatV(cv::Mat M1, cv::Mat M2, cv::Mat M3, cv::Mat M4)
{
  assert(M1.cols == M2.cols && M1.cols>0 && M2.cols>0);
  int rows = M1.rows + M2.rows;
  int cols = M1.cols;
  if(M3.cols == cols) rows += M3.rows;
  if(M4.cols == cols) rows += M4.rows;
  cv::Mat M(rows,cols,CV_64F);
  for(int j=0;j<cols;j++){
    for(int i=0;i<M1.rows;i++){
      M.at<double>(i,j) = M1.at<double>(i,j);
    }
    for(int i=0;i<M2.rows;i++){
      M.at<double>(i+M1.rows,j) = M2.at<double>(i,j);
    }
    if(M3.cols == cols){
      for(int i=0;i<M3.rows;i++){
        M.at<double>(i+M1.rows+M2.rows,j) = M3.at<double>(i,j);
      }
    }
    if(M4.cols == cols){
      for(int i=0;i<M4.rows;i++){
        M.at<double>(i+M1.rows+M2.rows+M3.rows,j) = M4.at<double>(i,j);
      }
    }
  }
  return M;
}

/** Solve \f$A.x = b\f$
 * @param A is the input coefficient matrix
 * @param b is the input vector
 * @retval Solution x of equation A*x =b
 */
cv::Mat Tools::solvePseudoInverse(cv::Mat A, cv::Mat b)
{
  cv::Mat pseudoInverse = A.t() * A;
  pseudoInverse = pseudoInverse.inv() * A.t();
  cv::Mat x = pseudoInverse * b;
  return x;
}

/** Compute the matrix pseudo-inverse
 * @param M is the input matrix
 */
cv::Mat Tools::pseudoInv(cv::Mat &M)
{
  cv::Mat temp = M.t() * M;
  return temp.inv() * M.t();
}

/** Convert a cv::Point3d vector to cv::Mat (n x 3)
 * @param pts is input 3D points vector
 */
cv::Mat Tools::toMat(std::vector<cv::Point3d> &pts)
{
  cv::Mat M(pts.size(),3,CV_64F);
  for(unsigned int i=0;i<pts.size();i++){
    M.at<double>(i,0) = pts.at(i).x;
    M.at<double>(i,1) = pts.at(i).y;
    M.at<double>(i,2) = pts.at(i).z;
  }
  return M;
}

/** Convert a cv::Point2d vector to cv::Mat (n x 2)
 * @param pts is input 2D points vector
 */
cv::Mat Tools::toMat(std::vector<cv::Point2d> &pts)
{
  cv::Mat M(pts.size(),2,CV_64F);
  for(unsigned int i=0;i<pts.size();i++){
    M.at<double>(i,0) = pts.at(i).x;
    M.at<double>(i,1) = pts.at(i).y;
  }
  return M;
}

/** Convert a cv::Point2d to cv::Mat (2 x 1)
 * @param p is input 2D point
 */
cv::Mat Tools::toMat(cv::Point2d p)
{
  cv::Mat M(2,1,CV_64F);
  M.at<double>(0,0) = p.x;
  M.at<double>(1,0) = p.y;
  return M;
}

/** Convert a cv::Point3d to cv::Mat (3 x 1)
 * @param Q is input 3D point
 */
cv::Mat Tools::toMat(cv::Point3d Q)
{
  cv::Mat M(3,1,CV_64F);
  M.at<double>(0,0) = Q.x;
  M.at<double>(1,0) = Q.y;
  M.at<double>(2,0) = Q.z;
  return M;
}

/** Convert a cv::Point3d vector to homogeneous cv::Mat (n x 4)
 * @param pts is input 3D points vector
 */
cv::Mat Tools::toHomogeneous(std::vector<cv::Point3d> &pts)
{
  cv::Mat M(pts.size(),4,CV_64F);
  for(unsigned int i=0;i<pts.size();i++){
    M.at<double>(i,0) = pts.at(i).x;
    M.at<double>(i,1) = pts.at(i).y;
    M.at<double>(i,2) = pts.at(i).z;
    M.at<double>(i,3) = 1.0;
  }
  return M;
}

/** Convert a cv::Point2d vector to homogeneous cv::Mat (n x 3)
 * @param pts is input 2D points vector
 */
cv::Mat Tools::toHomogeneous(std::vector<cv::Point2d> &pts)
{
  cv::Mat M(pts.size(),3,CV_64F);
  for(unsigned int i=0;i<pts.size();i++){
    M.at<double>(i,0) = pts.at(i).x;
    M.at<double>(i,1) = pts.at(i).y;
    M.at<double>(i,2) = 1.0;
  }
  return M;
}

/** Convert a cv::Point3d vector to homogeneous cv::Mat (4 x 1)
 * @param Q is input 3D point
 */
cv::Mat Tools::toHomogeneous(cv::Point3d Q)
{
  cv::Mat M(4,1,CV_64F);
  M.at<double>(0,0) = Q.x;
  M.at<double>(1,0) = Q.y;
  M.at<double>(2,0) = Q.z;
  M.at<double>(3,0) = 1.0;
  return M;
}

/** Convert a cv::Point2d vector to homogeneous cv::Mat (3 x 1)
 * @param Q is input 2D point
 */
cv::Mat Tools::toHomogeneous(cv::Point2d Q)
{
  cv::Mat M(3,1,CV_64F);
  M.at<double>(0,0) = Q.x;
  M.at<double>(1,0) = Q.y;
  M.at<double>(2,0) = 1.0;
  return M;
}

/** Convert a cv::Mat to a cv::Point2d
 * @param m is input matrix
 */
cv::Point2d Tools::toPoint2d(cv::Mat &m)
{
  cv::Point2d pt(get(m,0,0),get(m,0,1));
  return pt;
}

/** Convert a cv::Mat to a cv::Point3d
 * @param m is input matrix
 */
cv::Point3d Tools::toPoint3d(cv::Mat &m)
{
  assert(m.rows==1 || m.cols==1);
  if(m.rows==1) return cv::Point3d(get(m,0,0),get(m,0,1),get(m,0,2));
  else return cv::Point3d(get(m,0,0),get(m,1,0),get(m,2,0));
}

/** Convert a cv::Mat to a cv::Point2d vector
 * @param M is input matrix
 */
std::vector<cv::Point2d> Tools::toVect2D(cv::Mat &M)
{
  assert(M.cols>=2);
  std::vector<cv::Point2d> vect;
  for(int i=0;i<M.rows;i++)
    vect.push_back(cv::Point2d(get(M,i,0),get(M,i,1)));
  return vect;
}

/** Convert a cv::Mat to a cv::Point3d vector
 * @param M is input matrix
 */
std::vector<cv::Point3d> Tools::toVect3D(cv::Mat &M)
{
  assert(M.cols>=3);
  std::vector<cv::Point3d> vect;
  for(int i=0;i<M.rows;i++)
    vect.push_back(cv::Point3d(get(M,i,0),get(M,i,1),get(M,i,2)));
  return vect;
}

/** Convert a double points to float points
 * @param pts is input vector
 */
std::vector<cv::Point2f> Tools::toFloatVector(std::vector<cv::Point2d> &pts)
{
  std::vector<cv::Point2f> vect;
  for(unsigned int i=0;i<pts.size();i++){
    cv::Point2f pt = pts.at(i);
    vect.push_back(pt);
  }
  return vect;
}

/** Convert a double points to float points
 * @param pts is input vector
 */
std::vector<cv::Point3f> Tools::toFloatVector(std::vector<cv::Point3d> &pts)
{
  std::vector<cv::Point3f> vect;
  for(unsigned int i=0;i<pts.size();i++){
    cv::Point3f pt = pts.at(i);
    vect.push_back(pt);
  }
  return vect;
}


/** Compute mean of matrix columns
 * @param M is the input matrix
 */
cv::Mat Tools::meanCols(cv::Mat &M)
{
  cv::Mat m = cv::Mat(1,M.cols,CV_64F,cv::Scalar(0));
  for(int i=0;i<M.rows;i++){
    for(int j=0;j<M.cols;j++)
      m.at<double>(0,j) +=  M.at<double>(i,j);
  }
  m = m / M.rows;
  return m;
}

/** Compute mean of matrix rows
 * @param M is the input matrix
 */
cv::Mat Tools::meanRows(cv::Mat &M)
{
  cv::Mat m = cv::Mat(M.rows,1,CV_64F,cv::Scalar(0));
  for(int i=0;i<M.rows;i++){
    for(int j=0;j<M.cols;j++)
      m.at<double>(i,0) +=  M.at<double>(i,j);
  }
  m = m / M.cols;
  return m;
}

/** Convert a matrix to a scalar
 * @param M is the input matrix (1x3)
 */
cv::Scalar Tools::toScalar(cv::Mat M)
{
  cv::Scalar s(get(M,0,0),get(M,0,1),get(M,0,2));
  return s;
}

/** Convert a double to a std::string
 * @param M is the input matrix (1x3)
 */
std::string Tools::toStr(double d)
{
    std::stringstream ss;
    ss << d;
    return ss.str();
}

/** Create a rotation matrix on X axis
 * @param x is the rotation (radian)
 */
cv::Mat Tools::toRotationMatrixX(double x)
{
  cv::Mat R, Rrod = cv::Mat::zeros(3,1,CV_64F);
  Rrod.at<double>(0,0) = x;
  cv::Rodrigues(Rrod,R);
  return R;
}

/** Create a rotation matrix on Y axis
 * @param y is the rotation (radian)
 */
cv::Mat Tools::toRotationMatrixY(double y)
{
  cv::Mat R, Rrod = cv::Mat::zeros(3,1,CV_64F);
  Rrod.at<double>(1,0) = y;
  cv::Rodrigues(Rrod,R);
  return R;
}

/** Create a rotation matrix on Z axis
 * @param z is the rotation (radian)
 */
cv::Mat Tools::toRotationMatrixZ(double z)
{
  cv::Mat R, Rrod = cv::Mat::zeros(3,1,CV_64F);
  Rrod.at<double>(2,0) = z;
  cv::Rodrigues(Rrod,R);
  return R;
}

/** Create a rotation matrix (Rx*Ry*Rz)
 * @param x,y,z are rotations (radian)
 */
cv::Mat Tools::toRotationMatrix(double x, double y, double z)
{
  cv::Mat R = cv::Mat::eye(3,3,CV_64F);
  if(x!=0.0) R *= toRotationMatrixX(x);
  if(y!=0.0) R *= toRotationMatrixY(y);
  if(z!=0.0) R *= toRotationMatrixZ(z);
  return R;
}

/** Create a translation matrix
 * @param x,y,z are displacements
 */
cv::Mat Tools::toTranslationMatrix(double x, double y, double z)
{
  cv::Mat t(3,1,CV_64F);
  t.at<double>(0,0) = x;
  t.at<double>(1,0) = y;
  t.at<double>(2,0) = z;
  return t;
}

/** Compute Rodrigues formula on rotation vector (x,y,z)
 * @param x,y,z is the input rotation vector
 */
cv::Mat Tools::Rodrigues(double x, double y, double z)
{
  cv::Mat out;
  cv::Mat M(3,1,CV_64F);
  set(M,0,0,x);
  set(M,1,0,y);
  set(M,2,0,z);
  cv::Rodrigues(M,out);
  return out;
}

/** Compute Rodrigues formula on rotation matrix
 * @param M is the input rotation matrix
 */
cv::Mat Tools::Rodrigues(cv::Mat M)
{
  cv::Mat out;
  cv::Rodrigues(M,out);
  return out;
}

/** Compute Rodrigues formula on rotation vector (x,y,z)
 * @param x,y,z is the input rotation vector
 */
cv::Mat Tools::quaternion(double a, double b, double c, double d)
{
  return createMat33(a*a+b*b-c*c-d*d, 2*b*c-2*a*d, 2*b*d+2*a*c,
                     2*b*c+2*a*d, a*a-b*b+c*c-d*d, 2*c*d-2*a*b,
                     2*b*d-2*a*c, 2*c*d+2*a*b, a*a-b*b-c*c+d*d);
}

/** Create a 3x3 matrix
 * @param v1,v2,v3,v4,v5,v6,v7,v8,v9 are input values
 */
cv::Mat Tools::createMat33(double v1, double v2, double v3, double v4, double v5, double v6, double v7, double v8, double v9)
{
  cv::Mat M(3,3,CV_64F);
  M.at<double>(0,0) = v1;
  M.at<double>(0,1) = v2;
  M.at<double>(0,2) = v3;
  M.at<double>(1,0) = v4;
  M.at<double>(1,1) = v5;
  M.at<double>(1,2) = v6;
  M.at<double>(2,0) = v7;
  M.at<double>(2,1) = v8;
  M.at<double>(2,2) = v9;
  return M;
}

/** Create a 3x1 matrix
 * @param v1,v2,v3 are input values
 */
cv::Mat Tools::createMat31(double v1, double v2, double v3)
{
  cv::Mat M(3,1,CV_64F);
  M.at<double>(0,0) = v1;
  M.at<double>(1,0) = v2;
  M.at<double>(2,0) = v3;
  return M;
}

/** Create a 4x1 matrix
 * @param v1,v2,v3,v4 are input values
 */
cv::Mat Tools::createMat41(double v1, double v2, double v3, double v4)
{
  cv::Mat M(4,1,CV_64F);
  M.at<double>(0,0) = v1;
  M.at<double>(1,0) = v2;
  M.at<double>(2,0) = v3;
  M.at<double>(3,0) = v4;
  return M;
}

/** Set a value in an existing matrix
 * @param M is the matrix
 * @param r,c are respectively row and column
 * @param val is the double to set
 */
void Tools::set(cv::Mat &M, int r, int c, double val)
{
  M.at<double>(r,c) = val;
}

/** Get a value of an existing matrix
 * @param M is the matrix
 * @param r,c are respectively row and column
 * @retval M(r,c)
 */
double Tools::get(cv::Mat &M, int r, int c)
{
  double val = M.at<double>(r,c);
  return val;
}

/** Check if a vector contains a specific integer
 * @param v is the input vector
 * @param n is the input integer
 * @return True if n is include in v
 */
bool Tools::contains(std::vector<int> &v, int n)
{
  for(int i=0;i<v.size();i++)
    if(v.at(i)==n)
      return true;
  return false;
}

/** Get a subset of arrays
 * @param v1,v2 are input vectors
 * @param o1,o2 are output vectors
 * @param nb is the size of subset, negative numbers clone the vector
 */
void Tools::getSubset(std::vector<cv::Point2d> v1, std::vector<cv::Point2d> &o1, std::vector<cv::Point2d> v2, std::vector<cv::Point2d> &o2, int nb)
{
  assert(v1.size()==v2.size());
  int count = v1.size();
  o1.clear();
  o2.clear();
  if(nb>=count || nb<0){
    o1 = v1;
    o2 = v2;
  }else{
    std::vector<int> temp;
    while(temp.size()<nb){
      int index = rand()%count;
      if(!contains(temp,index))
        temp.push_back(index);
    }
    for(int i=0;i<temp.size();i++){
      o1.push_back( v1.at(temp.at(i)) );
      o2.push_back( v2.at(temp.at(i)) );
    }
  }
}

/** Get a subset of arrays
 * @param v1,v2 are input vectors
 * @param o1,o2 are output vectors
 * @param nb is the size of subset, negative numbers clone the vector
 */
void Tools::getSubset(std::vector<cv::Point3d> v1, std::vector<cv::Point3d> &o1, std::vector<cv::Point3d> v2, std::vector<cv::Point3d> &o2, int nb)
{
  assert(v1.size()==v2.size());
  int count = v1.size();
  o1.clear();
  o2.clear();
  if(nb>=count || nb<0){
    o1 = v1;
    o2 = v2;
  }else{
    std::vector<int> temp;
    while(temp.size()<nb){
      int index = rand()%count;
      if(!contains(temp,index))
        temp.push_back(index);
    }
    for(int i=0;i<temp.size();i++){
      o1.push_back( v1.at(temp.at(i)) );
      o2.push_back( v2.at(temp.at(i)) );
    }
  }
}

/** Get a subset of arrays
 * @param v3,v2 are input vectors
 * @param o3,o2 are output vectors
 * @param nb is the size of subset, negative numbers clone the vector
 */
void Tools::getSubset(std::vector<cv::Point3d> v3, std::vector<cv::Point3d> &o3, std::vector<cv::Point2d> v2, std::vector<cv::Point2d> &o2, int nb)
{
  assert(v3.size()==v2.size());
  int count = v3.size();
  o3.clear();
  o2.clear();
  if(nb>=count || nb<0){
    o3 = v3;
    o2 = v2;
  }else{
    std::vector<int> temp;
    while(temp.size()<nb){
      int index = rand()%count;
      if(!contains(temp,index))
        temp.push_back(index);
    }
    for(int i=0;i<temp.size();i++){
      o3.push_back( v3.at(temp.at(i)) );
      o2.push_back( v2.at(temp.at(i)) );
    }
  }
}

/** Get a subset of arrays
 * @param v3 is input vector
 * @param o3 is output vector
 * @param nb is the size of subset, negative numbers clone the vector
 */
void Tools::getSubset(std::vector<cv::Point3d> v3, std::vector<cv::Point3d> &o3, int nb)
{
  int count = v3.size();
  o3.clear();
  if(nb>=count || nb<0){
    o3 = v3;
  }else{
    std::vector<int> temp;
    while(temp.size()<nb){
      int index = rand()%count;
      if(!contains(temp,index))
        temp.push_back(index);
    }
    for(int i=0;i<temp.size();i++)
      o3.push_back( v3.at(temp.at(i)) );
  }
}

/** Get a subset of arrays
 * @param v2 is input vector
 * @param o2 is output vector
 * @param nb is the size of subset, negative numbers clone the vector
 */
void Tools::getSubset(std::vector<cv::Point2d> v2, std::vector<cv::Point2d> &o2, int nb)
{
  int count = v2.size();
  o2.clear();
  if(nb>=count || nb<0){
    o2 = v2;
  }else{
    std::vector<int> temp;
    while(temp.size()<nb){
      int index = rand()%count;
      if(!contains(temp,index))
        temp.push_back(index);
    }
    for(int i=0;i<temp.size();i++)
      o2.push_back( v2.at(temp.at(i)) );
  }
}

/** Compute the cross-product of two 3D vectors
 * @param u,v are input vectors
 */
cv::Point3d Tools::crossProduct(cv::Point3d u, cv::Point3d v)
{
  double a = u.y*v.z - u.z*v.y;
  double b = u.z*v.x - u.x*v.z;
  double c = u.x*v.y - u.y*v.x;
  return cv::Point3d(a,b,c);
}

/** Compute intersection of a line and a plan
* @param P1,P2 are two point of line
* @param u,v are two vector of plan
* @param P is a point of plan
*/
cv::Point3d Tools::linePlanIntersection(cv::Point3d P1, cv::Point3d P2, cv::Point3d u, cv::Point3d v, cv::Point3d P)
{
  double a = u.y*v.z - u.z*v.y;
  double b = u.z*v.x - u.x*v.z;
  double c = u.x*v.y - u.y*v.x;
  double d = -(a*P.x+b*P.y+c*P.z);

  cv::Point3d dd = P2-P1;
  double k = (-d-a*P1.x-b*P1.y-c*P1.z)/(a*dd.x+b*dd.y+c*dd.z);
  return P1 + k*dd;
}

/** Compute intersection of (p1--p2)-(p3--p4)\n Midpoint method
* @param P1,P2 Two point of line 1
* @param P3,P4 Two point of line 2
*/
cv::Point3d Tools::linesIntersection(cv::Point3d P1, cv::Point3d P2, cv::Point3d P3 ,cv::Point3d P4)
{
  cv::Point3d n,v,u;

  u.x = P2.x - P1.x;
  u.y = P2.y - P1.y;
  u.z = P2.z - P1.z;
  u *= 1.0/cv::norm(u);

  v.x = P4.x - P3.x;
  v.y = P4.y - P3.y;
  v.z = P4.z - P3.z;
  v *= 1.0/cv::norm(v);

  n = crossProduct(u,v);
  if(cv::norm(n)<1e-10){
      //Parallel lines
      return 1.0e100*u;
  }
  n *= 1.0/cv::norm(n);

  //Intersection
  cv::Point3d A = linePlanIntersection(P1,P2,n,v,P3);
  cv::Point3d B = linePlanIntersection(P3,P4,n,u,P1);

  /*std::cout << "O1 : " << P1 << std::endl;
  std::cout << "X1 : " << P2 << std::endl;
  std::cout << "O2 : " << P3 << std::endl;
  std::cout << "X2 : " << P4 << std::endl;
  std::cout << "A : " << A << std::endl;
  std::cout << "B : " << B << std::endl;*/

  /*FILE *fo = fopen("PTS_O.txt","w");
  FILE *fx = fopen("PTS_X.txt","w");
  FILE *fm = fopen("PTS_M.txt","w");
  FILE *fl = fopen("PTS_L.txt","w");
  fprintf(fo,"%f %f %f\n",P1.x,P1.y,P1.z);
  fprintf(fx,"%f %f %f\n",P2.x,P2.y,P2.z);
  fprintf(fo,"%f %f %f\n",P3.x,P3.y,P3.z);
  fprintf(fx,"%f %f %f\n",P4.x,P4.y,P4.z);
  fprintf(fm,"%f %f %f\n",A.x,A.y,A.z);
  fprintf(fm,"%f %f %f\n",B.x,B.y,B.z);
  fprintf(fl,"%f %f %f\n",P1.x,P1.y,P1.z);
  fprintf(fl,"%f %f %f\n",P2.x,P2.y,P2.z);
  fprintf(fl,"%f %f %f\n\n\n",A.x,A.y,A.z);
  fprintf(fl,"%f %f %f\n",P3.x,P3.y,P3.z);
  fprintf(fl,"%f %f %f\n",P4.x,P4.y,P4.z);
  fprintf(fl,"%f %f %f\n\n\n",B.x,B.y,B.z);
  fclose(fo);
  fclose(fx);
  fclose(fm);
  fclose(fl);*/
  /*std::cout << P1.x << " " << P1.y << " " << P1.z << std::endl;
  std::cout << P2.x << " " << P2.y << " " << P2.z << std::endl;
  std::cout << P3.x << " " << P3.y << " " << P3.z << std::endl;
  std::cout << P4.x << " " << P4.y << " " << P4.z << std::endl;
  std::cout << A.x << " " << A.y << " " << A.z << std::endl;
  std::cout << B.x << " " << B.y << " " << B.z << std::endl;

  std::cout << "Distance : " << cv::norm(A-B) << std::endl;*/

  return (A+B)*0.5;
}

/** Normalize points coordinates
 * @param pts1,pts2 are input points vectors
 * @param K1,K2 are input calibration matrices
 * @param pts1n,pts2n are output points vectors
 */
void Tools::normalize(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &K1, cv::Mat &K2, std::vector<cv::Point2d> &pts1n, std::vector<cv::Point2d> &pts2n)
{
  cv::Mat K1i = K1.inv();
  cv::Mat K2i = K2.inv();
  pts1n.clear();
  pts2n.clear();
  for(unsigned int i=0;i<pts1.size();i++){
    cv::Mat pt1 = toHomogeneous(pts1.at(i));
    cv::Mat pt2 = toHomogeneous(pts2.at(i));
    cv::Mat newpt1 = K1i * pt1;
    cv::Mat newpt2 = K2i * pt2;
    pts1n.push_back(cv::Point2d(newpt1.at<double>(0,0),newpt1.at<double>(1,0)));
    pts2n.push_back(cv::Point2d(newpt2.at<double>(0,0),newpt2.at<double>(1,0)));
  }
}

/** Normalize points coordinates
 * @param pts is input points vector
 * @param K is input calibration matrix
 * @param ptsn is output points vector
 */
void Tools::normalize(std::vector<cv::Point2d> &pts, cv::Mat &K, std::vector<cv::Point2d> &ptsn)
{
  cv::Mat M1 = toHomogeneous(pts);
  cv::Mat Mout = K.inv() * M1.t();
  ptsn.clear();
  for(unsigned int i=0;i<pts.size();i++){
    ptsn.push_back(cv::Point2d(Mout.at<double>(0,i),Mout.at<double>(1,i)));
  }
}

/** Normalize points coordinates
 * @param pts is input points vector
 * @param K is input calibration matrix
 * @param ptsn is output points matrix
 */
void Tools::normalize(std::vector<cv::Point2d> &pts, cv::Mat &K, cv::Mat &ptsn)
{
  cv::Mat Ki = K.inv();
  ptsn = cv::Mat(pts.size(),3,CV_64F);
  for(unsigned int i=0;i<pts.size();i++){
    cv::Mat pt = toHomogeneous(pts.at(i));
    cv::Mat newpt = Ki * pt;
    ptsn.at<double>(i,0) = newpt.at<double>(0,0);
    ptsn.at<double>(i,1) = newpt.at<double>(1,0);
    ptsn.at<double>(i,2) = 1.0;
  }
}

/** Normalize points coordinates
 * @param Min is input points matrix
 * @param K is input calibration matrix
 * @param Mout is output points matrix
 */
void Tools::normalize(cv::Mat Min, cv::Mat &K, cv::Mat &Mout)
{
  cv::Mat Ki;
  cv::invert(K,Ki);
  Mout = cv::Mat(Min.rows,3,CV_64F);
  for(int i=0;i<Min.rows;i++){
    cv::Mat pt = Min.row(i).clone().t();
    cv::Mat newpt;
    multiply(Ki,pt,newpt);
    Mout.at<double>(i,0) = newpt.at<double>(0,0);
    Mout.at<double>(i,1) = newpt.at<double>(1,0);
    Mout.at<double>(i,2) = 1.0;
  }
}

/** Normalize points coordinates
 * @param pts is input points vector
 * @param K is input calibration matrix
 */
cv::Mat Tools::normalize(std::vector<cv::Point2d> &pts, cv::Mat &K)
{
  cv::Mat matPts = toHomogeneous(pts);
  return K.inv() * matPts;
}


int Tools::getNearestPoint3D(std::vector<cv::Point3d> &l, cv::Point3d &p)
{
  assert(l.size()>0);
  int nearest = 0;
  double smallest_norm = cv::norm(p-l.at(0));

  for(int i=1;i<l.size();i++){
    double n = cv::norm(p-l.at(i));
    if(n<smallest_norm){
      nearest = i;
      smallest_norm = n;
    }
  }
  return nearest;
}

int Tools::getNearestPoint2D(std::vector<cv::Point2d> &l, cv::Point2d &p)
{
  assert(l.size()>0);
  int nearest = 0;
  double smallest_norm = cv::norm(p-l.at(0));

  for(int i=1;i<l.size();i++){
    double n = cv::norm(p-l.at(i));
    if(n<smallest_norm){
      nearest = i;
      smallest_norm = n;
    }
  }
  return nearest;
}

cv::Mat Tools::unwarp(cv::Mat &img, cv::Point2d center, double R0, double R1, double theta, double phi, int w, int h, FLIPMODE flip_mode, cv::OutputArray mapx_out, cv::OutputArray mapy_out)
{
  cv::Mat mapx(h,w,CV_32F);
  cv::Mat mapy(h,w,CV_32F);
  if( mapx_out.needed() ){
    mapx_out.create(h,w,CV_32F);
    mapx = mapx_out.getMat();
  }
  if( mapy_out.needed() ){
    mapy_out.create(h,w,CV_32F);
    mapy = mapy_out.getMat();
  }
  for(int m=0;m<w;m++){
    for(int n=0;n<h;n++){
      double t = -theta - (double)m*(phi)/(double)w;
      double r = R0 + (double)n*(R1-R0)/(double)h;
      mapx.at<float>(n,m) = center.x + r*cos(t);
      mapy.at<float>(n,m) = center.y + r*sin(t);
    }
  }
  cv::Mat out;
  cv::remap(img,out,mapx,mapy,CV_INTER_LINEAR);
  if(flip_mode==NONE) return out;
  cv::flip(out,out,flip_mode);
  return out;
}
