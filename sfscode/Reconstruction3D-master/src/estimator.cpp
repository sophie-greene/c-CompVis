#include "../include/estimator.h"

/** Set the number of point use to compute the model
 * @param nbPoints is the number of points
 */
Estimator::Estimator(int nbPoints)
{
  srand(time(NULL));
  modelPoints = nbPoints;
  rng = cvRNG(rand());
  checkPartialSubsets = true;
}

Estimator::~Estimator()
{

}

/** Set the number of point use to compute the model
 * @param nbPoints is the new number of points
 */
void Estimator::setPointNumber(int nbPoints)
{
  modelPoints = nbPoints;
}

/** Compute the retro-projection error
 * @param m1,m2 are input points arrays
 * @param model is the input Fundamental matrix
 * @param err is the output matrix of errors
 */
void Estimator::computeReprojError( cv::Mat &m1, cv::Mat &m2, cv::Mat &model, cv::Mat &err )
{
  int i, count = m1.rows;

  //display(count,"count");
  //double *F = (double*)model.data;

  double F[9];
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      F[3*i+j] = model.at<double>(i,j);

  err = cv::Mat(count,1,CV_64F);

  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));
    double a, b, c, d1, d2, s1, s2;

    a = F[0]*pt1.x + F[1]*pt1.y + F[2];
    b = F[3]*pt1.x + F[4]*pt1.y + F[5];
    c = F[6]*pt1.x + F[7]*pt1.y + F[8];

    s2 = 1./(a*a + b*b);
    d2 = pt2.x*a + pt2.y*b + c;

    a = F[0]*pt2.x + F[3]*pt2.y + F[6];
    b = F[1]*pt2.x + F[4]*pt2.y + F[7];
    c = F[2]*pt2.x + F[5]*pt2.y + F[8];

    s1 = 1./(a*a + b*b);
    d1 = pt1.x*a + pt1.y*b + c;

    double D1 = d1*d1*s1;
    double D2 = d2*d2*s2;
    err.at<double>(i,0) = D1>D2?D1:D2;
    //display(pt1.x,pt1.y,err.at<double>(i,0),"error");
  }
}

/** Count the number of inliers
 * @param m1,m2 are input points arrays
 * @param M is the input Fundamental matrix
 * @param err is the ouput matrix of errors
 * @param reprojThreshold is the input reprojection threshold
 * @retval The number of inliers
 */
int Estimator::findInliers(cv::Mat &m1, cv::Mat &m2, cv::Mat &M, cv::Mat &err, double reprojThreshold)
{
  int count = m1.rows, goodCount = 0;
  computeReprojError( m1, m2, M, err );
  reprojThreshold *= reprojThreshold;

  for(int i = 0; i < count; i++ ){
    if(err.at<double>(i,0)<=reprojThreshold){
      goodCount++;
    }
  }
  //double min,max;
  //cv::minMaxIdx(err,&min,&max);
  //if(min<1000) std::cout << "min(err)=" << min << std::endl;
  return goodCount;
}

/** Check if an interger is already in a vector
 * @param vec is the input vector
 * @param val is the integer to test
 * @return True is found
 */
bool Estimator::contains(std::vector<int> &vec, int &val)
{
  for(unsigned int i=0;i<vec.size();i++){
    if(vec.at(i)==val)
      return true;
  }
  return false;
}

/** Get a random subset for points matrices
 * @param m1,m2 are input matrices
 * @param ms1,ms2 are output subset of m1 and m2
 * @param nb is the size of subsets
 * @return True is found
 */
bool Estimator::getSubset(cv::Mat &m1, cv::Mat &m2, cv::Mat &ms1, cv::Mat &ms2, int nb)
{
  int max = m1.rows;
  std::vector<int> indexes;

  ms1 = cv::Mat(modelPoints,m1.cols,CV_64F);
  ms2 = cv::Mat(modelPoints,m2.cols,CV_64F);

  for(int i=0;i<modelPoints;i++){
    bool found = false;
    //Try nb times to get a point
    for(int iter=0; iter < nb && found==false; iter++){
      int index = rand()%max;
      if(!contains(indexes,index)){
	indexes.push_back(index);

	for(int j=0;j<m1.cols;j++)
	  ms1.at<double>(i,j) = m1.at<double>(index,j);
	for(int j=0;j<m2.cols;j++)
	  ms2.at<double>(i,j) = m2.at<double>(index,j);

	//if( checkPartialSubsets && (checkSubset( ms1, i+1 ) && checkSubset( ms2, i+1 )))
	found = true;
      }
    }
    if(!found)
      return false;
  }
  return true;
}

/** Check that the i-th selected point does not belong to a line connecting some previously selected points
 * @param m is the input matrix
 * @param count is the index to test
 * @return True is good
 */
bool Estimator::checkSubset( cv::Mat &m, int count )
{
  int j, k, i, i0, i1;

  if( checkPartialSubsets )
    i0 = i1 = count - 1;
  else
    i0 = 0, i1 = count - 1;

  for( i = i0; i <= i1; i++ )
    {
      // check that the i-th selected point does not belong
      // to a line connecting some previously selected points
      for( j = 0; j < i; j++ )
        {
	  double dx1 = m.at<double>(j,0) - m.at<double>(i,0);
	  double dy1 = m.at<double>(j,1) - m.at<double>(i,1);
	  for( k = 0; k < j; k++ )
            {
	      double dx2 = m.at<double>(k,0) - m.at<double>(i,0);
	      double dy2 = m.at<double>(k,1) - m.at<double>(i,1);
	      if( fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
		break;
            }
	  if( k < j )
	    break;
        }
      if( j < i )
	break;
    }

  return i >= i1;
}

/** Compute the Fundamental matrix using two set of points using RANSAC
 * @param m1,m2 are input matrices
 * @param M is the output Fundamental matrix
 * @param reprojThreshold is the input reprojection threshold
 * @param confidence is the input confidence
 * @param maxIters is the input number of iterations
 * @return True is good
 */
bool Estimator::runRANSAC( cv::Mat &m1, cv::Mat &m2, cv::Mat &M, double reprojThreshold, double confidence, int maxIters)
{
  cv::Mat err;
  return runRANSAC(m1,m2,M,err,reprojThreshold,confidence,maxIters);
}

/** Compute the Fundamental matrix using two set of points using RANSAC
 * @param m1,m2 are input matrices
 * @param M is the output Fundamental matrix
 * @param err is the output error matrix, square of the real distance
 * @param reprojThreshold is the input reprojection threshold
 * @param confidence is the input confidence
 * @param maxIters is the input number of iterations
 * @return True is good
 */
bool Estimator::runRANSAC( cv::Mat &m1, cv::Mat &m2, cv::Mat &M, cv::Mat &err, double reprojThreshold, double confidence, int maxIters)
{
  std::vector<cv::Mat> models;
  cv::Mat err_min;
  cv::Mat ms1, ms2;

  int iter, niters = maxIters;
  int count = m1.rows;
  int maxGoodCount = 0;

  if( count < modelPoints ){
    std::cout << "Not enough points" << std::cout;
    return false;
  }

  for( iter = 0; iter < niters; iter++ ){
    int goodCount, nmodels;

    //Get a random subset of points
    bool found = getSubset( m1, m2, ms1, ms2, 300 );
    if(!found){
      if( iter == 0 ){
	std::cout << "No subset found" << std::cout;
	return false;
      }
      break;
    }

    models.clear();
    //Get several or unique solution
    nmodels = runKernel( ms1, ms2, models );

    //std::cout << "Nb solution : "<<nmodels<<std::endl;
    if( nmodels <= 0 )
      continue;

    //For each solution compute the number of inliers
    for(int i = 0; i < nmodels; i++ ){
      cv::Mat model_i = models.at(i).clone();
      //std::cout << "model_i=" << model_i << std::endl;

      goodCount = findInliers( m1, m2, model_i, err, reprojThreshold );
      //std::cout << "goodCount = "<<goodCount<<std::endl;
      //if(goodCount>0) std::cout << "Found model "<< i << " ("<< goodCount<<")"<<std::endl;

      if( goodCount > MAX(maxGoodCount, modelPoints-1) ){
	maxGoodCount = goodCount;
	niters = updateNumItersRANSAC( confidence,
				       (double)(count - goodCount)/count, modelPoints, niters );
	M = model_i.clone();
	err_min = err.clone();
	//std::cout << "Found M ("<<goodCount<<")\n";
      }
    }
  }
  if(iter == niters )
    std::cout << "Number of iteration max reached" << std::endl;

  //std::cout << "M=\n"<<M<<std::endl;

  if( maxGoodCount <= 0 ){
    std::cout << "0 good points" << std::endl;
    return false;
  }

#if 1
  //Recompute with maximum of inliers
  double reprojThreshold2 = reprojThreshold*reprojThreshold;
  cv::Mat pts1(maxGoodCount,m1.cols,CV_64F);
  cv::Mat pts2(maxGoodCount,m2.cols,CV_64F);
  int k=0;
  for(int i = 0; i < count; i++ ){
    if(err_min.at<double>(i,0)<reprojThreshold2){
      cloneRow(m1,i,pts1,k);
      cloneRow(m2,i,pts2,k);
      k++;
    }
  }
  if(!runNPoint(pts1,pts2,M)){
    std::cout << "0 good points after recompute" << std::endl;
    return false;
  }
#endif

  int goodCount = findInliers( m1, m2, M, err, reprojThreshold );
  std::cout << "maxGoodCount = " << goodCount<<"/"<<maxGoodCount<<" on "<<count <<" pts in "<< iter <<" iterations" << std::endl;

  return true;
}

/** Clone row a of matrix M1 in row b of matrix M2
 * @param M1,M2 are input matrix
 * @param a,b are input rows
 */
void Estimator::cloneRow(cv::Mat &M1, int a, cv::Mat &M2, int b)
{
  for(int i=0;i<M1.cols;i++)
    M2.at<double>(b,i) = M1.at<double>(a,i);
}

/** Compute the Fundamental matrix using two set of points using RANSAC
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @retval Numer of solutions
 */
int Estimator::runKernel(cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M)
{
  return modelPoints == 7 ? run7Point( m1, m2, M ) : run8Point( m1, m2, M );
}

/** Compute the Fundamental matrix using two set of points
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @retval Numer of solutions
 */
int Estimator::run7Point( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M )
{
  //std::cout << "run7Point" << std::endl;
  int n = 0;
  double t0, t1, t2;
  cv::Mat A = cv::Mat( 7, 9, CV_64F );

  // form a linear system: i-th row of A(=a) represents
  // the equation: (pt2, 1)'*F*(pt1, 1) = 0
  for(int i = 0; i < 7; i++ ){
    //Kroneker product
    for(int a=0;a<3;a++)
      for(int b=0;b<3;b++)
	A.at<double>(i,a*3+b) = m1.at<double>(i,b) * m2.at<double>(i,a);
  }

  // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
  // the solution is linear subspace of dimensionality 2.
  // => use the last two singular vectors as a basis of the space
  // (according to SVD properties)
  cv::SVD svd(A,cv::SVD::FULL_UV);

  //cvSVD( &A, &W, 0, &V, CV_SVD_MODIFY_A + CV_SVD_V_T );
  //f1 = v + 7*9;
  //f2 = v + 8*9;

  //double f1[9],f2[9];
  cv::Mat F1(3,3,CV_64F);
  cv::Mat F2(3,3,CV_64F);

  //displayMat(svd.u,"svd.u");
  //displayMat(svd.vt,"svd.vt");

  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      F1.at<double>(i,j) = svd.vt.row(7).at<double>(0,3*i+j);
      F2.at<double>(i,j) = svd.vt.row(8).at<double>(0,3*i+j);
    }
  }

  //displayMat(F1,"F1");
  //displayMat(F2,"F2");

  // f1, f2 is a basis => lambda*f1 + mu*f2 is an arbitrary f. matrix.
  // as it is determined up to a scale, normalize lambda & mu (lambda + mu = 1),
  // so f ~ lambda*f1 + (1 - lambda)*f2.
  // use the additional constraint det(f) = det(lambda*f1 + (1-lambda)*f2) to find lambda.
  // it will be a cubic equation.
  // find c - polynomial coefficients.

  cv::Mat C = cv::Mat(1,4,CV_64F);
  cv::Mat R = cv::Mat(1,3,CV_64F);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      F1.at<double>(i,j) -= F2.at<double>(i,j);

  t0 = F2.at<double>(1,1)*F2.at<double>(2,2) - F2.at<double>(1,2)*F2.at<double>(2,1);
  t1 = F2.at<double>(1,0)*F2.at<double>(2,2) - F2.at<double>(1,2)*F2.at<double>(2,0);
  t2 = F2.at<double>(1,0)*F2.at<double>(2,1) - F2.at<double>(1,1)*F2.at<double>(2,0);

  C.at<double>(0,3) = F2.at<double>(0,0)*t0 - F2.at<double>(0,1)*t1 + F2.at<double>(0,2)*t2;

  C.at<double>(0,2) = F1.at<double>(0,0)*t0 - F1.at<double>(0,1)*t1 + F1.at<double>(0,2)*t2 -
    F1.at<double>(1,0)*(F2.at<double>(0,1)*F2.at<double>(2,2) - F2.at<double>(0,2)*F2.at<double>(2,1)) +
    F1.at<double>(1,1)*(F2.at<double>(0,0)*F2.at<double>(2,2) - F2.at<double>(0,2)*F2.at<double>(2,0)) -
    F1.at<double>(1,2)*(F2.at<double>(0,0)*F2.at<double>(2,1) - F2.at<double>(0,1)*F2.at<double>(2,0)) +
    F1.at<double>(2,0)*(F2.at<double>(0,1)*F2.at<double>(1,2) - F2.at<double>(0,2)*F2.at<double>(1,1)) -
    F1.at<double>(2,1)*(F2.at<double>(0,0)*F2.at<double>(1,2) - F2.at<double>(0,2)*F2.at<double>(1,0)) +
    F1.at<double>(2,2)*(F2.at<double>(0,0)*F2.at<double>(1,1) - F2.at<double>(0,1)*F2.at<double>(1,0));

  t0 = F1.at<double>(1,1)*F1.at<double>(2,2) - F1.at<double>(1,2)*F1.at<double>(2,1);
  t1 = F1.at<double>(1,0)*F1.at<double>(2,2) - F1.at<double>(1,2)*F1.at<double>(2,0);
  t2 = F1.at<double>(1,0)*F1.at<double>(2,1) - F1.at<double>(1,1)*F1.at<double>(2,0);

  C.at<double>(0,1) = F2.at<double>(0,0)*t0 - F2.at<double>(0,1)*t1 + F2.at<double>(0,2)*t2 -
    F2.at<double>(1,0)*(F1.at<double>(0,1)*F1.at<double>(2,2) - F1.at<double>(0,2)*F1.at<double>(2,1)) +
    F2.at<double>(1,1)*(F1.at<double>(0,0)*F1.at<double>(2,2) - F1.at<double>(0,2)*F1.at<double>(2,0)) -
    F2.at<double>(1,2)*(F1.at<double>(0,0)*F1.at<double>(2,1) - F1.at<double>(0,1)*F1.at<double>(2,0)) +
    F2.at<double>(2,0)*(F1.at<double>(0,1)*F1.at<double>(1,2) - F1.at<double>(0,2)*F1.at<double>(1,1)) -
    F2.at<double>(2,1)*(F1.at<double>(0,0)*F1.at<double>(1,2) - F1.at<double>(0,2)*F1.at<double>(1,0)) +
    F2.at<double>(2,2)*(F1.at<double>(0,0)*F1.at<double>(1,1) - F1.at<double>(0,1)*F1.at<double>(1,0));

  C.at<double>(0,0) = F1.at<double>(0,0)*t0 - F1.at<double>(0,1)*t1 + F1.at<double>(0,2)*t2;

  // solve the cubic equation; there can be 1 to 3 roots ...
  n = cv::solveCubic(C,R);
  //n = cvSolveCubic( &coeffs, &roots );

  if( n < 1 || n > 3 )
    return n;

  //std::cout << n << std::endl;

  for(int k = 0; k < n; k++ )
    {
      // for each root form the fundamental matrix
      double lambda = R.at<double>(0,k), mu = 1.;
      double s = F1.at<double>(2,2)*lambda + F2.at<double>(2,2);

      cv::Mat F = cv::Mat(3,3,CV_64F,cv::Scalar(0));

      // normalize each matrix, so that F(3,3) (~fmatrix[8]) == 1
      if( fabs(s) > DBL_EPSILON ){
	mu = 1./s;
	lambda *= mu;
      }

      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  F.at<double>(i,j) = F1.at<double>(i,j)*lambda + F2.at<double>(i,j)*mu;

      if( fabs(s) > DBL_EPSILON ){
	F.at<double>(2,2) = 1.0;
      }else{
	F.at<double>(2,2) = 0.0;
      }

      //displayMat(F,"F");
      M.push_back(F);
    }

  return n;
}

/** Compute the Fundamental matrix using two set of points
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @retval Numer of solutions
 */
int Estimator::run8Point( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M )
{
  //std::cout << "run8Point\n";
  cv::Point2d m0c(0,0), m1c(0,0);
  double t, scale0 = 0, scale1 = 0;

  int i, j, k, count = m1.rows;

  //std::cout << "m1.size=" << m1.rows<<"x"<<m1.cols << std::endl;
  //std::cout << "m2.size=" << m2.rows<<"x"<<m2.cols << std::endl;

  // compute centers and average distances for each of the two point sets
  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));

    double x = pt1.x, y = pt1.y;
    m0c.x += x; m0c.y += y;

    x = pt2.x, y = pt2.y;
    m1c.x += x; m1c.y += y;
  }

  // calculate the normalizing transformations for each of the point sets:
  // after the transformation each set will have the mass center at the coordinate origin
  // and the average distance from the origin will be ~sqrt(2).
  t = 1./count;
  m0c.x *= t; m0c.y *= t;
  m1c.x *= t; m1c.y *= t;

  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));

    double x = pt1.x - m0c.x, y = pt1.y - m0c.y;
    scale0 += sqrt(x*x + y*y);

    x = pt2.x - m1c.x, y = pt2.y - m1c.y;
    scale1 += sqrt(x*x + y*y);
  }

  scale0 *= t;
  scale1 *= t;

  //std::cout << "m1.cols=" << m1.cols << std::endl;
  //std::cout << "count=" << count << std::endl;
  //std::cout << "scale0=" << scale0 << std::endl;
  //std::cout << "scale1=" << scale1 << std::endl;
  if( scale0 < FLT_EPSILON || scale1 < FLT_EPSILON ){
    //std::cout << "scale0 < FLT_EPSILON || scale1 < FLT_EPSILON" << std::endl;
    //assert(scale0 > FLT_EPSILON && scale1 > FLT_EPSILON);
    return 0;
  }

  scale0 = sqrt(2.)/scale0;
  scale1 = sqrt(2.)/scale1;

  cv::Mat A(9,9,CV_64F,cv::Scalar(0));

  // form a linear system Ax=0: for each selected pair of points m1 & m2,
  // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
  // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));
    double x0 = (pt1.x - m0c.x)*scale0;
    double y0 = (pt1.y - m0c.y)*scale0;
    double x1 = (pt2.x - m1c.x)*scale1;
    double y1 = (pt2.y - m1c.y)*scale1;
    double r[9] = { x1*x0, x1*y0, x1, y1*x0, y1*y0, y1, x0, y0, 1 };
    //std::cout << x0 << " " << y0 << " " << x1 << " " << y1 << std::endl;
    for( j = 0; j < 9; j++ )
      for( k = 0; k < 9; k++ )
	A.at<double>(j,k) += r[j]*r[k];
  }

  //std::cout << "A new = " << A <<std::endl;

  cv::Mat V, W;
  cv::eigen(A, W, V);

  //std::cout << "V new =" << V << std::endl;
  //std::cout << "W new =" << W << std::endl;


  for( i = 0; i < 9; i++ ){
    if( fabs(W.at<double>(i,0)) < DBL_EPSILON )
      break;
  }

  if( i < 8 ){
    std::cout << "i < 8" << std::endl;
    return 0;
  }

  //std::cout << "W.size=" << W.rows<<"x"<<W.cols << std::endl;
  //std::cout << "V.size=" << V.rows<<"x"<<V.cols << std::endl;

  // take the last column of v as a solution of Af = 0

  cv::Mat F0 = cv::Mat(3,3,CV_64F);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      F0.at<double>(i,j) = V.row(8).at<double>(3*i+j);

  //cv::Mat F0 = V.col(8);

  //std::cout << "V=" << V << std::endl;
  //displayMat(F0, "F0 new");

  // make F0 singular (of rank 2) by decomposing it with SVD,
  // zeroing the last diagonal element of W and then composing the matrices back.

  // use v as a temporary storage for different 3x3 matrices
  /*W = U = V = TF = F0;
    W.data.db = v;
    U.data.db = v + 9;
    V.data.db = v + 18;
    TF.data.db = v + 27;

    cvSVD( &F0, &W, &U, &V, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
    W.data.db[8] = 0.;*/


  cv::SVD svd(F0);
  cv::Mat Wd = cv::Mat(3,3,CV_64F,cv::Scalar(0));
  Wd.at<double>(0,0) = svd.w.at<double>(0,0);
  Wd.at<double>(1,1) = svd.w.at<double>(1,0);
  Wd.at<double>(2,2) = 0.0;


  //cv::Mat Ut = svd.u.t();
  //displayMat(Ut,"U new");
  //std::cout << "Wd=" << Wd << std::endl;

  // F0 <- U*diag([W(1), W(2), 0])*V'
  //cvGEMM( &U, &W, 1., 0, 0., &TF, CV_GEMM_A_T );
  //cvGEMM( &TF, &V, 1., 0, 0., &F0, 0/*CV_GEMM_B_T*/ );

  F0 = svd.u * Wd * svd.vt;
  //cv::Mat temp;
  //cv::gemm( svd.u, Wd, 1., 0, 0., temp);
  //cv::gemm( temp, svd.vt, 1., 0, 0., F0 );

  // apply the transformation that is inverse
  // to what we used to normalize the point coordinates
  {
    double tt0[9] = { scale0, 0, -scale0*m0c.x, 0, scale0, -scale0*m0c.y, 0, 0, 1 };
    double tt1[9] = { scale1, 0, -scale1*m1c.x, 0, scale1, -scale1*m1c.y, 0, 0, 1 };
    cv::Mat T0(3,3,CV_64F,tt0);
    cv::Mat T1(3,3,CV_64F,tt1);

    // F0 <- T1'*F0*T0
    F0 = T1.t() * F0 * T0;
    //cv::gemm( T1.t(), F0, 1., 0, 0., temp);
    //cv::gemm( temp, T0, 1., 0, 0., F0);

    // make F(3,3) = 1
    double scale = F0.at<double>(2,2);
    //std::cout << scale << std::endl;
    if( fabs(scale) > FLT_EPSILON ){
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  F0.at<double>(i,j) = F0.at<double>(i,j)/scale;
    }
    //cv::divide( scale, F0, F0);

  }
  M.clear();
  M.push_back(F0);
  //std::cout << "F=" << F << std::endl;
  //displayMat(F0,"F0");
  return 1;
}

/** Compute the Fundamental matrix using two set of points
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @retval Numer of solutions
 */
int Estimator::run11Point( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M )
{
  //TODO, reformulate for 11 points
  if(m1.rows<11 || m2.rows<11)
    return 0;
  //std::cout << "run8Point\n";
  cv::Point2d m0c(0,0), m1c(0,0);
  double t, scale0 = 0, scale1 = 0;

  int i, j, k, count = m1.rows;

  //std::cout << "m1.size=" << m1.rows<<"x"<<m1.cols << std::endl;
  //std::cout << "m2.size=" << m2.rows<<"x"<<m2.cols << std::endl;

  // compute centers and average distances for each of the two point sets
  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));

    double x = pt1.x, y = pt1.y;
    m0c.x += x; m0c.y += y;

    x = pt2.x, y = pt2.y;
    m1c.x += x; m1c.y += y;
  }

  // calculate the normalizing transformations for each of the point sets:
  // after the transformation each set will have the mass center at the coordinate origin
  // and the average distance from the origin will be ~sqrt(2).
  t = 1./count;
  m0c.x *= t; m0c.y *= t;
  m1c.x *= t; m1c.y *= t;

  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));

    double x = pt1.x - m0c.x, y = pt1.y - m0c.y;
    scale0 += sqrt(x*x + y*y);

    x = pt2.x - m1c.x, y = pt2.y - m1c.y;
    scale1 += sqrt(x*x + y*y);
  }

  scale0 *= t;
  scale1 *= t;

  //std::cout << "m1.cols=" << m1.cols << std::endl;
  //std::cout << "count=" << count << std::endl;
  //std::cout << "scale0=" << scale0 << std::endl;
  //std::cout << "scale1=" << scale1 << std::endl;
  if( scale0 < FLT_EPSILON || scale1 < FLT_EPSILON ){
    //std::cout << "scale0 < FLT_EPSILON || scale1 < FLT_EPSILON" << std::endl;
    //assert(scale0 > FLT_EPSILON && scale1 > FLT_EPSILON);
    return 0;
  }

  scale0 = sqrt(2.)/scale0;
  scale1 = sqrt(2.)/scale1;

  cv::Mat A(9,9,CV_64F,cv::Scalar(0));

  // form a linear system Ax=0: for each selected pair of points m1 & m2,
  // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
  // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));
    double x0 = (pt1.x - m0c.x)*scale0;
    double y0 = (pt1.y - m0c.y)*scale0;
    double x1 = (pt2.x - m1c.x)*scale1;
    double y1 = (pt2.y - m1c.y)*scale1;
    double r[9] = { x1*x0, x1*y0, x1, y1*x0, y1*y0, y1, x0, y0, 1 };
    //std::cout << x0 << " " << y0 << " " << x1 << " " << y1 << std::endl;
    for( j = 0; j < 9; j++ )
      for( k = 0; k < 9; k++ )
	A.at<double>(j,k) += r[j]*r[k];
  }

  //std::cout << "A new = " << A <<std::endl;

  cv::Mat V, W;
  cv::eigen(A, W, V);

  //std::cout << "V new =" << V << std::endl;
  //std::cout << "W new =" << W << std::endl;


  for( i = 0; i < 9; i++ ){
    if( fabs(W.at<double>(i,0)) < DBL_EPSILON )
      break;
  }

  if( i < 8 ){
    std::cout << "i < 8" << std::endl;
    return 0;
  }

  //std::cout << "W.size=" << W.rows<<"x"<<W.cols << std::endl;
  //std::cout << "V.size=" << V.rows<<"x"<<V.cols << std::endl;

  // take the last column of v as a solution of Af = 0

  cv::Mat F0 = cv::Mat(3,3,CV_64F);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      F0.at<double>(i,j) = V.row(8).at<double>(3*i+j);

  //cv::Mat F0 = V.col(8);

  //std::cout << "V=" << V << std::endl;
  //displayMat(F0, "F0 new");

  // make F0 singular (of rank 2) by decomposing it with SVD,
  // zeroing the last diagonal element of W and then composing the matrices back.

  // use v as a temporary storage for different 3x3 matrices
  /*W = U = V = TF = F0;
    W.data.db = v;
    U.data.db = v + 9;
    V.data.db = v + 18;
    TF.data.db = v + 27;

    cvSVD( &F0, &W, &U, &V, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
    W.data.db[8] = 0.;*/


  cv::SVD svd(F0);
  cv::Mat Wd = cv::Mat(3,3,CV_64F,cv::Scalar(0));
  Wd.at<double>(0,0) = svd.w.at<double>(0,0);
  Wd.at<double>(1,1) = svd.w.at<double>(1,0);
  Wd.at<double>(2,2) = 0.0;


  //cv::Mat Ut = svd.u.t();
  //displayMat(Ut,"U new");
  //std::cout << "Wd=" << Wd << std::endl;

  // F0 <- U*diag([W(1), W(2), 0])*V'
  //cvGEMM( &U, &W, 1., 0, 0., &TF, CV_GEMM_A_T );
  //cvGEMM( &TF, &V, 1., 0, 0., &F0, 0/*CV_GEMM_B_T*/ );
  cv::Mat temp;
  cv::gemm( svd.u, Wd, 1., 0, 0., temp);
  cv::gemm( temp, svd.vt, 1., 0, 0., F0 );

  // apply the transformation that is inverse
  // to what we used to normalize the point coordinates
  {
    double tt0[9] = { scale0, 0, -scale0*m0c.x, 0, scale0, -scale0*m0c.y, 0, 0, 1 };
    double tt1[9] = { scale1, 0, -scale1*m1c.x, 0, scale1, -scale1*m1c.y, 0, 0, 1 };
    cv::Mat T0(3,3,CV_64F,tt0);
    cv::Mat T1(3,3,CV_64F,tt1);

    // F0 <- T1'*F0*T0
    cv::gemm( T1.t(), F0, 1., 0, 0., temp);
    cv::gemm( temp, T0, 1., 0, 0., F0);

    // make F(3,3) = 1
    double scale = F0.at<double>(2,2);
    //std::cout << scale << std::endl;
    if( fabs(scale) > FLT_EPSILON ){
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  F0.at<double>(i,j) = F0.at<double>(i,j)/scale;
    }
    //cv::divide( scale, F0, F0);

  }
  M.clear();
  M.push_back(F0);
  //std::cout << "F=" << F << std::endl;
  //displayMat(F0,"F0");
  return 1;
}

/** Compute the Fundamental matrix using two set of points
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @param nbPoints define the number of points to use
 * @retval No Error
 */
bool Estimator::runNPoint(cv::Mat &m1, cv::Mat &m2, cv::Mat &M, int nbPoints)
{
  cv::Point2d m0c(0,0), m1c(0,0);
  double t, scale0 = 0, scale1 = 0;
  int i, j, k, count = nbPoints;
  int max = MIN(m1.rows,m2.rows);
  if(count<0 || count>max) count = max;

  // compute centers and average distances for each of the two point sets
  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));
    double x = pt1.x, y = pt1.y;
    m0c.x += x; m0c.y += y;
    x = pt2.x, y = pt2.y;
    m1c.x += x; m1c.y += y;
  }

  // calculate the normalizing transformations for each of the point sets:
  // after the transformation each set will have the mass center at the coordinate origin
  // and the average distance from the origin will be ~sqrt(2).
  t = 1./count;
  m0c.x *= t; m0c.y *= t;
  m1c.x *= t; m1c.y *= t;

  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));
    double x = pt1.x - m0c.x, y = pt1.y - m0c.y;
    scale0 += sqrt(x*x + y*y);
    x = pt2.x - m1c.x, y = pt2.y - m1c.y;
    scale1 += sqrt(x*x + y*y);
  }
  scale0 *= t;
  scale1 *= t;

  if( scale0 < FLT_EPSILON || scale1 < FLT_EPSILON ){
    return false;
  }

  scale0 = sqrt(2.)/scale0;
  scale1 = sqrt(2.)/scale1;

  cv::Mat A(9,9,CV_64F,cv::Scalar(0));

  // form a linear system Ax=0: for each selected pair of points m1 & m2,
  // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
  // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
  for( i = 0; i < count; i++ ){
    cv::Point2d pt1(m1.at<double>(i,0),m1.at<double>(i,1));
    cv::Point2d pt2(m2.at<double>(i,0),m2.at<double>(i,1));
    double x0 = (pt1.x - m0c.x)*scale0;
    double y0 = (pt1.y - m0c.y)*scale0;
    double x1 = (pt2.x - m1c.x)*scale1;
    double y1 = (pt2.y - m1c.y)*scale1;
    double r[9] = { x1*x0, x1*y0, x1, y1*x0, y1*y0, y1, x0, y0, 1 };
    for( j = 0; j < 9; j++ )
      for( k = 0; k < 9; k++ )
	A.at<double>(j,k) += r[j]*r[k];
  }

  cv::Mat V, W;
  cv::eigen(A, W, V);

  for( i = 0; i < 9; i++ ){
    if( fabs(W.at<double>(i,0)) < DBL_EPSILON )
      break;
  }

  if( i < 8 ){
    return false;
  }

  // take the last column of v as a solution of Af = 0
  cv::Mat F0 = cv::Mat(3,3,CV_64F);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      F0.at<double>(i,j) = V.row(8).at<double>(3*i+j);

  // make F0 singular (of rank 2) by decomposing it with SVD,
  // zeroing the last diagonal element of W and then composing the matrices back.

  cv::SVD svd(F0);
  cv::Mat Wd = cv::Mat(3,3,CV_64F,cv::Scalar(0));
  Wd.at<double>(0,0) = svd.w.at<double>(0,0);
  Wd.at<double>(1,1) = svd.w.at<double>(1,0);
  Wd.at<double>(2,2) = 0.0;

  cv::Mat temp;
  cv::gemm( svd.u, Wd, 1., 0, 0., temp);
  cv::gemm( temp, svd.vt, 1., 0, 0., F0 );

  // apply the transformation that is inverse
  // to what we used to normalize the point coordinates
  {
    double tt0[9] = { scale0, 0, -scale0*m0c.x, 0, scale0, -scale0*m0c.y, 0, 0, 1 };
    double tt1[9] = { scale1, 0, -scale1*m1c.x, 0, scale1, -scale1*m1c.y, 0, 0, 1 };
    cv::Mat T0(3,3,CV_64F,tt0);
    cv::Mat T1(3,3,CV_64F,tt1);

    // F0 <- T1'*F0*T0
    cv::gemm( T1.t(), F0, 1., 0, 0., temp);
    cv::gemm( temp, T0, 1., 0, 0., F0);

    // make F(3,3) = 1
    double scale = F0.at<double>(2,2);
    if( fabs(scale) > FLT_EPSILON ){
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  F0.at<double>(i,j) = F0.at<double>(i,j)/scale;
    }
  }

  M = F0.clone();
  return true;
}

/** Compute the Fundamental matrix using two set of points
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @param nbPoints define the number of points to use
 * @retval Numner of solutions
 */
bool Estimator::runNPoint(cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M, int nbPoints)
{
  cv::Mat M0;
  M.clear();
  if(runNPoint(m1,m2,M0,nbPoints)){
    M.push_back(M0);
    return true;
  }else{
    return false;
  }
}

/** Compute the Fundamental matrix using two set of points
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @param nbPoints define the number of points to use
 * @retval Numner of solutions
 */
int Estimator::runNPoint2(cv::Mat &m1, cv::Mat &m2, cv::Mat &M, int nbPoints)
{
  int count = m1.rows;
  cv::Mat A(count,9,CV_64F,cv::Scalar(0));
  for(int i = 0; i < count; i++ ){
    cv::Mat p1(1,3,CV_64F);
    cv::Mat p2(1,3,CV_64F);
    p1.at<double>(0,0) = m1.at<double>(i,0);
    p1.at<double>(0,1) = m1.at<double>(i,1);
    p1.at<double>(0,2) = m1.at<double>(i,2);
    p2.at<double>(0,0) = m2.at<double>(i,0);
    p2.at<double>(0,1) = m2.at<double>(i,1);
    p2.at<double>(0,2) = m2.at<double>(i,2);

    cv::Mat r;
    kron(p1,p2,r);

    r.copyTo(A.row(i));
    /*for(int j = 0; j < 9; j++ )
      for(int k = 0; k < 9; k++ )
        A.at<double>(j,k) += r.at<double>(0,j) * r.at<double>(0,k);//r[j]*r[k];*/
  }

  //std::cout << A << std::endl;

  cv::SVD svd(A);
  int index = svd.vt.rows-1;
  cv::Mat v = svd.vt.row(index).t();

  M = v.reshape(1,3);
  return 1;
}

/** Compute the Fundamental matrix using two set of points
 * @param m1,m2 are input points matrices
 * @param M is a vector of solutions
 * @param nbPoints define the number of points to use
 * @retval Numner of solutions
 * @warning Only for perpective cameras
 */
int Estimator::runNPoint2(cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M, int nbPoints)
{
  //TODO, reformulate for nbPoints points
  if(m1.rows<nbPoints || m2.rows<nbPoints)
    return 0;

  cv::Mat m1c2, m2c2;
  cv::Mat m1c3, m2c3;
  double scale1 = 0, scale2 = 0;

  int count = m1.rows;

  // compute centers and average distances for each of the two point sets

  if(m1.cols==3) m1c2 = mean2(m1);
  if(m1.cols==4) m1c3 = mean3(m1);
  if(m2.cols==3) m2c2 = mean2(m2);
  if(m2.cols==4) m2c3 = mean3(m2);

  // calculate the normalizing transformations for each of the point sets:
  // after the transformation each set will have the mass center at the coordinate origin
  // and the average distance from the origin will be ~sqrt(2).

  if(m1.cols==3) scale1 = normalize2(m1,m1c2);
  if(m1.cols==4) scale1 = normalize3(m1,m1c3);
  if(m2.cols==3) scale2 = normalize2(m2,m2c2);
  if(m2.cols==4) scale2 = normalize3(m2,m2c3);

  if( scale1 < FLT_EPSILON || scale2 < FLT_EPSILON ){
    return 0;
  }

  scale1 = sqrt(2.)/scale1;
  scale2 = sqrt(2.)/scale2;

  cv::Mat A(9,9,CV_64F,cv::Scalar(0));

  // form a linear system Ax=0: for each selected pair of points m1 & m2,
  // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
  // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
  for(int i = 0; i < count; i++ ){
    cv::Mat p1(1,3,CV_64F);
    cv::Mat p2(1,3,CV_64F);
    if(m1.cols==3) p1 = m1.row(i);
    if(m2.cols==3) p2 = m2.row(i);
    if(m1.cols==4) {p1.at<double>(0,0) = m1.at<double>(i,0); p1.at<double>(0,1) = m1.at<double>(i,1); p1.at<double>(0,2) = m1.at<double>(i,2);}
    if(m2.cols==4) {p2.at<double>(0,0) = m2.at<double>(i,0); p2.at<double>(0,1) = m2.at<double>(i,1); p2.at<double>(0,2) = m2.at<double>(i,2);}

    cv::Mat mid1(1,3,CV_64F);
    cv::Mat mid2(1,3,CV_64F);

    if(m1.cols==3) {mid1.at<double>(0,0) = m1c2.at<double>(0,0); mid1.at<double>(0,0) = m1c2.at<double>(1,0);}
    if(m1.cols==4) {mid1 = m1c3.t();}
    if(m2.cols==3) {mid2.at<double>(0,0) = m2c2.at<double>(0,0); mid1.at<double>(0,0) = m2c2.at<double>(1,0);}
    if(m2.cols==4) {mid2 = m2c3.t();}

    p1 = (p1 - mid1) * scale1;
    p2 = (p2 - mid2) * scale2;

    if(m1.cols==3) p1.at<double>(0,2) = 1.0;
    if(m2.cols==3) p2.at<double>(0,2) = 1.0;

    cv::Mat r;
    kron(p1,p2,r);
    for(int j = 0; j < 9; j++ )
      for(int k = 0; k < 9; k++ )
	A.at<double>(j,k) += r.at<double>(0,j) * r.at<double>(0,k);//r[j]*r[k];
  }
  cv::Mat V, W;
  cv::eigen(A, W, V);

  int it;
  for(it = 0; it < 9; it++ ){
    if( fabs(W.at<double>(it,0)) < DBL_EPSILON )
      break;
  }

  if(it < 8 ){
    return 0;
  }

  // take the last column of v as a solution of Af = 0
  cv::Mat F0 = cv::Mat(3,3,CV_64F);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      F0.at<double>(i,j) = V.row(8).at<double>(3*i+j);

  // make F0 singular (of rank 2) by decomposing it with SVD,
  // zeroing the last diagonal element of W and then composing the matrices back.

  cv::SVD svd(F0);
  cv::Mat Wd = cv::Mat(3,3,CV_64F,cv::Scalar(0));
  Wd.at<double>(0,0) = svd.w.at<double>(0,0);
  Wd.at<double>(1,1) = svd.w.at<double>(1,0);
  Wd.at<double>(2,2) = 0.0;

  cv::Mat temp;
  cv::gemm( svd.u, Wd, 1., 0, 0., temp);
  cv::gemm( temp, svd.vt, 1., 0, 0., F0 );

  // apply the transformation that is inverse
  // to what we used to normalize the point coordinates
  {
    double *tt1, *tt2;
    if(m1.cols==3){
      double ttt1[9] = { scale1, 0, -scale1*m1c2.at<double>(0,0), 0, scale1, -scale1*m1c2.at<double>(1,0), 0, 0, 1 };
      tt1 = ttt1;
    }
    if(m1.cols==4){
      double ttt1[9] = { scale1, 0, -scale1*m1c3.at<double>(0,0), 0, scale1, -scale1*m1c3.at<double>(1,0), 0, 0, 1 };
      tt1 = ttt1;
    }
    if(m2.cols==3){
      double ttt2[9] = { scale2, 0, -scale2*m2c2.at<double>(0,0), 0, scale2, -scale2*m2c2.at<double>(1,0), 0, 0, 1 };
      tt2 = ttt2;
    }
    if(m2.cols==4){
      double ttt2[9] = { scale2, 0, -scale2*m2c3.at<double>(0,0), 0, scale2, -scale2*m2c3.at<double>(1,0), 0, 0, 1 };
      tt2 = ttt2;
    }

    cv::Mat T1 = cv::Mat(3,3,CV_64F,tt1);
    cv::Mat T2 = cv::Mat(3,3,CV_64F,tt2);

    // F0 <- T2'*F0*T1
    cv::gemm( T1.t(), F0, 1., 0, 0., temp);
    cv::gemm( temp, T2, 1., 0, 0., F0);

    // make F(3,3) = 1
    double scale = F0.at<double>(2,2);
    if( fabs(scale) > FLT_EPSILON ){
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  F0.at<double>(i,j) = F0.at<double>(i,j)/scale;
    }
  }
  M.clear();
  M.push_back(F0);
  return 1;
}

/** Compute de mean of the 2 first element of a matrix
 * @param M is the input matrix n*m (m>=2)
 * @return matrix 2*1 with the means
 */
cv::Mat Estimator::mean2(cv::Mat &M)
{
  cv::Mat m = cv::Mat::zeros(2,1,CV_64F);
  for(int i=0;i<M.rows;i++){
    m.at<double>(0,0) += M.at<double>(i,0);
    m.at<double>(1,0) += M.at<double>(i,1);
  }
  m *= 1./(double)M.rows;
  return m;
}

/** Compute de mean of the 3 first element of a matrix
 * @param M is the input matrix n*m (m>=3)
 * @return matrix 3*1 with the means
 */
cv::Mat Estimator::mean3(cv::Mat &M)
{
  cv::Mat m = cv::Mat::zeros(3,1,CV_64F);
  for(int i=0;i<M.rows;i++){
    m.at<double>(0,0) += M.at<double>(i,0);
    m.at<double>(1,0) += M.at<double>(i,1);
    m.at<double>(2,0) += M.at<double>(i,2);
  }
  m *= 1./(double)M.rows;
  return m;
}

/** Compute de kronecker product of 3 first element of 2 vectors
 * @param M1,M2 are the input matrices 1*n (n>=3)
 * @param M is the output matrix 1*9;
 */
void Estimator::kron(cv::Mat M1, cv::Mat M2, cv::Mat &M)
{
  assert(M1.rows==1 && M2.rows==1);
  M = cv::Mat(1,M1.cols*M2.cols,CV_64F);
  for(int i=0;i<M1.cols;i++)
    for(int j=0;j<M2.cols;j++)
      M.at<double>(0,M2.cols*i+j) = M1.at<double>(0,i) * M2.at<double>(0,j);
}

/** Compute de kronecker product of 3 first element of 2 vectors
 * @param M1,M2 are the input matrices 1*n (n>=3)
 */
cv::Mat Estimator::kron(cv::Mat M1, cv::Mat M2)
{
  assert(M1.rows==1 && M2.rows==1);
  cv::Mat M = cv::Mat(1,M1.cols*M2.cols,CV_64F);
  for(int i=0;i<M1.cols;i++)
    for(int j=0;j<M2.cols;j++)
      M.at<double>(0,M2.cols*i+j) = M1.at<double>(0,i) * M2.at<double>(0,j);
  return M;
}
/** Compute normalization for n*2 matrix
 * @param M is the input matrix n*m (m>=2)
 * @param m is the inpur centroid 2*1;
 * @retval norm
 */
double Estimator::normalize2(cv::Mat &M, cv::Mat m)
{
  double scale = 0.0;
  for(int i = 0; i < M.rows; i++ ){
    double x = M.at<double>(i,0) - m.at<double>(0,0);
    double y = M.at<double>(i,1) - m.at<double>(1,0);
    scale += sqrt(x*x + y*y);
  }
  scale *= 1./(double)M.rows;
  return scale;
}

/** Compute normalization for n*3 matrix
 * @param M is the input matrix n*m (m>=3)
 * @param m is the inpur centroid 3*1;
 * @retval norm
 */
double Estimator::normalize3(cv::Mat &M, cv::Mat m)
{
  double scale = 0.0;
  for(int i = 0; i < M.rows; i++ ){
    double x = M.at<double>(i,0) - m.at<double>(0,0);
    double y = M.at<double>(i,1) - m.at<double>(1,0);
    double z = M.at<double>(i,2) - m.at<double>(2,0);
    scale += sqrt(x*x + y*y +z*z);
  }
  scale *= 1./(double)M.rows;
  return scale;
}

/** Compute the Fundamental matrix using two set of points using Levenberg-Marquardt
 * @param m1,m2 are input matrices
 * @param M is the output Fundamental matrix
 * @param confidence is the input confidence
 * @param maxIters is the input number of iterations
 * @return True is good
 */
bool Estimator::runLMeDS( cv::Mat &m1, cv::Mat &m2, cv::Mat &M, double confidence, int maxIters )
{
  bool result = false;
  displayMat(m1,"m1");
  displayMat(m2,"m2");
  displayMat(M,"M");
  std::cout << confidence << " " << maxIters << std::endl;
  /*
    const double outlierRatio = 0.45;

    cv::Ptr<CvMat> models;
    cv::Ptr<CvMat> ms1, ms2;
    cv::Ptr<CvMat> err;

    int iter, niters = maxIters;
    int count = m1->rows*m1->cols;
    double minMedian = DBL_MAX, sigma;

    CV_Assert( CV_ARE_SIZES_EQ(m1, m2) && CV_ARE_SIZES_EQ(m1, mask) );

    if( count < modelPoints )
    return false;

    models = cvCreateMat( modelSize.height*maxBasicSolutions, modelSize.width, CV_64FC1 );
    err = cvCreateMat( 1, count, CV_32FC1 );

    if( count > modelPoints )
    {
    ms1 = cvCreateMat( 1, modelPoints, m1->type );
    ms2 = cvCreateMat( 1, modelPoints, m2->type );
    }
    else
    {
    niters = 1;
    ms1 = cvCloneMat(m1);
    ms2 = cvCloneMat(m2);
    }

    niters = cvRound(log(1-confidence)/log(1-pow(1-outlierRatio,(double)modelPoints)));
    niters = MIN( MAX(niters, 3), maxIters );

    for( iter = 0; iter < niters; iter++ )
    {
    int i, nmodels;
    if( count > modelPoints )
    {
    bool found = getSubset( m1, m2, ms1, ms2, 300 );
    if( !found )
    {
    if( iter == 0 )
    return false;
    break;
    }
    }

    nmodels = runKernel( ms1, ms2, models );
    if( nmodels <= 0 )
    continue;
    for( i = 0; i < nmodels; i++ )
    {
    CvMat model_i;
    cvGetRows( models, &model_i, i*modelSize.height, (i+1)*modelSize.height );
    computeReprojError( m1, m2, &model_i, err );
    icvSortDistances( err->data.i, count, 0 );

    double median = count % 2 != 0 ?
    err->data.fl[count/2] : (err->data.fl[count/2-1] + err->data.fl[count/2])*0.5;

    if( median < minMedian )
    {
    minMedian = median;
    cvCopy( &model_i, model );
    }
    }
    }

    if( minMedian < DBL_MAX )
    {
    sigma = 2.5*1.4826*(1 + 5./(count - modelPoints))*sqrt(minMedian);
    sigma = MAX( sigma, 0.001 );

    count = findInliers( m1, m2, model, err, mask, sigma );
    result = count >= modelPoints;
    }
  */
  return result;
}

/** Update the number of iteration needed for RANSAC
 * @param p is the input confidence
 * @param ep is the input percentage of false matching
 * @param model_points is the input number of points in the model (7 or 8 for 3*3 matrix)
 * @param max_iters is the input current number of iteration needed
 * @retval The new number of iteration
 */
int Estimator::updateNumItersRANSAC( double p, double ep, int model_points, int max_iters )
{
  if( model_points <= 0 )
    CV_Error( CV_StsOutOfRange, "the number of model points should be positive" );

  p = MAX(p, 0.);
  p = MIN(p, 1.);
  ep = MAX(ep, 0.);
  ep = MIN(ep, 1.);

  // avoid inf's & nan's
  double num = MAX(1. - p, DBL_MIN);
  double denom = 1. - pow(1. - ep,model_points);
  if( denom < DBL_MIN )
    return 0;

  num = log(num);
  denom = log(denom);

  return denom >= 0 || -num >= max_iters*(-denom) ?
    max_iters : (int)(num/denom);
}

/** Display a matrix
 * @param M is the matrix
 * @param name is an optional matrix name
 */
void Estimator::displayMat(cv::Mat M, std::string name){
  if(name.size()>0)
    std::cout << name << " = ";
  std::cout << M << std::endl << std::endl;
}

/** Display a matrix
 * @param M is the matrix
 * @param name is an optional matrix name
 */
void Estimator::displayMat(CvMat *M, std::string name){
  cv::Mat m(M);
  displayMat(m,name);
}

/** Display a matrix size
 * @param M is the matrix
 * @param name is an optional matrix name
 */
void Estimator::displayMatSize(cv::Mat M, std::string name){
  std::cout << name << " size = " << M.rows <<"x"<<M.cols << std::endl << std::endl;
}

/** Display a matrix size
 * @param M is the matrix
 * @param name is an optional matrix name
 */
void Estimator::displayMatSize(CvMat *M, std::string name){
  cv::Mat m(M);
  displayMatSize(m,name);
}

/** Display a double
 * @param n is the double
 * @param name is an optional double name
 */
void Estimator::display(double n, std::string name){
  if(name.size()>0)
    std::cout << name << " = ";
  std::cout << n << std::endl;
}

/** Display a double
 * @param n1,n2 are the doubles
 * @param name is an optional name
 */
void Estimator::display(double n1,double n2, std::string name){
  if(name.size()>0)
    std::cout << name << " = ";
  std::cout << n1 << " " << n2 << std::endl;
}

/** Display a double
 * @param n1,n2,n3 are the doubles
 * @param name is an optional name
 */
void Estimator::display(double n1,double n2,double n3, std::string name){
  if(name.size()>0)
    std::cout << name << " = ";
  std::cout << n1 << " " << n2 << " " << n3 << std::endl;
}


/*
  void Estimator::computeReprojErrorOriginal( const CvMat* _m1, const CvMat* _m2, const CvMat* model, CvMat* _err )
  {
  int i, count = _m1->rows*_m1->cols;
  const CvPoint2D64f* m1 = (const CvPoint2D64f*)_m1->data.ptr;
  const CvPoint2D64f* m2 = (const CvPoint2D64f*)_m2->data.ptr;
  const double* F = model->data.db;
  float* err = _err->data.fl;

  //display(count,"count");

  for( i = 0; i < count; i++ )
  {
  double a, b, c, d1, d2, s1, s2;

  a = F[0]*m1[i].x + F[1]*m1[i].y + F[2];
  b = F[3]*m1[i].x + F[4]*m1[i].y + F[5];
  c = F[6]*m1[i].x + F[7]*m1[i].y + F[8];

  s2 = 1./(a*a + b*b);
  d2 = m2[i].x*a + m2[i].y*b + c;

  a = F[0]*m2[i].x + F[3]*m2[i].y + F[6];
  b = F[1]*m2[i].x + F[4]*m2[i].y + F[7];
  c = F[2]*m2[i].x + F[5]*m2[i].y + F[8];

  s1 = 1./(a*a + b*b);
  d1 = m1[i].x*a + m1[i].y*b + c;

  err[i] = (float)std::max(d1*d1*s1, d2*d2*s2);
  //display(m1[i].x,m1[i].y,err[i],"ERROR");
  }
  }

  int Estimator::findInliersOriginal( const CvMat* m1, const CvMat* m2, const CvMat* model, CvMat* _err, CvMat*, double threshold )
  {
  int i, count = _err->rows*_err->cols, goodCount = 0;
  const float* err = _err->data.fl;
  //uchar* mask = _mask->data.ptr;

  computeReprojErrorOriginal( m1, m2, model, _err );
  threshold *= threshold;
  for( i = 0; i < count; i++ ){
  if(err[i] <= threshold)
  goodCount ++;
  }
  return goodCount;
  }

  bool Estimator::getSubsetOriginal( const CvMat* m1, const CvMat* m2, CvMat* ms1, CvMat* ms2, int maxAttempts )
  {
  cv::AutoBuffer<int> _idx(modelPoints);
  int* idx = _idx;
  int i = 0, j, k, idx_i, iters = 0;
  int type = CV_MAT_TYPE(m1->type), elemSize = CV_ELEM_SIZE(type);
  const int *m1ptr = m1->data.i, *m2ptr = m2->data.i;
  int *ms1ptr = ms1->data.i, *ms2ptr = ms2->data.i;
  int count = m1->cols*m1->rows;

  assert( CV_IS_MAT_CONT(m1->type & m2->type) && (elemSize % sizeof(int) == 0) );
  elemSize /= sizeof(int);

  for(; iters < maxAttempts; iters++)
  {
  for( i = 0; i < modelPoints && iters < maxAttempts; )
  {
  idx[i] = idx_i = cvRandInt(&rng) % count;
  for( j = 0; j < i; j++ )
  if( idx_i == idx[j] )
  break;
  if( j < i )
  continue;
  for( k = 0; k < elemSize; k++ )
  {
  ms1ptr[i*elemSize + k] = m1ptr[idx_i*elemSize + k];
  ms2ptr[i*elemSize + k] = m2ptr[idx_i*elemSize + k];
  }
  if( checkPartialSubsets && (!checkSubsetOriginal( ms1, i+1 ) || !checkSubsetOriginal( ms2, i+1 )))
  {
  iters++;
  continue;
  }
  i++;
  }
  if( !checkPartialSubsets && i == modelPoints &&
  (!checkSubsetOriginal( ms1, i ) || !checkSubsetOriginal( ms2, i )))
  continue;
  break;
  }

  return i == modelPoints && iters < maxAttempts;
  }

  bool Estimator::checkSubsetOriginal( const CvMat* m, int count )
  {
  int j, k, i, i0, i1;
  CvPoint2D64f* ptr = (CvPoint2D64f*)m->data.ptr;

  assert( CV_MAT_TYPE(m->type) == CV_64FC2 );

  if( checkPartialSubsets )
  i0 = i1 = count - 1;
  else
  i0 = 0, i1 = count - 1;

  for( i = i0; i <= i1; i++ )
  {
  // check that the i-th selected point does not belong
  // to a line connecting some previously selected points
  for( j = 0; j < i; j++ )
  {
  double dx1 = ptr[j].x - ptr[i].x;
  double dy1 = ptr[j].y - ptr[i].y;
  for( k = 0; k < j; k++ )
  {
  double dx2 = ptr[k].x - ptr[i].x;
  double dy2 = ptr[k].y - ptr[i].y;
  if( fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
  break;
  }
  if( k < j )
  break;
  }
  if( j < i )
  break;
  }

  return i >= i1;
  }

  bool Estimator::runRANSACOriginal( const CvMat* m1, const CvMat* m2, CvMat* model, CvMat*, double reprojThreshold, double confidence, int maxIters )
  {
  bool result = false;
  // cv::Ptr<CvMat> mask = cvCloneMat(mask0);
  cv::Ptr<CvMat> models, err;
  cv::Ptr<CvMat> ms1, ms2;

  int iter, niters = maxIters;
  int count = m1->rows*m1->cols, maxGoodCount = 0;
  CV_Assert( CV_ARE_SIZES_EQ(m1, m2));

  if( count < modelPoints )
  return false;

  CvSize modelSize = cvSize(3,3);
  int maxBasicSolutions = 3;
  models = cvCreateMat( modelSize.height*maxBasicSolutions, modelSize.width, CV_64FC1 );
  err = cvCreateMat( 1, count, CV_32FC1 );
  //tmask = cvCreateMat( 1, count, CV_8UC1 );

  if( count > modelPoints )
  {
  ms1 = cvCreateMat( 1, modelPoints, m1->type );
  ms2 = cvCreateMat( 1, modelPoints, m2->type );
  }
  else
  {
  niters = 1;
  ms1 = cvCloneMat(m1);
  ms2 = cvCloneMat(m2);
  }

  for( iter = 0; iter < niters; iter++ )
  {
  int i, goodCount, nmodels;
  if( count > modelPoints )
  {
  bool found = getSubsetOriginal( m1, m2, ms1, ms2, 300 );
  if( !found )
  {
  if( iter == 0 )
  return false;
  break;
  }
  }

  nmodels = runKernelOriginal( ms1, ms2, models );

  if( nmodels <= 0 )
  continue;
  for( i = 0; i < nmodels; i++ )
  {
  CvMat model_i;
  cvGetRows( models, &model_i, i*modelSize.height, (i+1)*modelSize.height );
  goodCount = findInliersOriginal( m1, m2, &model_i, err, 0, reprojThreshold );

  if( goodCount > MAX(maxGoodCount, modelPoints-1) )
  {
  //std::swap(tmask, mask);
  cvCopy( &model_i, model );

  maxGoodCount = goodCount;
  niters = cvRANSACUpdateNumIters( confidence,
  (double)(count - goodCount)/count, modelPoints, niters );
  }
  }
  }

  if( maxGoodCount > 0 ){
  std::cout << "maxGoodCount = " << maxGoodCount << std::endl;
  //if( mask != mask0 )
  //    cvCopy( mask, mask0 );
  result = true;
  }

  return result;
  }

  int Estimator::runKernelOriginal( const CvMat* m1, const CvMat* m2, CvMat* model )
  {
  return modelPoints == 7 ? run7PointOriginal( m1, m2, model ) : run8PointOriginal( m1, m2, model );
  }

  int Estimator::run7PointOriginal( const CvMat* _m1, const CvMat* _m2, CvMat* _fmatrix )
  {
  double a[7*9], w[7], v[9*9], c[4], r[3];
  double* f1, *f2;
  double t0, t1, t2;
  CvMat A = cvMat( 7, 9, CV_64F, a );
  CvMat V = cvMat( 9, 9, CV_64F, v );
  CvMat W = cvMat( 7, 1, CV_64F, w );
  CvMat coeffs = cvMat( 1, 4, CV_64F, c );
  CvMat roots = cvMat( 1, 3, CV_64F, r );
  const CvPoint2D64f* m1 = (const CvPoint2D64f*)_m1->data.ptr;
  const CvPoint2D64f* m2 = (const CvPoint2D64f*)_m2->data.ptr;
  double* fmatrix = _fmatrix->data.db;
  int i, k, n;

  // form a linear system: i-th row of A(=a) represents
  // the equation: (m2[i], 1)'*F*(m1[i], 1) = 0
  for( i = 0; i < 7; i++ )
  {
  double x0 = m1[i].x, y0 = m1[i].y;
  double x1 = m2[i].x, y1 = m2[i].y;

  a[i*9+0] = x1*x0;
  a[i*9+1] = x1*y0;
  a[i*9+2] = x1;
  a[i*9+3] = y1*x0;
  a[i*9+4] = y1*y0;
  a[i*9+5] = y1;
  a[i*9+6] = x0;
  a[i*9+7] = y0;
  a[i*9+8] = 1;
  }

  //displayMat(&A,"A");

  // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
  // the solution is linear subspace of dimensionality 2.
  // => use the last two singular vectors as a basis of the space
  // (according to SVD properties)
  cvSVD( &A, &W, 0, &V, CV_SVD_MODIFY_A + CV_SVD_V_T );
  f1 = v + 7*9;
  f2 = v + 8*9;

  // f1, f2 is a basis => lambda*f1 + mu*f2 is an arbitrary f. matrix.
  // as it is determined up to a scale, normalize lambda & mu (lambda + mu = 1),
  // so f ~ lambda*f1 + (1 - lambda)*f2.
  // use the additional constraint det(f) = det(lambda*f1 + (1-lambda)*f2) to find lambda.
  // it will be a cubic equation.
  // find c - polynomial coefficients.
  for( i = 0; i < 9; i++ )
  f1[i] -= f2[i];

  t0 = f2[4]*f2[8] - f2[5]*f2[7];
  t1 = f2[3]*f2[8] - f2[5]*f2[6];
  t2 = f2[3]*f2[7] - f2[4]*f2[6];

  c[3] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2;

  c[2] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2 -
  f1[3]*(f2[1]*f2[8] - f2[2]*f2[7]) +
  f1[4]*(f2[0]*f2[8] - f2[2]*f2[6]) -
  f1[5]*(f2[0]*f2[7] - f2[1]*f2[6]) +
  f1[6]*(f2[1]*f2[5] - f2[2]*f2[4]) -
  f1[7]*(f2[0]*f2[5] - f2[2]*f2[3]) +
  f1[8]*(f2[0]*f2[4] - f2[1]*f2[3]);

  t0 = f1[4]*f1[8] - f1[5]*f1[7];
  t1 = f1[3]*f1[8] - f1[5]*f1[6];
  t2 = f1[3]*f1[7] - f1[4]*f1[6];

  c[1] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2 -
  f2[3]*(f1[1]*f1[8] - f1[2]*f1[7]) +
  f2[4]*(f1[0]*f1[8] - f1[2]*f1[6]) -
  f2[5]*(f1[0]*f1[7] - f1[1]*f1[6]) +
  f2[6]*(f1[1]*f1[5] - f1[2]*f1[4]) -
  f2[7]*(f1[0]*f1[5] - f1[2]*f1[3]) +
  f2[8]*(f1[0]*f1[4] - f1[1]*f1[3]);

  c[0] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2;

  // solve the cubic equation; there can be 1 to 3 roots ...
  n = cvSolveCubic( &coeffs, &roots );

  if( n < 1 || n > 3 )
  return n;

  for( k = 0; k < n; k++, fmatrix += 9 )
  {
  // for each root form the fundamental matrix
  double lambda = r[k], mu = 1.;
  double s = f1[8]*r[k] + f2[8];

  // normalize each matrix, so that F(3,3) (~fmatrix[8]) == 1
  if( fabs(s) > DBL_EPSILON )
  {
  mu = 1./s;
  lambda *= mu;
  fmatrix[8] = 1.;
  }
  else
  fmatrix[8] = 0.;

  for( i = 0; i < 8; i++ )
  fmatrix[i] = f1[i]*lambda + f2[i]*mu;
  }

  return n;
  }

  int Estimator::run8PointOriginal( const CvMat* _m1, const CvMat* _m2, CvMat* _fmatrix )
  {
  double a[9*9], w[9], v[9*9];
  CvMat W = cvMat( 1, 9, CV_64F, w );
  CvMat V = cvMat( 9, 9, CV_64F, v );
  CvMat A = cvMat( 9, 9, CV_64F, a );
  CvMat U, F0, TF;

  CvPoint2D64f m0c = {0,0}, m1c = {0,0};
  double t, scale0 = 0, scale1 = 0;

  const CvPoint2D64f* m1 = (const CvPoint2D64f*)_m1->data.ptr;
  const CvPoint2D64f* m2 = (const CvPoint2D64f*)_m2->data.ptr;
  double* fmatrix = _fmatrix->data.db;
  CV_Assert( (_m1->cols == 1 || _m1->rows == 1) && CV_ARE_SIZES_EQ(_m1, _m2));
  int i, j, k, count = _m1->cols*_m1->rows;

  // compute centers and average distances for each of the two point sets
  for( i = 0; i < count; i++ )
  {
  double x = m1[i].x, y = m1[i].y;
  m0c.x += x; m0c.y += y;

  x = m2[i].x, y = m2[i].y;
  m1c.x += x; m1c.y += y;
  }

  // calculate the normalizing transformations for each of the point sets:
  // after the transformation each set will have the mass center at the coordinate origin
  // and the average distance from the origin will be ~sqrt(2).
  t = 1./count;
  m0c.x *= t; m0c.y *= t;
  m1c.x *= t; m1c.y *= t;

  for( i = 0; i < count; i++ )
  {
  double x = m1[i].x - m0c.x, y = m1[i].y - m0c.y;
  scale0 += sqrt(x*x + y*y);

  x = m2[i].x - m1c.x, y = m2[i].y - m1c.y;
  scale1 += sqrt(x*x + y*y);
  }

  scale0 *= t;
  scale1 *= t;

  //std::cout << "scale0_origin=" << scale0 << std::endl;
  //std::cout << "scale1_origin=" << scale1 << std::endl;

  if( scale0 < FLT_EPSILON || scale1 < FLT_EPSILON )
  return 0;

  scale0 = sqrt(2.)/scale0;
  scale1 = sqrt(2.)/scale1;

  cvZero( &A );

  // form a linear system Ax=0: for each selected pair of points m1 & m2,
  // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
  // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
  for( i = 0; i < count; i++ )
  {
  double x0 = (m1[i].x - m0c.x)*scale0;
  double y0 = (m1[i].y - m0c.y)*scale0;
  double x1 = (m2[i].x - m1c.x)*scale1;
  double y1 = (m2[i].y - m1c.y)*scale1;
  double r[9] = { x1*x0, x1*y0, x1, y1*x0, y1*y0, y1, x0, y0, 1 };
  //std::cout << x0 << " " << y0 << " " << x1 << " " << y1 << std::endl;
  for( j = 0; j < 9; j++ )
  for( k = 0; k < 9; k++ )
  a[j*9+k] += r[j]*r[k];
  }

  //cv::Mat Atemp(&A);
  //std::cout << "A old = " << Atemp << std::endl;

  cvEigenVV(&A, &V, &W);

  for( i = 0; i < 9; i++ )
  {
  if( fabs(w[i]) < DBL_EPSILON )
  break;
  }

  if( i < 8 )
  return 0;

  F0 = cvMat( 3, 3, CV_64F, v + 9*8 ); // take the last column of v as a solution of Af = 0

  //displayMat(&F0,"F0 old");

  // make F0 singular (of rank 2) by decomposing it with SVD,
  // zeroing the last diagonal element of W and then composing the matrices back.

  // use v as a temporary storage for different 3x3 matrices
  W = U = V = TF = F0;
  W.data.db = v;
  U.data.db = v + 9;
  V.data.db = v + 18;
  TF.data.db = v + 27;

  cvSVD( &F0, &W, &U, &V, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
  W.data.db[8] = 0.;

  //displayMat(&U,"U old");

  // F0 <- U*diag([W(1), W(2), 0])*V'
  cvGEMM( &U, &W, 1., 0, 0., &TF, CV_GEMM_A_T );
  cvGEMM( &TF, &V, 1., 0, 0., &F0, 0 );
  // apply the transformation that is inverse
  // to what we used to normalize the point coordinates
  {
  double tt0[] = { scale0, 0, -scale0*m0c.x, 0, scale0, -scale0*m0c.y, 0, 0, 1 };
  double tt1[] = { scale1, 0, -scale1*m1c.x, 0, scale1, -scale1*m1c.y, 0, 0, 1 };
  CvMat T0, T1;
  T0 = T1 = F0;
  T0.data.db = tt0;
  T1.data.db = tt1;

  // F0 <- T1'*F0*T0
  cvGEMM( &T1, &F0, 1., 0, 0., &TF, CV_GEMM_A_T );
  F0.data.db = fmatrix;
  cvGEMM( &TF, &T0, 1., 0, 0., &F0, 0 );

  //displayMat(&F0, "Ftemp");

  // make F(3,3) = 1
  //std::cout << F0.data.db[8] << std::endl;
  if( fabs(F0.data.db[8]) > FLT_EPSILON )
  cvScale( &F0, &F0, 1./F0.data.db[8] );
  }

  //displayMat(&F0,"F0");
  return 1;
  }
*/
