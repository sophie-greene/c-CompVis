#include "../include/reconstruction3D_simple.h"

Reconstruction3DSimple::Reconstruction3DSimple()
{

}

void Reconstruction3DSimple::firstBuild(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat K, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D)
{
  assert(pts1.size()==pts2.size());
  assert(pts1.size()>=5);

  cv::Mat F = cv::findFundamentalMat(pts1,pts2);
  cv::Mat E = K.t() * F * K;

  std::vector<cv::Mat> Rvec,tvec;

  /*local_vis_eric::MAlgo5Points m5pts;
  for(int i=0;i<(pts1.size()>255?255:pts1.size());i++){
    std::vector<cv::Point2d> pts1_5,pts2_5;
    for(int i=0;i<5;i++){
        int index = rand()%pts1.size();
        pts1_5.push_back(pts1.at(index));
        pts2_5.push_back(pts2.at(index));
    }
    m5pts.CalculeRT(pts1_5, pts2_5,Rvec,tvec);
  }*/

  findExtrinsicPositions(E,Rvec,tvec);

  findBestExtrinsicPosition(Rvec,tvec,K,pts1,pts2,R,t,pts3D);
}

void Reconstruction3DSimple::findExtrinsicPositions(cv::Mat E, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec)
{
  //E = [t]x R
  //E = u*w*vt = U*D*Vt
  cv::SVD svd(E);
  double w1_[9] = {0,1,0, -1,0,0, 0,0,1};
  double w2_[9] = {0,-1,0, 1,0,0, 0,0,1};
  cv::Mat W1 = cv::Mat(3,3,CV_64F,w1_);
  cv::Mat W2 = cv::Mat(3,3,CV_64F,w2_);

  cv::Mat R1,R2,t1,t2;
  //Compute R
  R1 = svd.u*W1*svd.vt;
  R2 = svd.u*W1.t()*svd.vt;
  if(cv::determinant(R1)<0) {R1 = -svd.u*W2*svd.vt;}
  if(cv::determinant(R2)<0) {R2 = -svd.u*W2.t()*svd.vt;}

  //Compute t
  t1 = svd.u.col(2).clone();
  t2 = -t1.clone();

  Rvec.push_back(R1);  tvec.push_back(t1);
  Rvec.push_back(R1);  tvec.push_back(t2);
  Rvec.push_back(R2);  tvec.push_back(t1);
  Rvec.push_back(R2);  tvec.push_back(t2);
}

void Reconstruction3DSimple::findBestExtrinsicPosition(std::vector<cv::Mat> Rvec, std::vector<cv::Mat> tvec, cv::Mat K, std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D)
{
  assert(Rvec.size()==tvec.size());
  assert(pts1.size()==pts2.size());
  int nb_point = pts1.size();

  std::cout << "Find best solution over " << Rvec.size() << " possibilities" << std::endl;

  int max_good = 0;
  int nb_forward_1_max = 0;
  int nb_forward_2_max = 0;
  double error_min = 10e10;

  double threshold = 5.0;
  cv::Mat I3 = cv::Mat::eye(3,3,CV_64F);
  cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F);


  for(unsigned int i=0;i<Rvec.size();i++){
    int nb_good = 0;
#if 1
    cv::Mat Rcurrent = Rvec.at(i).clone();
    cv::Mat tcurrent = tvec.at(i).clone();
#else
    cv::Mat Rcurrent = Rvec.at(i).t();
    cv::Mat tcurrent = -Rcurrent*tvec.at(i);
#endif
    cv::Mat Rtemp = Rcurrent.t();
    cv::Mat ttemp = -Rcurrent.t()*tcurrent;

    std::vector<cv::Point3d> p3;
#if 1
    triangulate(pts1,pts2,p3,I3,t0,Rcurrent,tcurrent,K);
#else
    triangulate(pts1,pts2,p3,I3,t0,Rtemp,ttemp,K);
#endif

#if 1
    std::vector<cv::Point2d> pts1_reproj = projectPointsSimple(I3,t0,K,p3);
    std::vector<cv::Point2d> pts2_reproj = projectPointsSimple(Rcurrent,tcurrent,K,p3);
#else
#if 1
    std::vector<cv::Point2d> pts1_reproj = projectPoints(I3,t0,K,p3);
    std::vector<cv::Point2d> pts2_reproj = projectPoints(Rcurrent,tcurrent,K,p3);
#else
    std::vector<cv::Point2d> pts1_reproj = projectPoints(I3,t0,K,p3);
    std::vector<cv::Point2d> pts2_reproj = projectPoints(Rtemp,ttemp,K,p3);
#endif
#endif


    double error = 0.0;
    for(unsigned int k=0;k<pts1.size();k++){
      double error1 = cv::norm(pts1_reproj.at(k)-pts1.at(k));
      double error2 = cv::norm(pts2_reproj.at(k)-pts2.at(k));
      error += error1 + error2;
      if( error1<threshold && error2<threshold )
        nb_good++;
    }

    int nb_forward_1 = 0, nb_forward_2 = 0;
    for(unsigned int k=0;k<p3.size();k++){
      if(isForward(I3,t0,p3.at(k)))
        nb_forward_1++;
#if 0
      if(isForward(Rcurrent,tcurrent,p3.at(k)))
#else
      if(isForward(Rtemp,ttemp,p3.at(k)))
#endif
        nb_forward_2++;
    }

    if(nb_good>=5 && /*nb_good>=max_good &&*/ (nb_forward_1 +nb_forward_2) >= (nb_forward_1_max + nb_forward_2_max) /*&& error<=error_min*2.0*/){
      nb_forward_1_max = nb_forward_1;
      nb_forward_2_max = nb_forward_2;
      error_min = error;
#if 0 //DISPLAY
      std::cout << "*" ;
#endif
      pts3D.clear();
      pts3D = p3;
      max_good = nb_good;
#if 0
      R = Rcurrent.clone();
      t = tcurrent.clone();
#else
      R = Rcurrent.t();
      t = - Rcurrent.t() * tcurrent;
#endif
    }

#if 0 //DISPLAY
    std::cout << "Result : Good="<<nb_good<<" Forward=("<<nb_forward_1<<"/"<<nb_forward_2<<") Error="<<error<<std::endl;
    Tools::display(Tools::Rodrigues(Rcurrent),"Rcurrent");
    Tools::display(tcurrent,"tcurrent");
    std::cout << std::endl;
#endif
  }
#if 0 //DISPLAY
  std::cout << "Best match : " << max_good << " points" << std::endl;
#endif
}


bool Reconstruction3DSimple::isForward(cv::Mat R, cv::Mat t, cv::Point3d pt)
{
  cv::Mat Rtemp = R.t();
  cv::Mat ttemp = -R.t()*t;
  cv::Mat P = Tools::concatH(Rtemp,ttemp);
  cv::Mat pt_proj_mat = P * Tools::toHomogeneous(pt);
  cv::Point3d pt_proj = Tools::toPoint3d( pt_proj_mat );
  if(pt_proj.z>0){
    return true;
  }
  return false;
}

void Reconstruction3DSimple::computeExtrinsicParameters(std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, cv::Mat K, cv::Mat &R, cv::Mat &t)
{
  std::cout << "solvePnPRansac with " << pts2D.size() << " points" << std::endl;
  assert(pts2D.size()==pts3D.size());
  assert(pts2D.size()>=5);
  cv::Mat Rvec,tvec;
  std::vector<cv::Point2f> im = Tools::toFloatVector(pts2D);
  std::vector<cv::Point3f> obj = Tools::toFloatVector(pts3D);
  cv::solvePnPRansac(obj, im, K, cv::Mat(), Rvec, tvec, false, 2500, 0.8, pts3D.size()/3);
  R = Tools::Rodrigues(Rvec);
  t = -R.t()*tvec.clone();
  R = R.t();
}

void Reconstruction3DSimple::computeExtrinsicParameters2(std::vector<cv::Point2d> imagePoints, std::vector<cv::Point3d> objectPoints, cv::Mat K, cv::Mat &R, cv::Mat &t, bool useExtrinsicGuess)
{
  assert(objectPoints.size()==imagePoints.size());

  if(t.rows!=3 || t.cols!=1)
    t = cv::Mat::zeros(3,1,CV_64F);

  if(R.rows!=3 || R.cols!=3)
    R = cv::Mat::eye(3,3,CV_64F);

  int count  = objectPoints.size();
  std::cout << "cvFindExtrinsicCameraParams2 with " << count << " points" << std::endl;

  int subsetSize = 5;
  assert(count>=subsetSize);
  if(count<subsetSize){
     R = cv::Mat::eye(3,3,CV_64F);
     t = cv::Mat::zeros(3,1,CV_64F);
     return;
  }

  int maxIter = subsetSize==count?1:5000;

  int nbGood = 0;
  int max_k = 0;
  int k;


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
      Tools::getSubset(objectPoints,objectPoints_temp,imagePoints,imagePoints_temp,subsetSize);

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
          _a[3*i+j] = K.at<double>(i,j);

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

      std::vector<cv::Point2d> reproj = projectPoints(Rtemp, ttemp, K, objectPoints);

      int nb = 0;
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

void Reconstruction3DSimple::triangulate(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0, cv::Mat R1, cv::Mat t1, cv::Mat K)
{
  assert(pts1.size()!=0);
  assert(pts1.size()==pts2.size());
  int numPoints = pts1.size();

  std::vector<cv::Point2d> pts1n, pts2n;
  Tools::normalize(pts1,K,pts1n);
  Tools::normalize(pts2,K,pts2n);

  cv::Mat pts4D;
  cv::Mat pts1mat = Tools::toMat(pts1n).t();
  cv::Mat pts2mat = Tools::toMat(pts2n).t();

  cv::Mat P1 = Tools::concatH(R0,t0);
  cv::Mat P2 = Tools::concatH(R1,t1);
  cv::triangulatePoints(P1,P2,pts1mat,pts2mat,pts4D);

  pts3D.clear();
  for(int i=0;i<numPoints;i++){
    cv::Point3d Q( Tools::get(pts4D,0,i), Tools::get(pts4D,1,i), Tools::get(pts4D,2,i));
    double scale = Tools::get(pts4D,3,i);
    Q *= 1.0/scale;
    pts3D.push_back(Q);
  }
}

void Reconstruction3DSimple::triangulateSimple(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0, cv::Mat R1, cv::Mat t1, cv::Mat K)
{
  assert(pts1.size()!=0);
  assert(pts1.size()==pts2.size());
  int numPoints = pts1.size();

  cv::Mat P1 = Tools::concatH(R0,t0);
  cv::Mat P2 = Tools::concatH(R1,t1);

  cv::Mat A = cv::Mat(6,4,CV_64F);
  // Solve system for each point
  int i,j;
  for( i = 0; i < numPoints; i++ ){
    //Fill matrix for current point
    for( j = 0; j < 2; j++ ){
      double x,y;
      if(j==0){
        x = pts1.at(i).x;
        y = pts1.at(i).y;
        for( int k = 0; k < 4; k++ ){
          A.at<double>(j*3+0, k) = x * P1.at<double>(2,k) -     P1.at<double>(0,k) ;
          A.at<double>(j*3+1, k) = y * P1.at<double>(2,k) -     P1.at<double>(1,k) ;
          A.at<double>(j*3+2, k) = x * P1.at<double>(1,k) - y * P1.at<double>(0,k) ;
        }
      }else{
        x = pts2.at(i).x;
        y = pts2.at(i).y;
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
}
cv::Point3d Reconstruction3DSimple::triangulateOnePoint(cv::Mat R, cv::Mat t, cv::Mat E, cv::Point2d p1, cv::Point2d p2)
{
  double eq0,eq1;
  eq0=E.at<double>(0,0)*p1.x+E.at<double>(0,1)*p1.y+E.at<double>(0,2);
  eq1=E.at<double>(1,0)*p1.x+E.at<double>(1,1)*p1.y+E.at<double>(1,2);
  double c0=-eq1;
  double c1=eq0;
  double c2=p2.x*eq1-p2.y*eq0;

  double C0=R.at<double>(0,0)*c0+R.at<double>(1,0)*c1+R.at<double>(2,0)*c2;
  double C1=R.at<double>(0,1)*c0+R.at<double>(1,1)*c1+R.at<double>(2,1)*c2;
  double C2=R.at<double>(0,2)*c0+R.at<double>(1,2)*c1+R.at<double>(2,2)*c2;
  double C3=t.at<double>(0,0)*c0+t.at<double>(1,0)*c1+t.at<double>(2,0)*c2;

  double s=-(p1.x*C0+p1.y*C1+C2);
  return cv::Point3d(p1.x*C3/s, p1.y*C3/s, C3/s);
}

cv::Point2d Reconstruction3DSimple::projectPoint(cv::Mat R, cv::Mat t, cv::Mat K, cv::Point3d pt3D)
{
  cv::Mat pt = K * Tools::concatH(R,t) * Tools::toHomogeneous(pt3D);
  return Tools::toPoint2d(pt);
}

std::vector<cv::Point2d> Reconstruction3DSimple::projectPoints(cv::Mat R, cv::Mat t, cv::Mat K, std::vector<cv::Point3d> pts3D)
{
  if(pts3D.size()==0)
    return std::vector<cv::Point2d>();

  cv::Mat out;
  cv::Mat Rtemp = R.t();
  cv::Mat ttemp = -R.t()*t;
  cv::projectPoints(pts3D,Rtemp,ttemp,K,cv::Mat(),out);
  //cv::projectPoints(pts3D,Tools::Rodrigues(R),t,K,cv::Mat(),out);

  out = out.reshape(1).clone();
  return Tools::toVect2D(out);
}

std::vector<cv::Point2d> Reconstruction3DSimple::projectPointsSimple(cv::Mat R, cv::Mat t, cv::Mat K, std::vector<cv::Point3d> pts3D)
{
  std::vector<cv::Point2d> pts;
  for(int i=0;i<pts3D.size();i++){
      cv::Point3d pt = pts3D.at(i);
#if 0
      double x,y,z;
      x = pt.x - t.at<double>(0,0);
      y = pt.y - t.at<double>(1,0);
      z = pt.z - t.at<double>(2,0);
      pt.x = R.at<double>(0,0)*x + R.at<double>(0,1)*y + R.at<double>(0,2)*z;
      pt.y = R.at<double>(1,0)*x + R.at<double>(1,1)*y + R.at<double>(1,2)*z;
      pt.z = R.at<double>(2,0)*x + R.at<double>(2,1)*y + R.at<double>(2,2)*z;
#else
      double x = pt.x, y = pt.y, z = pt.z;
      pt.x = R.at<double>(0,0)*x + R.at<double>(0,1)*y + R.at<double>(0,2)*z + t.at<double>(0,0);
      pt.y = R.at<double>(1,0)*x + R.at<double>(1,1)*y + R.at<double>(1,2)*z + t.at<double>(1,0);
      pt.z = R.at<double>(2,0)*x + R.at<double>(2,1)*y + R.at<double>(2,2)*z + t.at<double>(2,0);

#endif
      pt *= 1.0/pt.z;
      cv::Point2d p;
      p.x = K.at<double>(0,0)*pt.x+K.at<double>(0,2);
      p.y = K.at<double>(1,1)*pt.y+K.at<double>(1,2);
      pts.push_back(p);
  }
  return pts;
}

cv::Mat Reconstruction3DSimple::getRetropjImage(cv::Mat img, std::vector<cv::Point2d> pts_input, std::vector<cv::Point2d> pts_reproj, std::vector<bool> good_matching, bool displayAll, bool displayPointID)
{
  cv::Mat out = img.clone();
  int k=0;
  int count = MIN(pts_input.size(),pts_reproj.size());
  for(int i=0; i<count;i++){
    int linesize = 2;
    cv::Scalar linecolor = CV_RGB(255,255,255);
    bool draw = true;
    if(i<good_matching.size()){
      if(good_matching.at(i)){
        if(displayPointID)
          cv::putText(out, Tools::toStr(k),pts_input.at(i)+cv::Point2d(5,0),cv::FONT_HERSHEY_SIMPLEX,0.6,CV_RGB(255,0,0),2);
        linecolor = CV_RGB(0,255,0);
        k++;
      }else{
        linecolor = CV_RGB(255,0,0);
        linesize = 1;
        if(!displayAll)
          draw = false;
      }
    }else{
      if(displayPointID)
        cv::putText(out, Tools::toStr(k),pts_input.at(i)+cv::Point2d(5,0),cv::FONT_HERSHEY_SIMPLEX,0.6,CV_RGB(255,0,0),2);
      k++;
    }
    if(draw){
      cv::circle(out,pts_input.at(i), 3,CV_RGB(0,0,255),-1);
      cv::circle(out,pts_reproj.at(i),3,linecolor,-1);
      cv::line(out,pts_input.at(i),pts_reproj.at(i),linecolor,linesize);
    }
  }
  return out;
}

// This method is the same as icvReconstructPointsFor3View, with only a few numbers adjusted for two-view geometry
void Reconstruction3DSimple::triangulatePoints(cv::Mat projMatr1, cv::Mat projMatr2, std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point3d> &points4D)
{
    int numPoints = pts1.size();

    if( numPoints < 1 )
        CV_Error( CV_StsOutOfRange, "Number of points must be more than zero" );

    if( pts1.size()!=pts2.size() )
        CV_Error( CV_StsUnmatchedSizes, "Number of points must be the same" );

    if( projMatr1.cols != 4 || projMatr1.rows != 3 ||
        projMatr2.cols != 4 || projMatr2.rows != 3)
        CV_Error( CV_StsUnmatchedSizes, "Size of projection matrices must be 3x4" );

    points4D.clear();

    cv::Mat A(6,4,CV_64F);

    std::vector<cv::Point2d> pts[2];
    pts[0] = pts1;
    pts[1] = pts2;

    cv::Mat proj[2];
    proj[0] =  projMatr1;
    proj[1] =  projMatr2;

    /* Solve system for each point */
    for(int i = 0; i < numPoints; i++ )/* For each point */
    {
        /* Fill matrix for current point */
        for(int j = 0; j < 2; j++ )/* For each view */
        {
            double x,y;
            x = pts[j].at(i).x; //cvmGet(projPoints[j],0,i);
            y = pts[j].at(i).y; //cvmGet(projPoints[j],1,i);
            for( int k = 0; k < 4; k++ )
            {
                A.at<double>(j*3+0,k) = x * proj[j].at<double>(k,2)  -     proj[j].at<double>(k,0);
                A.at<double>(j*3+1,k) = y * proj[j].at<double>(k,2)  -     proj[j].at<double>(k,1);
                A.at<double>(j*3+2,k) = x * proj[j].at<double>(k,1)  - y * proj[j].at<double>(k,0);
            }
        }
        /* Solve system for current point */
        {
            cv::SVD svd(A);

#if 0
            double x = svd.vt.at<double>(0,3);
            double y = svd.vt.at<double>(1,3);
            double z = svd.vt.at<double>(2,3);
            double w = svd.vt.at<double>(3,3);
#else
            double x = svd.vt.at<double>(3,0);
            double y = svd.vt.at<double>(3,1);
            double z = svd.vt.at<double>(3,2);
            double w = svd.vt.at<double>(3.3);
#endif

            points4D.push_back(cv::Point3d(x/w,y/w,z/w));
        }
    }
}

int Reconstruction3DSimple::reconstruct(std::vector<std::string> imageList, cv::Mat K, cv::Mat dist, Matcher *matcher, DataManager *dm, int step)
{
  assert(imageList.size()>=2);

  cv::Mat frame1,frame2;
  //Load first image
  frame1 = cv::imread(imageList.at(0));
  //std::cout << "File name : " << imageList.at(0) << std::endl;

  //Undistort first image
  cv::Mat frame1_u;
  cv::undistort(frame1,frame1_u,K,dist);
  frame1 = frame1_u;

  //Variable
  std::vector<PointData> listPointsInPreviousImage;
  int newStep = step;
  cv::Mat I3 = cv::Mat::eye(3,3,CV_64F);
  cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F);
  dm->addPose(Tools::Rodrigues(I3),t0,0,imageList.at(0));

  std::vector<cv::Point2d> pts2D_INPUT;
  std::vector<cv::Point3d> pts3D_INPUT;


  for(unsigned int k=0;k+newStep<imageList.size();){
    //Variable
    std::vector<cv::Point2d> pts1,pts2;
    std::vector<cv::Point3d> pts3D;
    cv::Mat R,t;

    //Load new image
    frame2 = cv::imread(imageList.at(k+newStep));
    //std::cout << "File name : " << imageList.at(k) << std::endl;

    //Undistort new image
    cv::Mat frame2_u;
    cv::undistort(frame2,frame2_u,K,dist);
    frame2 = frame2_u;

    //Timer
    Timer timer;

    //Detect points
    matcher->setImages(frame1,frame2);
    matcher->compute(true,true);
    pts1 = matcher->getPoints(1);
    pts2 = matcher->getPoints(2);

    //Display
    int nb_pairs = pts1.size();
    std::cout << color::Cyan() << "Image " << k+newStep << " : " << nb_pairs << " pairs detected in " << timer.elapsed() << " ms" << color::reset() << std::endl;

    if(pts1.size()<250 && newStep>2){
      newStep = newStep/2;
      std::cout << color::yellow() << "REJECT -----> Divide step (new step " << newStep << ")" << color::reset() << std::endl;
      continue;
    }

    //Previous Pose
    cv::Mat Rbak, tbak;
    dm->getLastPose(Rbak,tbak);
    Rbak = Tools::Rodrigues(Rbak);

    timer.restart();
    //If fisrt image then triangulate, else add new points
    if(k==0){
      //Add First pose to the manager
      firstBuild(pts1,pts2,K,R,t,pts3D);

    }else{
      //Get correspondance with previous reconstruction
      pts2D_INPUT.clear();
      pts3D_INPUT.clear();
      for(int i=0;i<listPointsInPreviousImage.size();i++){
        for( int j=0;j<pts1.size();j++){
          if(listPointsInPreviousImage.at(i).getPoint()==pts1.at(j)){
            pts2D_INPUT.push_back(pts2.at(j));
            pts3D_INPUT.push_back( dm->getPoint3D(listPointsInPreviousImage.at(i).getPtID()) );
          }
        }
      }
      std::cout << "Get " << pts2D_INPUT.size() << " points in " << timer.elapsed() << " ms" << std::endl;

      if(pts3D_INPUT.size()<5){
        newStep = newStep/2;
        std::cout << color::yellow() << "REJECT -----> Divide step (new step " << newStep << ")" << color::reset() << std::endl;
        continue;
      }

      //Compute position of the new image
      computeExtrinsicParameters(pts2D_INPUT,pts3D_INPUT,K,R,t);

      triangulate(pts1,pts2,pts3D, Rbak.t(),-Rbak.t()*tbak, R.t(),-R.t()*t,K);
    }

    //Reproject points
    std::vector<cv::Point2d> pts1_reproj = projectPoints(Rbak,tbak,K,pts3D);
    std::vector<cv::Point2d> pts2_reproj = projectPoints(R,t,K,pts3D);

    //Save lists
    std::vector<PointData> newListPoint;
    std::vector<bool> matches;

    //Variable
    double clip = -1;
    double threshold = 5.0;
    int nb_good = 0;
    timer.restart();
    int previousPoseID = dm->getPreviousPoseID();
    int currentPoseID = dm->getCurrentPoseID();

    //Test if one points is good
    for(unsigned int n=0;n<pts3D.size();n++){
      if( isGood(pts1.at(n),pts1_reproj.at(n),pts2.at(n),pts2_reproj.at(n),threshold, pts3D.at(n),Pose(Rbak,tbak),Pose(R,t)) && (cv::norm(pts3D.at(n))<clip || clip < 0) ){
        int previousPoint3DID = -1;
        for(int i=0;i<listPointsInPreviousImage.size();i++){
          if(listPointsInPreviousImage.at(i).getPoint()==pts1.at(n)){
            previousPoint3DID = listPointsInPreviousImage.at(i).getPtID();
            break;
          }
        }
        if(previousPoint3DID>=0){
          PointData p(pts2.at(n),currentPoseID,previousPoint3DID);
          newListPoint.push_back(p);
        }else{
          int point3DID = dm->addPoint3d(pts3D.at(n));
          PointData p1(pts1.at(n),previousPoseID,point3DID);
          newListPoint.push_back(p1);
          PointData p2(pts2.at(n),currentPoseID,point3DID);
          newListPoint.push_back(p2);
        }

        matches.push_back(true);
        //pm.add(pts1.at(n), pts2.at(n), pts3D.at(n), k);
        nb_good++;
      }else{
        matches.push_back(false);
      }
    }
    std::cout << "Nb good : " << nb_good << std::endl;

    if(nb_good>200 || newStep<=2){
      double ratio = (double)nb_good/(double)pts3D.size();
      std::cout << (ratio<0.5?(ratio<0.25?color::Red():color::Yellow()):color::Green()) << "Add " << nb_good<<"/"<<pts3D.size() << " point3D in " << timer.elapsed() << " ms" << color::reset() << std::endl;

      //Add new pose to the manager
      dm->addPose(Tools::Rodrigues(R),t.clone(),k+newStep,imageList.at(k+newStep));

      for(int i=0;i<newListPoint.size();i++)
        dm->addPoint(newListPoint.at(i));

      //Save list
      listPointsInPreviousImage = newListPoint;

      //Export result
      FILE *x = fopen("video_x.txt","w");
      FILE *y = fopen("video_y.txt","w");
      FILE *z = fopen("video_z.txt","w");

      cv::Mat permute = Tools::toRotationMatrix(M_PI/2,0,0);
      std::vector<cv::Mat> Rvec,tvec;
      dm->getPoses(Rvec,tvec);
      cv::Mat ttot = cv::Mat::zeros(3,1,CV_64F);
      for(unsigned int i=0;i<Rvec.size();i++){
        cv::Mat Rt = Tools::Rodrigues(Rvec.at(i));
        cv::Mat tt = tvec.at(i).clone();
        gnuplot::displayMarker(Rt,tt,x,y,z,true);
        Tools::concatHorizontal(ttot,tvec.at(i),ttot);
      }

      fclose(x);
      fclose(y);
      fclose(z);

      gnuplot::openFile("path.txt");
      ttot = ttot.t()*permute;
      gnuplot::drawPath(ttot.t());
      gnuplot::closeFile();

      gnuplot::openFile("pts3D.txt");
      std::vector<cv::Point3d> pts_out = dm->getPoints3D();
      cv::Mat Rtemp = Tools::toMat(pts_out) * permute;
      pts_out = Tools::toVect3D(Rtemp);
      gnuplot::savePoints(pts_out,clip);
      gnuplot::closeFile();

      gnuplot::draw(FileProcessing::makefilename("result-path/result","png",k,3),"splot './video_x.txt' w l, './video_y.txt' w l, './video_z.txt' w l, './path.txt' w l", 1024,768);
      gnuplot::draw(FileProcessing::makefilename("result-3D/result","png",k,3),"splot './video_x.txt' w l, './video_y.txt' w l, './video_z.txt' w l, './pts3D.txt' w d, './path.txt' w l", 1024,768);

      cv::Mat I1,I2;
      I1 = getRetropjImage(frame1,pts1,pts1_reproj, matches,false);
      I2 = getRetropjImage(frame2,pts2,pts2_reproj, matches,false);

      for(int i=0;i<pts2D_INPUT.size();i++)
          cv::circle(I2,pts2D_INPUT.at(i),2,CV_RGB(255,0,255),-1);

      cv::putText(I2, Tools::toStr(k),cv::Point2d(50,45),cv::FONT_HERSHEY_SIMPLEX,1.5,CV_RGB(255,255,0),2);
      cv::Mat S = matcher->getResult(matches,1280,false);
      cv::Mat I = ImageProcessing::concatImages(I1,I2,1.0,true,1280);

      cv::Mat out = ImageProcessing::concatImages(S,I,1.0,false);
      cv::imwrite(FileProcessing::makefilename("result/result","jpg",k,5),out);

      //Permute images
      frame1 = frame2.clone();
      std::cout << std::endl;

      k+=newStep;
      newStep = step;
    }else{
      newStep = newStep/2;
      std::cout << color::yellow() << "REJECT -----> Divide step (new step " << newStep << ")" << color::reset() << std::endl;
    }

  }

  return 0;
}

bool Reconstruction3DSimple::isGood(cv::Point2d origin1, cv::Point2d reproj1, cv::Point2d origin2, cv::Point2d reproj2, double threshold, cv::Point3d pt3D, Pose pose1, Pose pose2)
{
  if(cv::norm(origin1-reproj1)>threshold) return false;
  if(cv::norm(origin2-reproj2)>threshold) return false;

  if(!isForward(pose1.getR(),pose1.getT(),pt3D)) return false;
  if(!isForward(pose2.getR(),pose2.getT(),pt3D)) return false;

  return true;
}

/*int Reconstruction3DSimple::addImage(cv::Mat &image1, cv::Mat &image2, std::vector<PointData> &lastPoint)
{
  std::vector<cv::Point2d> pts2D_INPUT;
  std::vector<cv::Point3d> pts3D_INPUT;

  //Get correspondance with previous reconstruction
  for(int i=0;i<listPointsInPreviousImage.size();i++){
    for( int j=0;j<pts1.size();j++){
      if(listPointsInPreviousImage.at(i).getPoint()==pts1.at(j)){
        pts2D_INPUT.push_back(pts2.at(j));
        pts3D_INPUT.push_back( dm->getPoint3D(listPointsInPreviousImage.at(i).getPtID()) );
      }
    }
  }
  std::cout << "Get " << pts2D_INPUT.size() << " points in " << timer.elapsed() << " ms" << std::endl;

  //Compute position of the new image
  computeExtrinsicParameters(pts2D_INPUT,pts3D_INPUT,K,R,t);

  triangulate(pts1,pts2,pts3D, Rbak.t(),-Rbak.t()*tbak, R.t(),-R.t()*t,K);

  std::vector<cv::Point2d> pts1_reproj = projectPoints(Rbak,tbak,K,pts3D);
  std::vector<cv::Point2d> pts2_reproj = projectPoints(R,t,K,pts3D);

  //Add new pose to the manager
  int poseID = dm->addPose(Tools::Rodrigues(R),t.clone());

  //Save lists
  std::vector<PointData> newListPoint;
  std::vector<bool> matches;

  //Variable
  double clip = -1;
  double threshold = 5.0;
  int nb_good = 0;
  timer.restart();

  //Test if one points is good
  for(unsigned int n=0;n<pts3D.size();n++){
    if( isGood(pts1.at(n),pts1_reproj.at(n),pts2.at(n),pts2_reproj.at(n),threshold, pts3D.at(n),Pose(Rbak,tbak),Pose(R,t)) && (cv::norm(pts3D.at(n))<clip || clip < 0) ){
      int previousPoint3DID = -1;
      for(int i=0;i<listPointsInPreviousImage.size();i++){
        if(listPointsInPreviousImage.at(i).getPoint()==pts1.at(n)){
          previousPoint3DID = listPointsInPreviousImage.at(i).getPtID();
          break;
        }
      }
      if(previousPoint3DID>=0){
        PointData p(pts2.at(n),poseID,previousPoint3DID);
        dm->addPoint(p);
        newListPoint.push_back(p);
      }else{
        int point3DID = dm->addPoint3d(pts3D.at(n));
        PointData p(pts2.at(n),poseID,point3DID);
        dm->addPoint(p);
        newListPoint.push_back(p);
      }

      matches.push_back(true);
      //pm.add(pts1.at(n), pts2.at(n), pts3D.at(n), k);
      nb_good++;
    }else{
      matches.push_back(false);
    }
  }
  return nb_good;
}*/
