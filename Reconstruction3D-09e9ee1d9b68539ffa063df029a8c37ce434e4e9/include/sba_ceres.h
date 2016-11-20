#ifndef SBA_CERES_H
#define SBA_CERES_H

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/solver.h"
#include "ceres/rotation.h"
#include "../include/tools.h"

#include "../include/datamanager.h"

class FileReader
{
public:
  FileReader(int nbParm) : nbParm_(nbParm){}
  bool read(std::string filename, std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector<PointData> &points2D) {
    FILE* fptr = fopen(filename.c_str(), "r");
    if (fptr == NULL) {
      return false;
    }

    int num_cameras, num_points, num_observations;
    fscanf(fptr, "%d", &num_cameras);
    fscanf(fptr, "%d", &num_points);
    fscanf(fptr, "%d", &num_observations);

    points3D.clear();
    R.clear();
    t.clear();
    points2D.clear();

    for (int i = 0; i < num_observations; ++i) {
      int camIndex = 0;
      int ptIndex = 0;
      double x,y;
      fscanf(fptr, "%d", &camIndex);
      fscanf(fptr, "%d",&ptIndex);
      fscanf(fptr, "%lf", &x);
      fscanf(fptr, "%lf", &y);

      points2D.push_back( PointData( cv::Point2d(x,y), camIndex, ptIndex) );
    }

    for (int i = 0; i < num_cameras; ++i) {
      double r1,r2,r3, t1,t2,t3;
      fscanf(fptr, "%lf", &r1);
      fscanf(fptr, "%lf", &r2);
      fscanf(fptr, "%lf", &r3);
      fscanf(fptr, "%lf", &t1);
      fscanf(fptr, "%lf", &t2);
      fscanf(fptr, "%lf", &t3);
      for(int j=0;j<nbParm_-6;j++){
        double temp;
        fscanf(fptr, "%lf", &temp);
      }
      /*double focal, d1,d2;
      fscanf(fptr, "%lf", &focal);
      fscanf(fptr, "%lf", &d1);
      fscanf(fptr, "%lf", &d2);
      std::cout << focal << " " << d1 << " " << d2 << std::endl;*/

      cv::Mat Rtemp = Tools::Rodrigues(r1,r2,r3);
      cv::Mat ttemp = Tools::createMat31(t1,t2,t3);

      Tools::display(Tools::createMat31(r1,r2,r3),"R");
      Tools::display(ttemp,"t");

      R.push_back(Rtemp);
      t.push_back(ttemp);
    }
    for (int i = 0; i < num_points; ++i) {
      double x,y,z;
      fscanf(fptr, "%lf", &x);
      fscanf(fptr, "%lf", &y);
      fscanf(fptr, "%lf", &z);
      points3D.push_back( cv::Point3d(x,y,z) );
    }

    fclose(fptr);
    return true;
  }
private:
  int nbParm_;
};

class BA_Data {
 public:
  ~BA_Data() {
    delete[] point_index_;
    delete[] camera_index_;

    delete[] observations_;

    delete[] points_parameters_;
    delete[] cameras_parameters_;
  }

  int num_observations()       const { return num_observations_;               }
  const double* observations() const { return observations_;                   }
  double* mutable_cameras()          { return cameras_parameters_;             }
  double* mutable_points()           { return points_parameters_;              }

  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 6;
  }
  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }

  bool load(std::vector<cv::Point3d> points3D, std::vector<cv::Mat> R, std::vector<cv::Mat> t, std::vector<PointData> points2D, cv::Mat K, cv::Mat dist) {
    assert(R.size()==t.size());

    K_ = K.clone();
    dist_ = dist.clone();

    num_cameras_ = R.size();
    num_points_ = points3D.size();
    num_observations_ = points2D.size();

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    cameras_parameters_ = new double[6 * num_cameras_];
    points_parameters_ = new double[3 * num_points_];

    for (int i = 0; i < num_observations_; ++i) {
      camera_index_[i] = points2D.at(i).getPoseID();
      point_index_[i] = points2D.at(i).getPtID();
      observations_[2*i + 0] = points2D.at(i).getPoint().x;
      observations_[2*i + 1] = points2D.at(i).getPoint().y;
    }

    for (int i = 0; i < num_cameras_; ++i) {
      cv::Mat Rvec, tvec = t.at(i);
      cv::Rodrigues(R.at(i),Rvec);
      cameras_parameters_[i*6 + 0] = Rvec.at<double>(0,0);
      cameras_parameters_[i*6 + 1] = Rvec.at<double>(1,0);
      cameras_parameters_[i*6 + 2] = Rvec.at<double>(2,0);
      cameras_parameters_[i*6 + 3] = tvec.at<double>(0,0);
      cameras_parameters_[i*6 + 4] = tvec.at<double>(1,0);
      cameras_parameters_[i*6 + 5] = tvec.at<double>(2,0);
    }

    for (int i = 0; i < num_points_; ++i) {
      points_parameters_[i*3 + 0] = points3D.at(i).x;
      points_parameters_[i*3 + 1] = points3D.at(i).y;
      points_parameters_[i*3 + 2] = points3D.at(i).z;
    }
    return true;
  }

  void get(std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector<PointData> &points2D) {
    points3D.clear();
    R.clear();
    t.clear();
    points2D.clear();

    for (int i = 0; i < num_cameras_; ++i) {
      cv::Mat Rtemp = Tools::Rodrigues(cameras_parameters_[i*6 + 0],cameras_parameters_[i*6 + 1],cameras_parameters_[i*6 + 2]);
      cv::Mat ttemp = Tools::createMat31(cameras_parameters_[i*6 + 3],cameras_parameters_[i*6 + 4],cameras_parameters_[i*6 + 5]);
      R.push_back(Rtemp);
      t.push_back(ttemp);
    }

    for (int i = 0; i < num_points_; ++i) {
      points3D.push_back( cv::Point3d(points_parameters_[i*3 + 0],points_parameters_[i*3 + 1],points_parameters_[i*3 + 2]) );
    }

    for (int i = 0; i < num_observations_; ++i) {
      cv::Point2d pt = Tools::projPoint(R.at(camera_index_ [i]),t.at(camera_index_ [i]),K_,dist_,points3D.at(point_index_[i]));
      PointData p( pt, camera_index_ [i], point_index_[i]);
      points2D.push_back(p);
    }

  }

 private:
  int num_cameras_;
  int num_points_;
  int num_observations_;
  //int num_parameters_;

  double* points_parameters_;
  double* cameras_parameters_;

  int* point_index_;
  int* camera_index_;
  double* observations_;
  cv::Mat K_,dist_;

};

struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y,const cv::Mat &K,const cv::Mat &dist)
      : observed_x(observed_x), observed_y(observed_y) {   
    camParam_[0] = K.at<double>(0,0);
    camParam_[1] = K.at<double>(1,1);
    camParam_[2] = K.at<double>(0,2);
    camParam_[3] = K.at<double>(1,2);
    K_ = K;
    for(int i=0;i<8;i++)
      distParam[i] = 0.0;
    for(int i=0;i<std::min(8,dist.rows);i++)
      distParam[i] = dist.at<double>(i,0);
    dist_ = dist;

  }

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

#if 0
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply second and fourth order radial distortion.
    //double l1 = camParam[0];
    //double l2 = camParam[1];
    //T r2 = xp*xp + yp*yp;
    //T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    T predicted_x = camParam_[0] * xp + camParam_[2];
    T predicted_y = camParam_[1] * yp + camParam_[3];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
#else
/*
    T X = M[i].x, Y = M[i].y, Z = M[i].z;
    T x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
    T y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
    T z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
*/

    T x = p[0];
    T y = p[1];
    T z = p[2];

    T r2, r4, r6, a1, a2, a3, cdist, icdist2;

    if(z!=T(0.0)){
      x /= z; y /= z;
    }

    T fx = T(camParam_[0]), fy = T(camParam_[1]);
    T cx = T(camParam_[2]), cy = T(camParam_[3]);

    r2 = x*x + y*y;
    r4 = r2*r2;
    r6 = r4*r2;
    a1 = 2.0*x*y;
    a2 = r2 + 2.0*x*x;
    a3 = r2 + 2.0*y*y;
    cdist = 1. + T(distParam[0])*r2 + T(distParam[1])*r4 + T(distParam[4])*r6;
    icdist2 = 1./(1. + T(distParam[5])*r2 + T(distParam[6])*r4 + T(distParam[7])*r6);
    T xd = x*cdist*icdist2 + T(distParam[2])*a1 + T(distParam[3])*a2;
    T yd = y*cdist*icdist2 + T(distParam[2])*a3 + T(distParam[3])*a1;

    T predicted_x = xd*fx + cx;
    T predicted_y = yd*fy + cy;

    /*std::vector<cv::Point3d > input;
    std::vector<cv::Point2d > output;

    input.push_back(cv::Point3d(double(x.a),double(y.a),double(z.a)));

    cv::projectPoints(input,cv::Mat(),cv::Mat(),K_,dist_,output);

    T predicted_x = output.at(0).x;
    T predicted_y = output.at(0).y;*/

    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
#endif
    return true;
  }

  double observed_x;
  double observed_y;
  double camParam_[4];
  double distParam[8];
  cv::Mat K_,dist_;
};

class SBA_CERES
{
public:
  SBA_CERES(std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector<PointData> &points2D, cv::Mat &K, cv::Mat &dist, int iter = 100){

    data.load(points3D,R,t,points2D,K,dist);

    ceres::Problem problem;
    for (int i = 0; i < data.num_observations(); ++i) {
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
              new ReprojectionError(
                  data.observations()[2 * i + 0],
                  data.observations()[2 * i + 1],K,dist));

      problem.AddResidualBlock(cost_function,
                               NULL /* squared loss */,
                               data.mutable_camera_for_observation(i),
                               data.mutable_point_for_observation(i));
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 4;
    options.max_num_iterations = iter;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

  }

  void getResult(std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector<PointData> &points2D)
  {
    data.get(points3D,R,t,points2D);
  }

private:
  double *observations;
  BA_Data data;
};

#endif // SBA_CERES_H
