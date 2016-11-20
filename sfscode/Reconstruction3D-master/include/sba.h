#ifndef SBA_H
#define SBA_H

#include "stdio.h"

#include <math.h>
#include <sba.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include "stdio.h"
#include "../include/tools.h"

#define FULLQUATSZ     4

/* pointers to additional data, used for computed image projections and their jacobians */
struct globs_{
  double *rot0params; /* initial rotation parameters, combined with a local rotation parameterization */
  double *intrcalib;  /* the 5 intrinsic calibration parameters in the order [fu, u0, v0, ar, skew],
                       * where ar is the aspect ratio fv/fu.
                       * Used only when calibration is fixed for all cameras;
                       * otherwise, it is null and the intrinsic parameters are
                       * included in the set of motion parameters for each camera
                       */
  int nccalib;	/* number of calibration parameters that must be kept constant.
                * 0: all parameters are free
                * 1: skew is fixed to its initial value, all other parameters vary (i.e. fu, u0, v0, ar)
                * 2: skew and aspect ratio are fixed to their initial values, all other parameters vary (i.e. fu, u0, v0)
                * 3: meaningless
                * 4: skew, aspect ratio and principal point are fixed to their initial values, only the focal length varies (i.e. fu)
                * 5: all intrinsics are kept fixed to their initial values
                * >5: meaningless
                * Used only when calibration varies among cameras
                */

  int ncdist; /* number of distortion parameters in Bouguet's model that must be kept constant.
               * 0: all parameters are free
               * 1: 6th order radial distortion term (kc[4]) is fixed
               * 2: 6th order radial distortion and one of the tangential distortion terms (kc[3]) are fixed
               * 3: 6th order radial distortion and both tangential distortion terms (kc[3], kc[2]) are fixed [i.e., only 2nd & 4th order radial dist.]
               * 4: 4th & 6th order radial distortion terms and both tangential distortion ones are fixed [i.e., only 2nd order radial dist.]
               * 5: all distortion parameters are kept fixed to their initial values
               * >5: meaningless
               * Used only when calibration varies among cameras and distortion is to be estimated
               */
  int cnp, pnp, mnp; /* dimensions */

  double *ptparams; /* needed only when bundle adjusting for camera parameters only */
  double *camparams; /* needed only when bundle adjusting for structure parameters only */
};

void img_projRT(int j, int i, double *aj, double *xij, void *adata);
void img_projRT_jac(int j, int i, double *aj, double *Aij, void *adata);
void img_projKDRT(int j, int i, double *aj, double *xij, void *adata);
void img_projKDRT_jac(int j, int i, double *aj, double *Aij, void *adata);
void img_projKRT(int j, int i, double *aj, double *xij, void *adata);
void img_projKRT_jac(int j, int i, double *aj, double *Aij, void *adata);

void img_projsRT_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsRT_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKDRT_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKDRT_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKRT_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKRT_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsRTS_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKDRTS_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKDRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKRTS_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
void img_projsKRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);

void calcImgProj(double a[5], double qr0[4], double v[3], double t[3], double M[3], double n[2]);
void calcImgProjFullR(double a[5], double qr0[4], double t[3], double M[3], double n[2]);
void calcImgProjJacKRTS(double a[5], double qr0[4], double v[3], double t[3], double M[3], double jacmKRT[2][11], double jacmS[2][3]);
void calcImgProjJacKRT(double a[5], double qr0[4], double v[3], double t[3], double M[3], double jacmKRT[2][11]);
void calcImgProjJacS(double a[5], double qr0[4], double v[3], double t[3], double M[3], double jacmS[2][3]);
void calcImgProjJacRTS(double a[5], double qr0[4], double v[3], double t[3], double M[3], double jacmRT[2][6], double jacmS[2][3]);
void calcImgProjJacRT(double a[5], double qr0[4], double v[3], double t[3], double M[3], double jacmRT[2][6]);
void calcDistImgProj(double a[5], double kc[5], double qr0[4], double v[3], double t[3], double M[3], double n[2]);
void calcDistImgProjFullR(double a[5], double kc[5], double qr0[4], double t[3], double M[3], double n[2]);
void calcDistImgProjJacKDRTS(double a[5], double kc[5], double qr0[4], double v[3], double t[3], double M[3], double jacmKDRT[2][16], double jacmS[2][3]);
void calcDistImgProjJacKDRT(double a[5], double kc[5], double qr0[4], double v[3], double t[3], double M[3], double jacmKDRT[2][16]);
void calcDistImgProjJacS(double a[5], double kc[5], double qr0[4], double v[3], double t[3], double M[3], double jacmS[2][3]);

class SBA
{
public:
  SBA();
  void computeMotionOnly(std::vector<cv::Point3d> &pts3D, std::vector<std::vector<std::pair<int,cv::Point2d> > > pts2D, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec, cv::Mat K, cv::Mat dist);

private:
  int max_iter;
  int verbose;
  int nframes, numpts3D;
  int nconstframes;
  char *vmask;
  int analyticjac, fixedcal, havedist;
  int cnp, mnp, pnp;
  double *motstruct, *imgpts, *covimgpts, *initrot;
  double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
};

#endif // SBA_H
