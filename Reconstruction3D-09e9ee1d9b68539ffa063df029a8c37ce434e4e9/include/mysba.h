#ifndef MYSBA_H
#define MYSBA_H

#include <math.h>
#include <sba.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

struct MyData{

};

class MySBA
{
public:
  MySBA();
  void compute();

  static void projFunction(int j, int i, double *aj, double *xij, double*, void *adata);
  static void projFunction_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
  //void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
};

#endif // MYSBA_H
