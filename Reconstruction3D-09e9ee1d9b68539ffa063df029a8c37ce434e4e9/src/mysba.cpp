#include "../include/mysba.h"

MySBA::MySBA()
{
}

void MySBA::compute()
{
  /*extern int
  sba_motstr_levmar(const int n, const int ncon, const int m, const int mcon, char *vmask,
             double *p, const int cnp, const int pnp, double *x, double *covx, const int mnp,
             void (*proj)(int j, int i, double *aj, double *bi, double *xij, void *adata),
             void (*projac)(int j, int i, double *aj, double *bi, double *Aij, double *Bij, void *adata),
             void *adata, const int itmax, const int verbose, const double opts[SBA_OPTSSZ], double info[SBA_INFOSZ]);*/

  /*extern int
  sba_motstr_levmar_x(const int n, const int ncon, const int m, const int mcon, char *vmask, double *p,
             const int cnp, const int pnp, double *x, double *covx, const int mnp,
             void (*func)(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata),
             void (*fjac)(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata),
             void *adata, const int itmax, const int verbose, const double opts[SBA_OPTSSZ], double info[SBA_INFOSZ]);*/

  int nbPoint3D;
  const int nbPoint3DConst = 0;
  int nbFrames;
  const int nbFramesConst = 0;
  char *vmask;
  double *pointDatas;
  int cnp, pnp, mnp;
  double *imagePoints, *imageCov;
  MyData data;
  int maxIter = 50;
  int verbose = 0;
  int numProj = 0;

  double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
  opts[0]=SBA_INIT_MU;
  opts[1]=SBA_STOP_THRESH;
  opts[2]=SBA_STOP_THRESH;
  opts[3]=SBA_STOP_THRESH;
  //opts[3]=0.05*numProj; // uncomment to force termination if the average reprojection error drops below 0.05
  opts[4]=0.0;
  //opts[4]=1E-05; // uncomment to force termination if the relative reduction in the RMS reprojection error drops below 1E-05


  int ret = sba_motstr_levmar(nbPoint3D, nbPoint3DConst, nbFrames, nbFramesConst, vmask, pointDatas, cnp, pnp, imagePoints, imageCov, mnp, projFunction, NULL, (void*)(&data), maxIter, verbose, opts, info);

  fprintf(stdout, "SBA returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals, %d lin. systems\n", ret, info[5], info[6], info[1]/numProj, info[0]/numProj, (int)info[7], (int)info[8], (int)info[9]);
}

void MySBA::projFunction(int j, int i, double *aj, double *xij, double *, void *adata)
{

}
