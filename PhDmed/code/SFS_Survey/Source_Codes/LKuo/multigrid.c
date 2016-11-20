/******************************************************************************/
/*  This is the multigrid implementation for Lee and Kuo's shape from shading */
/*  algorithm, which includes the smoothness constraint.                      */
/******************************************************************************/

/* for cpu timing */
#include <sys/time.h>
#include <sys/resource.h>

#include <stdio.h>
#include <math.h>
#include  "../util/ImageTools.h"
#include "DEF.h"
#include "nrutil.h"
#include  "../util/UCFReadPic.c"
#include  "../util/UCFWritePic.c"

extern double Ei[ARRAY_SIZE][ARRAY_SIZE];	/* intensity         */
int nGrid;
double C[MaxGrids][ARRAY_SQ][13];		/* stiffness matrix  */
double Sx, Sy, Sz;

int IMG_SIZE;
int i_cntr;
/* for cpu timing */
long tol_ts, tol_tm;

main()
{
    int i, h, size;
    double *b, *Z, len;
    int nIter, nVcycle;
    FILE *outf;
    char filename[80];
/* for cpu timing */
struct rusage rusage;
long bts,btm,ets,etm;

    readimg();

    fprintf(stderr, "Sx = ");
    scanf("%lf", &Sx);
    fprintf(stderr, "Sy = ");
    scanf("%lf", &Sy);
    fprintf(stderr, "Sz = ");
    scanf("%lf", &Sz);
    if(Sx == 0.0 && Sy == 0.0) Sx = Sy = 0.001;
    len = sqrt(Sx*Sx + Sy*Sy + Sz*Sz);
    Sx /= len;
    Sy /= len;
    Sz /= len;

    fprintf(stderr, "Number of multigrid iteration = ");
    scanf("%d", &nIter);

    fprintf(stderr, "Number of grids (7 for 256 by 256, 6 for 128 by 128) = ");
    scanf("%d", &nGrid);

    h = 1;
/* for cpu timing */
tol_ts = 0.0;
tol_tm = 0.0;
getrusage(0,&rusage);
bts = rusage.ru_utime.tv_sec;
btm = rusage.ru_utime.tv_usec;


    allocate(&b, IMG_SIZE*IMG_SIZE);
    allocate(&Z, IMG_SIZE*IMG_SIZE);

    /*  For som128 */
    /*
    Z[65*IMG_SIZE+65] = 6.1374210;
    */

    /*  For som140 */
    /*
    Z[70*IMG_SIZE+70] = 6.1374210;
    */

    /*  for sphere 64 by 64
    Z[IMG_SIZE/2*IMG_SIZE+IMG_SIZE/2] = 25.0;
    */
    /*  for sphere 128 by 128
    Z[IMG_SIZE/2*IMG_SIZE+IMG_SIZE/2] = 52.0;
    */
    /*
    load_depth(Z);
    */

    /* Number of multigrids to run */
    for (i_cntr = 0; i_cntr < nIter; i_cntr++) {
	if (i_cntr == 0)
	    nVcycle = 10;
	else if (i_cntr == 1)
	    nVcycle = 2;
        else
	    nVcycle = 1;

        for (i = 0; i < nGrid; i++) {
	    size = IMG_SIZE / (1 << i);
	    calc_C_b(Z, size, i+1, b);
        }

	/* Number of V-cycles in each multigrid */
	for (i = 0; i < nVcycle; i++) {	
	    recursive(h, b, &Z);
/* for cpu timing */
sprintf(filename, "out%02d-%02d.img", i_cntr, i);
create_image(filename, Z, IMG_SIZE);

sprintf(filename, "out%02d-%02d.dep", i_cntr, i);
if ((outf = fopen(filename, "w")) == NULL)
{
    fprintf(stderr, "Couldn't open %s for output\n",filename);
    exit(1);
}

for (i = 0; i < (IMG_SIZE*IMG_SIZE); i++)
{
    fprintf(outf, "%lf\n", Z[i]);
}

fclose(outf);
/* */
        }


	/* ------------  Output, for cpu timing  */
	sprintf(filename, "out%02d.img", i_cntr);
	create_image(filename, Z, IMG_SIZE);

	sprintf(filename, "out%02d.dep", i_cntr);
	if ((outf = fopen(filename, "w")) == NULL)
	{
	    fprintf(stderr, "Couldn't open %s for output\n",filename);
	    exit(1);
	}

	for (i = 0; i < (IMG_SIZE*IMG_SIZE); i++)
	{
	    fprintf(outf, "%lf\n", Z[i]);
        }

	fclose(outf);
	/* ----- */
    }   /* for i_cntr */

    free(b);

/* for cpu timing */
getrusage(0,&rusage);
ets = rusage.ru_utime.tv_sec;
etm = rusage.ru_utime.tv_usec;
tol_ts += (ets-bts);
tol_tm += (etm-btm);
printf(" %ld sec.  %ld usec. \n", tol_ts, tol_tm);
}



recursive(h, rhs, Z)
    int h;
    double *rhs, **Z;
{
    int size, k;
    double *fresid, *cresid;
    double *dZ;

    size = IMG_SIZE / (1 << (h-1));

    if (h == nGrid) {			/* the coarsest grid */
	exact(h, rhs, *Z, size);
	return;
    }

    relax(h, rhs, *Z, size);		/* presmoothing */

    allocate(&fresid, size*size);	/* allocate space and init to zeros */
    allocate(&cresid, size*size/4);	/* allocate space and init to zeros */
    residual(h, *Z, rhs, fresid);	/* compute the residual */
    restrict(cresid, fresid, size/2);
    free(fresid);
    allocate(&dZ, size*size/4);
    recursive(h+1, cresid, &dZ);
    free(cresid);
    interpolate(*Z, dZ, size);

    relax(h, rhs, *Z, size);		/* postsmoothing */

    free(dZ);				/* deallocate space */
}




exact(h, rhs, Z, size)
    int h, size;
    double *rhs, *Z;
{
    int i;
    double resid[16];

    /* Added for cpu timing
    printf("Enter exact. . .\n");
    */

    for (i = 0;  i < size*size;  i++) {
       resid[i] = 0.0;
       if (Z[i] != 0.0) {
	   printf("Z[%d] = %lf\n", i, Z[i]);
	}
    }

    GSI2(h-1, rhs, Z, size);

    /* Added for cpu timing
    for (i = 0;  i < size*size;  i++) {
       printf("resid[%d] = %lf\n", i, resid[i]);
    }
    */
    /*	compute the residual 
    residual(h, Z, rhs, resid);
    */
}




relax(h, rhs, Z, size)
    int h, size;
    double *rhs, *Z;
{
    GSI(h-1, rhs, Z, size);
}



allocate(ptr, size)
    double **ptr;
    int size;
{
    int i;
/* for timing */
long ts0, tm0, ts1, tm1;
struct rusage rusage;
long bts,btm,ets,etm;
 
 /* for cpu timing */
 getrusage(0,&rusage);
 ts0 = rusage.ru_utime.tv_sec;
 tm0 = rusage.ru_utime.tv_usec;


    if ( (*ptr = (double *)calloc(size, sizeof(double))) == NULL) {
        fprintf(stderr, "Out of memory in allocate.\n");
        exit(1);
    }

    for (i = 0; i < size; i++) {
	(*ptr)[i] = 0.0;
    }

/* for cpu timing */
getrusage(0,&rusage);
ts1 = rusage.ru_utime.tv_sec;
tm1 = rusage.ru_utime.tv_usec;

tol_ts -= (ts1 - ts0);
tol_tm -= (tm1 - tm0);
}
