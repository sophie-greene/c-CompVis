#include "DEF.h"

extern IMG_SIZE;
extern double C[MaxGrids][ARRAY_SQ][13];		/* stiffness matrix  */

/* 
** compute the residual
*/
residual(h, Z, rhs, resid)
    int h;
    double *Z, *rhs, *resid;
{
    int i, k;
    int idx[13];
    int size;

    size = IMG_SIZE / ( 1 << (h-1) );

    idx[0] = 0;
    idx[1] = 1;
    idx[2] = -size + 1;
    idx[3] = -size;
    idx[4] = -1;
    idx[5] = size - 1;
    idx[6] = size;

    idx[7]  = 2;
    idx[8]  = -2 * size;
    idx[9]  = -size - 1;
    idx[10] = -2;
    idx[11] = 2 * size;
    idx[12] = 1 + size;

    for (i = 0; i < (size*size); i++) {
	resid[i] = rhs[i];
	for (k = 0; k < 13; k++) {
	    if ( ((i+idx[k]) >= 0) && ((i+idx[k]) < (size*size)) ) {
		resid[i] -= (Z[i+idx[k]] * C[h-1][i][k]);
	    }
	}
    }   /* for i */

}
