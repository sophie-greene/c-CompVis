#include <math.h>
#include <stdio.h>
#include "DEF.h"

extern double C[MaxGrids][ARRAY_SQ][13];	/* stiffness matrix  */

/*
** Solve AZ = b, where A is sparse matrix
*/
GSI(index, b, Z, n)
    double *b, *Z;
    int n, index;			/* size of the current image */
{
    int i, j, k, nIter;
    int MaxIter = 500;
    int idx[13];
    int Mn;
    double old_tol, tol, tmp;
    double *Z0;

    Mn = n * n;
    allocate(&Z0, Mn);

    idx[0] = 0;
    idx[1] = 1;
    idx[2] = -n + 1;
    idx[3] = -n;
    idx[4] = -1;
    idx[5] = n - 1;
    idx[6] = n;

    idx[7]  = 2;
    idx[8]  = -2 * n;
    idx[9]  = -n - 1;
    idx[10] = -2;
    idx[11] = 2 * n;
    idx[12] = 1 + n;

    old_tol = 9e9;

    for (nIter = 0; nIter < MaxIter; nIter++) {
	for (i = 0; i < Mn; i++) {
	    Z0[i] = Z[i];
	}
	
	for (i = 0; i < Mn; i++) {
	    if (fabs(C[index][i][0]) < EPSILON) {
		fprintf(stderr, "Diagnal element is zero!\n");
		exit(1);
	    }

	    tmp = 0.0;
	    for (k = 1; k < 13; k++) {
		if ( ((i+idx[k]) >= 0) && ((i+idx[k])<Mn) ) {
		    /*
		    ** Gauss-Seidel
		    */
		    tmp += Z[i+idx[k]] * C[index][i][k];
		}
	    }
	    Z[i] = (b[i] - tmp) / C[index][i][0];
	}

	tol = 0.0;
	for (i = 0; i < Mn; i++) {
	    tol += fabs(Z0[i] - Z[i]);
	}

/*
** let's not print EVERY ONE!
*/
	if (tol < THRESH) {
	    return;
	}

	if (fabs(old_tol - tol) < 0.000001) {
	    return;
	}

	if (old_tol < tol) {
	    return;
	}

	old_tol = tol;
    }
}
