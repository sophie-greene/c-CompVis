/*
** size*size is the size of Z
** (size/2)*(size/2) is the size of dZ
*/
interpolate(Z, dZ, size)
    double *Z, *dZ;
    int size;
{
    double *fdZ;
    int i;

    allocate(&fdZ, size*size);

    interp(fdZ, dZ, size);

    for (i = 0; i < (size*size); i++) {
	Z[i] += fdZ[i];
    }

    free(fdZ);
}



interp(uf,uc,nf)
    double *uc,*uf;
    int nf;
{
    int ic,iif,jc,jf,nc;

    nc=nf/2;

    /* Do elements that are copies */
    for (jc=0,jf=0;jc<nc;jc++,jf+=2) {
	for (ic=0;ic<nc;ic++)  {
	    uf[(2*ic)*nf+jf]=uc[ic*nc+jc];
        }
    }

    /* Do odd-numbered columns, interpolating vertically */
    for (jf=0;jf<nf;jf+=2) {
	for (iif=1;iif<(nf-1);iif+=2) {
	    uf[iif*nf+jf]=0.5*(uf[(iif+1)*nf+jf]+uf[(iif-1)*nf+jf]);
        }
    }

    /* Do even-numbered columns, interpolating horizontally */
    for (jf=1;jf<(nf-1);jf+=2) {
	for (iif=0;iif < nf;iif++) {
	    uf[iif*nf+jf]=0.5*(uf[iif*nf+jf+1]+uf[iif*nf+jf-1]);
        }
    }

    /* Do last column and last row */
    for (jf = 0;  jf < nf;  jf++) {
	uf[(nf-1)*nf+jf] = 2.0 * uf[(nf-2)*nf+jf] - uf[(nf-3)*nf+jf];
	uf[jf*nf+(nf-1)] = 2.0 * uf[jf*nf+(nf-2)] - uf[jf*nf+(nf-3)];
    }
}
