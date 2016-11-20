restrict(uc,uf,nc)
    double *uc,*uf;
    int nc;
{
    int ic,iif,jc,jf,ncc=2*nc-1,nf=2*nc;

    for (jf=2,jc=1;jc<(nc/*-1*/);jc++,jf+=2) {
        for (iif=2,ic=1;ic<(nc/*-1*/);ic++,iif+=2) {
            uc[ic*nc+jc]=0.250*uf[iif*nf+jf]
			+0.125*( uf[(iif+1)*nf+jf] + uf[(iif-1)*nf+jf]
				+uf[iif*nf+jf+1] + uf[iif*nf+jf-1] )
			+0.0625*( uf[(iif+1)*nf+(jf-1)]+uf[(iif-1)*nf+(jf-1)]
				 +uf[(iif+1)*nf+(jf+1)]+uf[(iif-1)*nf+(jf+1)]);
        }
    }
    for (jc=0,ic=0;ic<nc;ic++,jc+=2) {
        uc[ic*nc+0]=uf[jc*nf+0];		/* top row */
    }
    for (jc=0,ic=0;ic<nc;ic++,jc+=2) {
        uc[0*nc+ic]=uf[0*nf+jc];		/* left edge */
    }
}
