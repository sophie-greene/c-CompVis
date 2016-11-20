#include <sys/time.h>
#include <sys/resource.h>
#include <stdio.h>
#include <math.h>
#include "../util/ImageTools.h"

#include "../util/UCFReadPic.c"

#define	SSize 256

struct rusage rusage;
long bts,btm,ets,etm;


main()
{
 char   filename[80];
 FILE   *outfile,*infile;
 int i,j,I,iter,Size;
 float Sx,Sy,Sz,Ps,Qs,p,q,pq,PQs,fZ,dfZ,Eij,Wn=0.0001*0.0001,Y,K;
 float Zn[SSize][SSize],Zn1[SSize][SSize],Si1[SSize][SSize],Si[SSize][SSize];
 PIC pic1;

 /* for synthetic images usually 1 or 2 iterations are enough */
 printf("Input number of iterations : ");
 scanf("%d",&iter);

 printf("Input size of image : ");
 scanf("%d",&Size);
 if(Size > 256)
 {
  printf("Image size too large!\n");
  exit(1);
 }

/* assume the initial estimate zero at time n-1 */
 for(i=0;i<Size;i++)
  for(j=0;j<Size;j++){
   Zn1[i][j] = 0.0;
   Si1[i][j] = 1.0; }

 printf("Input the image filename : ");
 scanf("%s",filename);
 if((infile = fopen(filename,"r")) == NULL)
 {
  fprintf(stderr,"Error Opening file : %s\n",filename);
  exit(1);
 }
 pic1 = UCFReadPic(infile);
 printf("\nInput the light source direction : \n");

 printf("\nSx = ");
 scanf("%f",&Sx);
 printf("\n");
 printf("Sy = ");
 scanf("%f",&Sy);
 printf("\n");
 printf("Sz = ");
 scanf("%f",&Sz);
 printf("\n");
 if(Sx == 0 && Sy == 0) Sx = Sy = 0.01;
 Ps = Sx/Sz;
 Qs = Sy/Sz;

/************************************************************************/
getrusage(0,&rusage);
bts = rusage.ru_utime.tv_sec;
btm = rusage.ru_utime.tv_usec;

 for(I=1;I<=iter;I++){
  for(i=0;i<Size;i++)
   for(j=0;j<Size;j++){ /* calculate -f(Zij) & df(Zij) */
    if(j-1 < 0 || i-1 < 0) /* take care boundary */
      p = q = 0.0;
    else {
          p = Zn1[i][j] - Zn1[i][(j-1)];
          q = Zn1[i][j] - Zn1[i-1][j]; }
    pq = 1.0 + p*p + q*q;
    PQs = 1.0 + Ps*Ps + Qs*Qs;
    Eij = pic1.image[i*pic1.maxX+j]/255.0;
    fZ = -1.0*(Eij - MAX(0.0,(1+p*Ps+q*Qs)/(sqrt(pq)*sqrt(PQs))));
    dfZ = -1.0*((Ps+Qs)/(sqrt(pq)*sqrt(PQs))-(p+q)*(1.0+p*Ps+q*Qs)/
                       (sqrt(pq*pq*pq)*sqrt(PQs))) ;
    Y = fZ + dfZ*Zn1[i][j];
    K = Si1[i][j]*dfZ/(Wn+dfZ*Si1[i][j]*dfZ);
    Si[i][j] = (1.0 - K*dfZ)*Si1[i][j]; 
    Zn[i][j] = Zn1[i][j] + K*(Y-dfZ*Zn1[i][j]);}

  for(i=0;i<Size;i++)
   for(j=0;j<Size;j++){
    Zn1[i][j] = Zn[i][j];
    Si1[i][j] = Si[i][j];}
 }
getrusage(0,&rusage);
ets = rusage.ru_utime.tv_sec;
etm = rusage.ru_utime.tv_usec;

printf(" %ld sec.  %ld usec. \n",ets-bts,etm-btm);

  printf("\nOutput depth map !\n");
  sprintf(filename,"final.out");
  outfile = fopen(filename,"w");
  for(i=0;i<Size;i++)
   for(j=0;j<Size;j++)
    fprintf(outfile,"%f\n",Zn[i][j]);
  fclose(outfile);
} /* end of main */


