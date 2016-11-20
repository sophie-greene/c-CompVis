/* Zheng and Chellappa's algorithm */

#include <sys/time.h>
#include <sys/resource.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include "../util/ImageTools.h"
#include "../util/UCFReadPic.c" 
#include "../util/UCFWritePic.c"

#define MSize 	256
#define MSize1	128
#define LS	32

PIC pic1,fpic;
FILE *imagefile;
char tchar,filename[101];
int lev;
float albedo,Ibias,Sx,Sy,Sz;
float Zk[MSize][MSize],Zk1[MSize][MSize];
float Pk[MSize][MSize],Pk1[MSize][MSize];
float Qk[MSize][MSize],Qk1[MSize][MSize];
float I[MSize][MSize];
int sizex,sizey,iter;

struct rusage rusage;
long bts,btm,ets,etm,Tts=0.0,Ttm=0.0;

reduce_all()
{
int i,x,y,xl,yl;

/* generate different size (level) UCF format image */

 sprintf(filename,"lev%d.img",lev);
 imagefile=fopen(filename,"w");
 UCFWritePic(pic1,imagefile);
 fclose(imagefile);

 for(i=lev;i>0;i--)
 {
  imagefile=fopen(filename,"r");
  pic1 = UCFReadPic(imagefile);
  fclose(imagefile);
  xl=pic1.maxX/2;
  yl=pic1.maxY/2;
  
  fpic.type=0;
  fpic.maxX=xl;
  fpic.maxY=yl;
  fpic.image = (BYTE *) calloc ((fpic.maxX*fpic.maxY), sizeof(BYTE)); 
  
  for (y=0; y<yl; y++)
   for (x=0; x<xl; x++)
    pixel(x,y,&fpic)= (pixel(x*2,y*2,&pic1)+pixel(x*2+1,y*2,&pic1)+pixel(x*2,y*2+1,&pic1)+pixel(x*2+1,y*2+1,&pic1))/4;
  		
  sprintf(filename,"lev%d.img",i-1);
  imagefile=fopen(filename,"w");
  UCFWritePic(fpic,imagefile);
  fclose(imagefile);
 
  free(fpic.image);
 }
}/* end reduce_all */

iteration()
{
float u = 1.0, dpq = 0.001;
int Nmax = 500;
int i,j,i1;
float px,pxx,pyy,qy,qxx,qyy,Zx,Zxx,Zy,Zyy,Ixx,Iyy;
float dp,dq,dZ,A11,A12,A22,R,Rp,Rq,C3,C2,C1,e,E,D;
float R1,R2,dP,dQ,dz,dP1,dQ1,dz1;

 dP = dQ = dz = 0.0;
 for(i1=1;i1<=Nmax;i1++)
 {
  dP1 = dP;
  dQ1 = dQ;
  dz1 = dz;
  dP = dQ = dz = 0.0;
  for(i=0;i<sizey;i++)
   for(j=0;j<sizex;j++)
   {
    if(j+1 < sizex)
     px = Pk[i][j+1] - Pk[i][j];
    else
     px = (Pk[i][j-1] - Pk[i][j]);

    if(j-1 >= 0 && j+1 < sizex)
     pxx = Pk[i][j+1] + Pk[i][j-1] - 2.0*Pk[i][j];
    else
      pxx = 0.0;
 
    if(i-1 >= 0 && i+1 < sizey)
     pyy = Pk[i+1][j] + Pk[i-1][j] - 2.0*Pk[i][j];
    else
      pyy = 0.0;
 
    if(i+1 < sizey)
     qy = Qk[i+1][j] - Qk[i][j];
    else
     qy = (Qk[i-1][j] - Qk[i][j]);
       
    if(j-1 >= 0 && j+1 < sizex)
     qxx = Qk[i][j+1] + Qk[i][j-1] - 2.0*Qk[i][j];
    else
      qxx = 0.0;
       
    if(i-1 >= 0 && i+1 < sizey)
     qyy = Qk[i+1][j] + Qk[i-1][j] - 2.0*Qk[i][j];
    else
      qyy = 0.0;
       
    if(j+1 < sizex)
     Zx = Zk[i][j+1] - Zk[i][j];
    else
     Zx = (Zk[i][j-1] - Zk[i][j]);
       
    if(j-1 >= 0 && j+1 < sizex)
     Zxx = Zk[i][j+1] + Zk[i][j-1] - 2.0*Zk[i][j];
    else
      Zxx = 0.0;
       
    if(i+1 < sizey)
     Zy = Zk[i+1][j] - Zk[i][j];
    else
     Zy = (Zk[i-1][j] - Zk[i][j]);
       
    if(i-1 >= 0 && i+1 < sizey)
     Zyy = Zk[i+1][j] + Zk[i-1][j] - 2.0*Zk[i][j];
    else
      Zyy = 0.0;
       
    if(j-1 >= 0 && j+1 < sizex)
     Ixx = I[i][j+1] + I[i][j-1] - 2.0*I[i][j];
    else
      Ixx = 0.0;
       

    if(i-1 >= 0 && i+1 < sizey)
     Iyy = I[i+1][j] + I[i-1][j] - 2.0*I[i][j];
    else
      Iyy = 0.0;
       
/*****************************************************************************/
 
    R = (Ibias + (Sz-Pk[i][j]*Sx-Qk[i][j]*Sy)/
        sqrt(Pk[i][j]*Pk[i][j]+Qk[i][j]*Qk[i][j]+1));
 
    R1 = (Ibias + (Sz-(Pk[i][j]+dpq)*Sx-Qk[i][j]*Sy)/
          sqrt((Pk[i][j]+dpq)*(Pk[i][j]-dpq)+Qk[i][j]*Qk[i][j]+1));
 
    Rp = R1 - R;
 
    R2 = (Ibias + (Sz-Pk[i][j]*Sx-(Qk[i][j]+dpq)*Sy)/
        sqrt(Pk[i][j]*Pk[i][j]+(Qk[i][j]+dpq)*(Qk[i][j]+dpq)+1));
 
    Rq = R2 - R;
 
    A11 = 5.0*Rp*Rp + 1.25*u;
    A12 = 5.0*Rp*Rq + 0.25*u;
    A22 = 5.0*Rq*Rq + 1.25*u;
     e = R - I[i][j];
    E = Rp*(pxx+pyy) + Rq*(qxx+qyy) - Ixx -Iyy;

    C3 = -px - qy + Zxx +Zyy;
    C1 = (E - e)*Rp - u*(Pk[i][j] - Zx) - 0.25*u*C3;
    C2 = (E - e)*Rq - u*(Qk[i][j] - Zy) - 0.25*u*C3;

    D = A11*A22 - A12*A12;

    dp = (C1*A22 - C2*A12)/D;
    dq = (C2*A11 - C1*A12)/D;
    dZ = (C3 + dp + dq)/4.0;

    Pk1[i][j] = Pk[i][j] + dp;
    Qk1[i][j] = Qk[i][j] + dq;
    Zk1[i][j] = Zk[i][j] + dZ;

    dP += dp;
    dQ += dq;
    dz += dZ;

   }/* end of i,j loop */
  if(fabs(dP-dP1) < 0.000001 && fabs(dQ-dQ1) < 0.000001 && fabs(dz-dz1) < 0.000001) break;

 for(i=0;i<sizey;i++)
  for(j=0;j<sizex;j++)
  {
   Pk[i][j] = Pk1[i][j];
   Qk[i][j] = Qk1[i][j];
   Zk[i][j] = Zk1[i][j];
  }

 } /* end of i1 */

} /* end of iteration */


exPQZ()
{
int xlen,ylen,i,j;

xlen = sizex * 2;
ylen = sizey * 2;

/* Rule 2 */
 
for(i=1;i<=ylen-1;i++)  /* one base (size from 1 to M) */
 for(j=2;j<=xlen;j++)
 {
  if(i%2 == 0 && j%2 == 0)
  {
   Pk[i-1][j-1] = Pk1[i/2-1][j/2-1];  /* -1 for zero base */
   Qk[i-1][j-1] = Qk1[i/2-1][j/2-1];
   Zk[i-1][j-1] = 2.0*Zk1[i/2-1][j/2-1];
  }
  if(i%2 != 0 && j%2 == 0)
  {
   Pk[i-1][j-1] = 0.5*(Pk1[(i+1)/2-1][j/2-1]+Pk1[(i-1)/2-1][j/2-1]);
   Qk[i-1][j-1] = 0.5*(Qk1[(i+1)/2-1][j/2-1]+Qk1[(i-1)/2-1][j/2-1]);
   Zk[i-1][j-1] = 0.5*(2.0*Zk1[(i+1)/2-1][j/2-1]+2.0*Zk1[(i-1)/2-1][j/2-1]);
  }
  if(i%2 == 0 && j%2 != 0)
  {
   Pk[i-1][j-1] = 0.5*(Pk1[i/2-1][(j+1)/2-1]+Pk1[i/2-1][(j-1)/2-1]);
   Qk[i-1][j-1] = 0.5*(Qk1[i/2-1][(j+1)/2-1]+Qk1[i/2-1][(j-1)/2-1]);
   Zk[i-1][j-1] = 0.5*(2.0*Zk1[i/2-1][(j+1)/2-1]+2.0*Zk1[i/2-1][(j-1)/2-1]);
  }
  if(i%2 != 0 && j%2 != 0)
  {
   Pk[i-1][j-1] = 0.25*(Pk1[(i+1)/2-1][(j+1)/2-1] +
                        Pk1[(i+1)/2-1][(j-1)/2-1] +
                        Pk1[(i-1)/2-1][(j+1)/2-1] +
                        Pk1[(i-1)/2-1][(j-1)/2-1]);
   Qk[i-1][j-1] = 0.25*(Qk1[(i+1)/2-1][(j+1)/2-1] +
                        Qk1[(i+1)/2-1][(j-1)/2-1] +
                        Qk1[(i-1)/2-1][(j+1)/2-1] +
                        Qk1[(i-1)/2-1][(j-1)/2-1]);
   Zk[i-1][j-1] = 0.25*(2.0*Zk1[(i+1)/2-1][(j+1)/2-1] +
                        2.0*Zk1[(i+1)/2-1][(j-1)/2-1] +
                        2.0*Zk1[(i-1)/2-1][(j+1)/2-1] +
                        2.0*Zk1[(i-1)/2-1][(j-1)/2-1]);
  }
 }/* end of i,j loop */
 
/* Rule 3 */
 
for(j=2;j<=xlen;j++)
 {
  Pk[ylen-1][j-1] = 2.0*Pk[(ylen-1)-1][j-1] - Pk[(ylen-2)-1][j-1];
  Qk[ylen-1][j-1] = 2.0*Qk[(ylen-1)-1][j-1] - Qk[(ylen-2)-1][j-1];
  Zk[ylen-1][j-1] = Zk[(ylen-1)-1][j] - Pk[(ylen-1)-1][j-1];
 }
for(i=1;i<=ylen-1;i++)
 {
  Pk[i-1][1-1] = 2.0*Pk[i-1][2-1] - Pk[i-1][3-1];
  Qk[i-1][1-1] = 2.0*Qk[i-1][2-1] - Qk[i-1][3-1];
  Zk[i-1][1-1] = Zk[i-1][2-1] - Qk[i-1][1-1];
 }
Pk[ylen-1][1-1] = 2.0*Pk[(ylen-1)-1][2-1] - Pk[(ylen-2)-1][3-1];
Qk[ylen-1][1-1] = 2.0*Qk[(ylen-1)-1][2-1] - Qk[(ylen-2)-1][3-1];
Zk[ylen-1][1-1] = Zk[(ylen-1)-1][2-1] - Pk[ylen-1][1-1] + Qk[ylen-1][1-1];

} /* end of exPQZ */

main() 
{
int i,j,loop;
char filename[80];
FILE *in,*out;
PIC lpic;

/* input image */
 puts("Enter the image file name =>");
 scanf("%s",filename);
 if ((imagefile=fopen(filename,"r"))==NULL) 
	{
	printf("File %s is not found. \n",filename);
	exit(1);
	};

 pic1=UCFReadPic(imagefile);
 fclose(imagefile); 

 if((pic1.maxX != MSize || pic1.maxY != MSize) && (pic1.maxX != MSize1 || pic1.maxY != MSize1)){
   printf("Input image size error !!\n");
   exit(1);
  }
 if(pic1.maxX == MSize)
  lev = 3;
 else lev = 2;

 reduce_all();

 printf("\nAlbedo = ");
 scanf("%f",&albedo);
 
 printf("\nIntensity bias = ");
 scanf("%f",&Ibias);
 
 printf("Input the light source :\n");
 printf("Sx = ");
 scanf("%f",&Sx);
 printf("\nSy = ");
 scanf("%f",&Sy);
 printf("\nSz = ");
 scanf("%f",&Sz);

 if(Sx == 0.0 && Sy == 0.0) Sx = Sy = 0.01;

 Sz = -1.0*Sz; /* switch the sign, viewer from -z */

 Sx = Sx / sqrt(Sx*Sx+Sy*Sy+Sz*Sz);
 Sy = Sy / sqrt(Sx*Sx+Sy*Sy+Sz*Sz);
 Sz = Sz / sqrt(Sx*Sx+Sy*Sy+Sz*Sz);

/* initial the lowest layer */
 for(i=0;i<LS;i++)
  for(j=0;j<LS;j++)
   Pk[i][j] = Qk[i][j] = Zk[i][j] = 0.0;

/* main loop */
 for(loop=0;loop<=lev;loop++)
 {
  /* input the image for current layer */
  sprintf(filename,"lev%d.img",loop);
  in = fopen(filename,"r");
  lpic = UCFReadPic(in);

getrusage(0,&rusage);
bts = rusage.ru_utime.tv_sec;
btm = rusage.ru_utime.tv_usec;
 
  for(i=0;i<lpic.maxY;i++)
   for(j=0;j<lpic.maxX;j++){
    I[i][j] = (pixel(j,i,&lpic) - Ibias) / albedo;
    if(I[i][j] < 0.0)
    {
     printf("Error for Ibias -- negative value.\n");
     exit(1);
    }
   }
  sizex = lpic.maxX;
  sizey = lpic.maxY;
 
/* iteration for current layer, output in Pk1[][], Qk1[][], Zk1[][] */     
  iteration();

  if(loop<lev)
   exPQZ(); /* expand Pk1, Qk1, Zk1 to Pk, Qk, Zk */

getrusage(0,&rusage);
ets = rusage.ru_utime.tv_sec;
etm = rusage.ru_utime.tv_usec;
 
printf("Level %d : %ld sec.  %ld usec. \n",loop,ets-bts,etm-btm);

Tts += ets-bts;
Ttm += etm-btm;

 } /* end of main loop */

printf("\nTotal  %ld sec. %ld msec \n",Tts,Ttm);

printf("Enter the output data file name for Z =>");
scanf("%s",filename);
if ((out=fopen(filename,"w"))==NULL)
        {
        printf("File %s is not found. \n",filename);
        exit(1);
        };
for(i=0;i<sizey;i++)
 for(j=0;j<sizex;j++)
  fprintf(out,"%f\n",Zk1[i][j]);
fclose(out);
 

} /* end of main */


		
