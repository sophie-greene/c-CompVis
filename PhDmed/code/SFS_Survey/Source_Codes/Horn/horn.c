#include <stdio.h>
#include <strings.h>
#include <math.h>
#include "../util/ImageTools.h"
#include "../util/UCFReadPic.c"
#include "../util/UCFWritePic.c"

#define ISIZE 	128 


int row[3]; 
float  A[3][6];

main (argc,argv)

int argc;
char *argv[];
  
{
FILE *infile,*outfile;
PIC pic1,pic2,edge;
char File1[80];
float Mx[ISIZE][ISIZE],My[ISIZE][ISIZE],Mz[ISIZE][ISIZE],
      Nx[ISIZE][ISIZE],Ny[ISIZE][ISIZE],Nz[ISIZE][ISIZE],
      TZ[ISIZE][ISIZE];
float d,lam,Sx,Sy,Sz,ss,Sxx,Syy,Szz,ns,mm,
      deg,ENx,ENy,ENz,nx,ny,nz,ee,max,min,dd,at1;
int I,J,K,i,j,k,E,X,flag=0,maxI;
float R=52.0,err;

E = 1;
lam = 0.25;
X = 1;

         if (argc != 2) {
		 fprintf(stderr,"Usage Error. Usage --> horn <Image>\n");
		 exit(1);
	 }
	 File1[0] = '\0';
	 strcpy(File1,argv[1]);
 	 if ((infile = fopen(File1,"r")) == NULL)
         {
		 fprintf(stderr,"Error Opening file : %s\n",File1);
		 exit(1);
         }

        printf(" Input Image file ==> %s\n",File1);  
	pic1  = UCFReadPic(infile);
        printf(" \n");
	fclose(infile);

printf("Input lambda (0.25) = ");
scanf("%f",&lam);
printf("\n");

printf("Input E (1.0) = ");
scanf("%f",&E);
printf("\n");

printf("Max Iteration = ");
scanf("%d",&maxI);
printf("\n");

printf("Input Sx = ");
scanf("%f",&Sx);
printf("\n");

printf("Input Sy = ");
scanf("%f",&Sy);
printf("\n");

printf("Input Sz = ");
scanf("%f",&Sz);
printf("\n");

dd = sqrt(Sx*Sx+Sy*Sy+Sz*Sz);
Sx = Sx/dd;
Sy = Sy/dd;
Sz = Sz/dd;

Sxx = Sx;
Syy = Sy;
Szz = Sz;

printf("Edge map = ");
scanf("%s",File1);
printf("\n");
if ((infile = fopen(File1,"r")) == NULL)
      {
	 fprintf(stderr,"Error Opening file : %s\n",File1);
	 exit(1);
      }

edge  = UCFReadPic(infile);
printf(" \n");
fclose(infile);

/* use the true depth to initial normal of edge points */
printf("True depth data file = ");
scanf("%s",File1);
printf("\n");
if ((infile = fopen(File1,"r")) == NULL)
      {
	 fprintf(stderr,"Error Opening file : %s\n",File1);
	 exit(1);
      }
for(j=0;j<ISIZE;j++)
 for(i=0;i<ISIZE;i++)
   fscanf(infile,"%f",&TZ[i][j]);

pic2.type = 0x0000;
pic2.maxX = ISIZE;
pic2.maxY = ISIZE;
pic2.image=(unsigned char *)calloc((pic2.maxX*pic2.maxY),sizeof(unsigned char));

/*   Initial N   */

        for (i=0;i<ISIZE;i++)
         for (j=0;j<ISIZE;j++)
         {
          if(edge.image[i+j*edge.maxX] == 255)
          {
           d = (i-ISIZE/2)*(i-ISIZE/2) + (ISIZE/2-j)*(ISIZE/2-j) + TZ[i][j]*TZ[i][j];
           Nx[i][j] = ((float)(i-64.0)/sqrt(d));  
           Ny[i][j] = ((float)(64.0-j)/sqrt(d));  
           Nz[i][j] = TZ[i][j]/sqrt(d);  
          }
          else
          {
           if(pic1.image[i+j*pic1.maxX] >0)
           {
            Nx[i][j] = 0.0;  
            Ny[i][j] = 0.0;  
            Nz[i][j] = 1.0;  
           }
           else
           {
            Nx[i][j] = 0.0;  
            Ny[i][j] = 0.0;  
            Nz[i][j] = 0.0;  
           }
          };
         };


/*   Interative loop   */
if(Sx<=0.0001)
at1=atan(Sy/0.000001); 
else
at1=atan(Sy/Sx);
printf("Initial S = (%5.3f,%5.3f,%5.3f)  A = %5.1f   B = %5.1f \n",Sx,Sy,Sz,57.3*acos(Sz/(sqrt(Sx*Sx+Sy*Sy+Sz*Sz))),57.3*at1);

K=1;
for(;;)
{

 for(I=X;I<pic1.maxX-X;I++)
  for(J=X;J<pic1.maxY-X;J++)
  {
   ns = (Nx[I][J]*Sx + Ny[I][J]*Sy + Nz[I][J]*Sz);

   Mx[I][J] = (Nx[I][J+1]+Nx[I][J-1]+Nx[I+1][J]+Nx[I-1][J])/4.0 +
          ((E*E)/(4.0*lam))*((pic1.image[I+J*pic1.maxY]/255.0 - MAX(0,ns))*Sx); 
   My[I][J] = (Ny[I][J+1]+Ny[I][J-1]+Ny[I+1][J]+Ny[I-1][J])/4.0 +
          ((E*E)/(4.0*lam))*((pic1.image[I+J*pic1.maxY]/255.0 - MAX(0,ns))*Sy);
   Mz[I][J] = (Nz[I][J+1]+Nz[I][J-1]+Nz[I+1][J]+Nz[I-1][J])/4.0 +
          ((E*E)/(4.0*lam))*((pic1.image[I+J*pic1.maxY]/255.0 - MAX(0,ns))*Sz);
  };

 for(I=X;I<pic1.maxX-X;I++)
  for(J=X;J<pic1.maxY-X;J++)
  {
  if(pic1.image[I+J*pic1.maxX] > 0 && edge.image[I+J*edge.maxX] != 255)
  {
   mm=sqrt(Mx[I][J]*Mx[I][J] + My[I][J]*My[I][J] + Mz[I][J]*Mz[I][J]);
   if (mm == 0)
   {
    Nx[I][J] = 0;
    Ny[I][J] = 0;
    Nz[I][J] = 0;
   }
   else
   {
    Nx[I][J] = Mx[I][J]/mm;
    Ny[I][J] = My[I][J]/mm;
    Nz[I][J] = Mz[I][J]/mm;
   }
  } 
  };

err = 0.0;
for(i=0;i<pic2.maxX;i++)
 for(j=0;j<pic2.maxY;j++)
 {
  ee = (Sxx*Nx[i][j] + Syy*Ny[i][j] + Szz*Nz[i][j]);
  pic2.image[i+j*pic2.maxY] = (int)(MAX(0,ee)*255);
  err += fabs(pic1.image[i+j*pic1.maxY]/255.0 - pic2.image[i+j*pic2.maxY]/255.0);
 };
err= err/(ISIZE*ISIZE);

printf("K = %d\n",K++);

printf("Intensity Error = %f\n",err);

if(K==maxI || err < 0.04)break;

}

 outfile = fopen("out2.img","w");
 UCFWritePic(pic2,outfile);
 fclose(outfile);

 outfile = fopen("normal2.out","w");
 for(i=0;i<pic2.maxX;i++)
  for(j=0;j<pic2.maxY;j++)
   fprintf(outfile,"%f %f %f \n",Nx[i][j],Ny[i][j],Nz[i][j]);
 fclose(outfile);

}

