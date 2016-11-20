/* Lee and Rosenfeld's algorithm */

#include <sys/time.h>
#include <sys/resource.h>
#include<stdio.h>
#include<math.h>
#include "../util/ImageTools.h"
#include "../util/UCFReadPic.c"

float MaX,MiN,BIo,pi=3.141592654;
PIC image;
float Albedo,*I,*Z;
char fname[80];
int size;                   /* size of square image (row) */
         
struct Norm {
             float x,y,z;
	    };
struct Norm *normal;

ReadImage()
 {

  char name [80];
  FILE *image_fp;

  printf("Enter the name of the input image: ");
  scanf("%s",name);
  while((image_fp=fopen(name,"r"))==NULL)
  {
    printf("Can't find the file %s.\n",name);
    printf("Enter the name of the input image: ");
    scanf("%s",name);
  }
  image=UCFReadPic(image_fp);
  fclose(image_fp);
  
  printf("Enter the output filename: ");
  scanf("%s",fname);
} 

/* normalizing the Image */
Normalize_Image()
{
int x,y;

for (x = 0; x < size*size; x++)
  {
    I[x]=(float) (image.image[x]);;
    Z[x]=0;
  }
}


Normal(Fx,Fy,i,j,Ip,slantL,tiltL) 
float Fx,Fy;
float slantL,tiltL;
float Ip;
int i,j;
{
float xs,ys,zs,xe,ye,x1,y1,z1,D;
float Ang,RS,tilt,slant,V;
int fl;

  RS = Ip/BIo;
  slant = acos(RS);

  xe = Fx;
  ye = Fy;


  x1 = xe*cos(slantL)*cos(tiltL) + ye*cos(slantL)*sin(tiltL);          
  y1 = xe*(-sin(tiltL)) + ye*cos(tiltL);
  z1 = z1;
  z1 = sqrt(x1*x1 + y1*y1);
  V = atan2((y1),(.001+x1));
  Ang = V ;
  if (Ang < 0) Ang = 2.0*pi+Ang;

  tilt = Ang;

  x1 = sin(slant)*cos(tilt);
  y1 = sin(slant)*sin(tilt);
  z1 = cos(slant);

      normal[i+j*size].x = x1;
      normal[i+j*size].y = y1;
      normal[i+j*size].z = z1;
}




Lee_Ros(I,xo,yo,zo)
float *I,xo,yo,zo;
{
int i,j;
float tilt,slant;
float Fx,Fy,FL,Ip;
double D,det,x1,y1,z1,x2,y2,z2,matrix[9],matrix2[9];
int x,y,msize;


for (i = 0; i < size; i++)
{
  normal[i].x =0.0;
  normal[i].y =0.0;
  normal[i].z =0.0;
}

tilt = atan2(yo,xo+.0001);
tilt = tilt * 180.0/pi;
if (tilt < 0) tilt = tilt + 360;
tilt = tilt * pi/180.0;
slant = acos(zo);


matrix[0] = cos(slant)*cos(tilt);
matrix[1] = cos(slant)*sin(tilt);
matrix[2] = -sin(slant);
matrix[3] = -sin(tilt);
matrix[4] = cos(tilt);
matrix[5] = 0;
matrix[6] = sin(slant)*cos(tilt);
matrix[7] = sin(slant)*sin(tilt);
matrix[8] = cos(slant);

for (x=0; x<9; x++)
    matrix2[x] = matrix[x];


msize=3;
inverse_matrix(msize,&det,matrix);


BIo = 0;
for ( y = 0; y <size-0; y++)
for (x = 0; x < size-0; x++)
      if (I[x+y*size] > BIo)
      BIo = I[x+y*size];

for (y = 0; y < size; y++)
for (x = 0; x < size; x++)
{

  Fx= -(I[x+1+y*size]-I[x+y*size]);
  Fy= (I[x+(y+1)*size]-I[x+y*size]);

  Ip = I[x+y*size];

  Normal(Fx,Fy,x,y,Ip,slant,tilt);

     x1 = normal[x+y*size].x;
     y1 = normal[x+y*size].y;
     z1 = normal[x+y*size].z;

     x2 = x1*matrix[0] + y1*matrix[1] + z1*matrix[2];
     y2 = x1*matrix[3] + y1*matrix[4] + z1*matrix[5];
     z2 = x1*matrix[6] + y1*matrix[7] + z1*matrix[8];

     D = sqrt(x2*x2 + y2*y2 + z2*z2) + .000001;
               
     x2 = x2/D;
     y2 = y2/D;
     z2 = z2/D;

     normal[x+y*size].x = x2;
     normal[x+y*size].y = y2;
     normal[x+y*size].z = z2;

     Z[x+y*size] = z2 * 255.0;

 }
 }

int inverse_matrix( size, det, matrix )

                                /*---------------------------------------*/
double	matrix[],               /* Matrix to invert.                     */
        *det;                   /* Will return determinate of matrix     */
int     size;                   /* Size of matrix:   size x size         */
                                /*                                       */
{                               /*                                       */
double 	aext,                   /* Extreme (largest) value in sub-matrix */
        atemp,                  /* Holds temp. value of the matrix       */
        de = 1.0;               /* Initial determate value.              */
int     ir[ 64],           /* Use to remember swaps pos. in rows    */
        ic[ 64 ],          /* Use to remember swaps pos. in columns */
        i,j,k,kk,               /* Ioop control variables                */
        itemp,idim,             /*                                       */
        iext,jext,              /* I extreme, J extreme for swapping pos */
        flag = 0;               /* Boolean test for finding correct rows */
                                /*      and columns in step 5.           */
                                /*---------------------------------------*/

     /* ITIALIZE SWAP ARRAYS TO CORRECT POSITIONS */

    for( j=0; j < size; j++ ) {
		ic[ j ] = j;
		ir[ j ] = j;
	}
/* STEP 1 */

  /* FIND INDEXES  (jext and iext) OF LARGEST ELEMENT IN REMAINING MATRIX */

	for( k=0; k < size; k++ ) {
		aext = 0.0;
		for( i=k; i < size; i++ ) {
	    	for( j=k; j < size; j++ ) {
		  	if( aext < fabs( matrix[i*size+j] ) ) {
		   		iext = i;
		   	    jext = j;
		   	   	aext = fabs( matrix[ i * size + j] );
	       	}
	    }
	}

    if( aext <= 0.0 ){
		printf("\nInverse_matrix; singular\n");
		*det = 0;
		return(-100);
		/*exit( -20 );
		*/
    }

	/* STEP 2 */

    if( k != iext ) {
	     /*  MOVE COLUMN WITH BIGGEST ELEMENT TO ROW K. */

		de = - de;
		for( j=0; j < size; j++ ) {
		    atemp		      = matrix[ k * size + j ];
		    matrix[ k * size + j ]    = matrix[ iext * size + j ];
		    matrix[ iext * size + j ] = atemp;
		}
		itemp    = ic[k];
		ic[k]    = ic[iext];
		ic[iext] = itemp;
	}

    if( k != jext ) {
	     /*  MOVE ROW WITH BIGGEST ELEMENT TO COLUMN K. */

		de = - de;
		for( i=0; i < size; i++ ) {
		    atemp	      	      = matrix[ i * size + k ];
		    matrix[ i * size + k]     = matrix[ i * size + jext ];
		    matrix[ i * size + jext ] = atemp;
		}
		itemp    = ir[k];
		ir[k]    = ir[jext];
		ir[jext] = itemp;
	}


	/* STEP 3 */
	     /* PERFORM GAUSS-JORDAN ELIMINATION WITH CURRENT ROW K. */

	aext = matrix[ k * size + k];
	de = de * aext;
	matrix[ k * size + k ] = 1.0;
	for( j=0; j < size; j++ )
		matrix[ k * size + j ] = matrix[ k * size + j ] / aext;
		for( i=0; i < size; i++ ) {
		    if( k != i ) {
				aext = matrix[i*size+k];
				if( aext != 0.0 ) {
				    matrix[ i * size + k ] = 0.0;
				    for( j=0; j < size; j++ )
                         matrix[i*size+j] = matrix[i*size+j] -
                                                     aext * matrix[k*size+j];
				}
		    }
		}
	}


	/* STEP 5 */
	      /* INVERSE ELEMENTS HAVE BEEN CALCULATED RETURN
		 MATRIX TO ORIGINAL ROW, COLUMN ORDER;         */

	idim = size - 1;
	for( k=0; k < idim; k++ ) {
		kk = k + 1;
		if( k != ic[k] ) {

	     /*  LOOP UNTIL FIND CORRECT POSITON IN SWAP ARRAY.
		 IF CORRECT POSITION IS NOT FOUND; ERROR OCCURED. */

			i = kk;
	    	flag = 0;
			while((i < size) && (!flag)) {
				if( k == ic[i] ) flag = 1;
					else i++;
			}

			if( flag ) {
				for( j=0; j < size; j++ ) {
		    		atemp                  = matrix[ j * size + k ];
					matrix[ j * size + k ] = matrix[ j * size + i];
					matrix[ j * size + i ] = atemp;
				}
				itemp = ic[i];
				ic[i] = ic[k];
				ic[k] = itemp;
			}
	   		else {
				printf("\nInverse_matrix: 1 suspected machine error.\n");
				printf(" k = %d  ic[ %d ] = %d \n",k,i,ic[i]);
				/*exit( -15 ) ;
				*/
	   		}
		}
		if( k != ir[k] ) {

	     /*  LOOP UNTIL FIND CORRECT POSITON IN SWAP ARRAY.
		 IF CORRECT POSITION IS NOT FOUND; ERROR OCCURED. */

	   		j = kk;
	    	flag = 0;
			while((j < size) && (!flag)) {
				if( k == ir[j] ) flag = 1;
				else j++;
	    	}

	    	if( flag ) {
				for( i=0; i < size; i++ ) {
				    atemp 	           = matrix[ k * size + i ];
				    matrix[ k * size + i ] = matrix[ j * size + i ];
				    matrix[ j * size + i ] = atemp;
				}
				itemp = ir[j];
				ir[j] = ir[k];
				ir[k] = itemp;
	    	}
	    	else {
				printf(" \nInverse_matrix: 2 suspected machine error.\n");
				printf(" k = %d  ic[ %d ] = %d \n",k,i,ic[i]);
				/*exit( -15 );
				*/
		    }
		}
	}
    *det = de;
    return( 1 );
}




/**********************************************************/
/*  This is the main routine where the program is started */
/*                                                        */
/*                                                        */
/**********************************************************/


main()
{
float x,y,z,D;
struct rusage rusage;
long bts,btm,ets,etm;

  ReadImage();
  size = image.maxX;   /* Assume square image */

  normal = (struct Norm *)calloc(size*size,sizeof(struct Norm));
  if( normal == NULL) {
          printf(" SCRIPT TABLE ALLOCATION FAILURE! \n");
	  exit(0);}


  Z = (float *)calloc(size*size,sizeof(float));
  if( Z == NULL) {
          printf(" SCRIPT TABLE ALLOCATION FAILURE! \n");
	  exit(0);}


  I = (float *)calloc(size*size,sizeof(float));
  if( I == NULL) {
          printf(" SCRIPT TABLE ALLOCATION FAILURE! \n");
	  exit(0);}


/* step 1 estimate the reflectance parameters tilt, slant and albedo=n */

   Albedo=255;

  printf("\nEnter light source "); 
  scanf("%f %f %f",&x,&y,&z); 
  D = sqrt(x*x + y*y + z*z);
  x = x/D ;
  y = y/D;
  z = z/D;


/*  step 2 convert the image to float and set Z to zero*/
  Normalize_Image(Albedo);

  printf("\nEnter min and max depth for rescale:\n");
  printf("\nMinimun : "); 
  scanf("%f",&MiN); 
  printf("\nMaximun : "); 
  scanf("%f",&MaX); 

getrusage(0,&rusage);
bts = rusage.ru_utime.tv_sec;
btm = rusage.ru_utime.tv_usec;

  Lee_Ros(I,x,y,z);

getrusage(0,&rusage);
ets = rusage.ru_utime.tv_sec;
etm = rusage.ru_utime.tv_usec;
 
printf("\n %ld sec.  %ld usec. \n",ets-bts,etm-btm);

  WriteDepthData(Z,fname);
    

}


WriteDepthData(Z,filename)
float *Z;
char *filename;
{
int i,y1,j;
float maxx,minn;
FILE *fp;


 minn=2555555;
 maxx=-111111;

for (i=0;i<size; i++) 
for (j=0;j<size; j++)
   {
     if (Z[i+j*size] > maxx)
        maxx = Z[i+j*size];
     if (Z[i+j*size] < minn)
        minn = Z[i+j*size];
   }


for (i=0;i<size; i++) 
for (j=0;j<size; j++)
     Z[i+j*size] = (Z[i+j*size]-minn)* (MaX-MiN)/(maxx-minn) + MiN ;


  if ((fp = fopen(filename,"w")) == NULL) {
        printf("Unable to open output depth file.\n");
        exit(1);
      }
     
     for (j=0;j<size; j++)
     for (i=0;i<size; i++) 
       fprintf(fp, "%f\n", Z[j*size+i]);
       
fclose(fp);


}


