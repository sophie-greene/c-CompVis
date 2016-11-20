/* Pentland'a algorithm */

#include <sys/time.h>
#include <sys/resource.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../util/ImageTools.h"
#include "../util/UCFReadPic.c"

#define SWAP(a,b) tempr = (a);(a) = (b);(b) = tempr
#define NDIM 2      /* 2D transform */ 

float         MAX2,MIN2,*real,*imag,pi=3.1415926535; 
float         minusone;
float         a,ao,*frequency1(),*frequency() ;
float         *MI,*Magnitude();
float         lx,ly,lz,tilt,slant;
int	      size,row,max,max2, *tpic,*cospic,*sinpic;
int           CT=0,BT;
float         *freq,*freq1,B;
FILE          *file,*outfile;
PIC           image,mypic,resultpic;
char          OutFile[66];

typedef struct 
{
	int type;
	unsigned int	maxX,
			maxY;
	float	*image;
}	PIC2;

PIC2 F;


main()
{
int x,y,s;
struct rusage rusage;
long bts,btm,ets,etm;


 ReadImage();

 printf("\nEnter the minimum and maximum depth for resacle:\n");
 printf("\nMinimum : ");
 scanf("%f",&MIN2);
 printf("Maximum : ");
 scanf("%f",&MAX2);
 
 printf("\nEnter the Light Source  x y z ");
 scanf("%f %f %f",&lx,&ly,&lz);
 if(lx == 0.0 && ly == 0.0) lx = ly = 0.001;
 
 max = max2 = image.maxX;
 Create_memory();

getrusage(0,&rusage);
bts = rusage.ru_utime.tv_sec;
btm = rusage.ru_utime.tv_usec;
 
 FFT_mod();

getrusage(0,&rusage);
ets = rusage.ru_utime.tv_sec;
etm = rusage.ru_utime.tv_usec;
 
printf("\n %ld sec.  %ld usec. \n",ets-bts,etm-btm);
 
/* Scale the depth map */ 
 Normalize(F,image);

/* Output the depth */ 
 outfile = fopen(OutFile,"w"); 
 for (y=0; y<max; y++) 
  for (x=0; x<max; x++)
  {
   s = y*max + x;
   fprintf(outfile,"%f\n",F.image[s]);
  }
 fclose(outfile); 

}/* end of main */


FFT_mod()
{
float H,H2,low,high,uj;
float PH,ss,sd;
float f2,f,thet;
float costilt,sintilt,sinslant;
float fl=1;
int go,x,y,z,s,size;
float w1,w2,OK,p,q,th;
float B2,sx,ys,zs,BIG=0,SMALL=1,D,LP,t1,t2,t3;
char ch;

size = max*max;

CtoF(F,image);

FFT(F,real,imag);

for (y=0; y<max; y++) 
for (x=0; x<max; x++) {
  s = y*max + x;
  if (y < 2 || y > max-2 || x < 2 || x > max-2)
  {
    real[s] = 0;
    imag[s] = 0;
  }
  }


freq = frequency(max,fl); 
MI = (float *)(malloc(max*max*sizeof(float)));
Magnitude2(MI);


D = sqrt(lx*lx + ly*ly + lz*lz);
lx = lx/D + .0001;
ly = ly/D;
lz = lz/D;

tilt = atan2(ly,lx+.0001);
slant = acos(lz);


for (x=0; x<image.maxX*image.maxX;x++) image.image[x] = 0;

costilt = cos(tilt);
sintilt= sin(tilt);
sinslant =  sin(slant);
t3 = (float)(max - 1);
freq[max/2] = freq[max/2-1]/2;

for (y=0; y<max; y++) {
for (x=0; x<max; x++) {

       w2 = freq[x];
       w1 = freq[y];

       f = sqrt (w1 *w1 + w2 * w2);

	if (f > BIG)
	    BIG = f;
	if (f < SMALL)
	    SMALL = f;

        if (w2 == 0)
          w2 =.001;

       thet = atan2(w1,w2);

       if (thet < 0)
	    thet = 2*pi + thet;

       s = (y*max)+x;


       if (cos(tilt-thet) < 0)
       sd = -.65;
       else sd = 0.65;
       p = cos(thet);
       q = sin(thet);
       BIG = sqrt(p*p + q*q + 1*1);

       /* Pentlands */
       H = (a*a + f*f)/(2.0*ao*a + (1-ao)*(a*a+f*f));
       H2 = (a*a )/(2.0*ao*a + (1-ao)*(a*a));
          B= 2 * pi * (f) * sinslant * (sd + w2*lx + w1*ly);

             if ((p*lx + q*ly)>0.0  )
             p =  (imag[s]/B) ;
             q = (-real[s]/B) ;

	    imag[s] =  q;
            real[s] = p;
	    

}
} 

for (y=0; y<max; y++) 
for (x=0; x<max; x++) {
  s = y*max + x;
  if (y < 2 || y > max-2 || x < 2 || x > max-2)
  {
  real[s] =0; 
  imag[s] = 0;
  }
}

invfourier(F,real,imag); 

}


CtoF(F,frame)
PIC2 F;
PIC frame;
{
int s,x,y,z;

for (y=0; y<max; y++) 
for (x=0; x<max; x++) {
  s = y*max + x;

  if(frame.image[s])
  F.image[s] = ((float)frame.image[s]);
  else F.image[s] =0;
  }
  }


Normalize(F,frame)
PIC2 F;
PIC frame;
{
float low,high;
int s,x,y,z;



low = 255000000;
high = -25000000;
for (y=0; y<max; y++)
    for (x=0; x<max; x++) 
    {
        if (low > F.image[x+y*max]) low = F.image[x+y*max];
        if (high < F.image[x+y*max]) high = F.image[x+y*max];
    }


for (y=0; y<max; y++)
    for (x=0; x<max; x++) 
      F.image[y*max+x]=((F.image[(y)*max+(x)]-low) * (MAX2-MIN2)/(high-low) + MIN2);

 }

float *vector(nl,nh)
int nl,nh;
{
 float *v;

 v = (float *)malloc((unsigned) (nh-nl+1)*sizeof(float));
 if (!v) {
    printf("error in vector() \n");
    exit(-3);
  }
  return v-nl;
}

int *ivector(nl,nh)
int nl,nh;
{
 int *v;

 v = (int *) malloc((unsigned) (nh-nl+1)*sizeof(int));
 if (!v) {
    printf("error in ivector() \n");
    exit(-2);
 }
 return v-nl;
}

free_vector(v,nl,nh)
float *v;
int nl,nh;
{
 free((char*) (v+nl));
}

free_ivector(v,nl,nh)
int v,nl,nh;
{
 free((char*) (v+nl));
}

  /*     This procedure computes the inverse fourier transform          */

invfourier(ipic,real,imag)
PIC2 ipic;
float *real,*imag;
{
 float tempf,HIST[1000];
 float tr1,trd;
 float *data,*addr,tr;
 int uz,zu,TEMP;
 float SMALL=255,SMALL1,BIG=0,BIG1;
 int x,y,i,isign,j,k,l,ll,ndum,*nn,addri,size;
 unsigned char *addrc;

 nn = ivector(1,NDIM);
 data = vector(1,2*ipic.maxX*ipic.maxY);
 nn[1] = ipic.maxX;
 nn[2] = ipic.maxY;
 isign = (0-1);                         /* perform inverse fourier transform */
 for (y=1;y<=ipic.maxY;y++)
    for (x=1;x<=ipic.maxX;x++) {
	l = x + ((y-1)*ipic.maxX);
	ll = 2*l-1;
	addri = (x-1) +((y-1)*ipic.maxX);
	data[ll] = real[addri];
	data[ll+1] = imag[addri];
    }
 fourn(data,nn,NDIM,isign);


 for (y=1;y<=ipic.maxY;y++)
    for (x=1;x<=ipic.maxX;x++) {
	l = x + ((y-1)*ipic.maxX);
	ll = 2*l-1; 
	addri = (y-1) + ((x-1)*ipic.maxY);

	trd = (float) (x+y-2.0);
        tr1 = pow(minusone,trd); 
        data[ll] = (float) (tr1* data[ll]); 
	tempf = data[ll] / (float) (mypic.maxX); 
	

        ipic.image[addri] = tempf; 
     }




     free_ivector(nn,1,NDIM);
     free_vector(data,1,2*ipic.maxX*ipic.maxY);
}

FFT(ipic,Real,Imag)
PIC2 ipic;  
float *Real,*Imag;
{
 FILE *fp;
 char Ifile[80], str[40];
 int size;
 float K;

 minusone = (0-1);
 mypic.type = 0;
 mypic.maxX = ipic.maxX;
 mypic.maxY = ipic.maxY;
 size = ipic.maxX * ipic.maxY;
 /*
 Real = vector(0,size-1);
 Imag = vector(0,size-1);
*/ 
 fourier(ipic,Real,Imag); 

	/* call fourier, returns real+imag*/ 
	
 resultpic.image = (unsigned char *) malloc(mypic.maxX*mypic.maxY);
 resultpic.type = mypic.type;
 resultpic.maxX = mypic.maxX;
 resultpic.maxY = mypic.maxY;
}


  /* This procedure prepares the original image data for sending to the  */
  /* actual fourier (fourn) procedure. The image data is placed in the   */
  /* real part, and the imaginary part is left blank or initialized. The */
  /* result after calling fourn is in both the real and imaginary parts. */

fourier(ipic,real,imag)
PIC2 ipic;
float *real,*imag;
{
 float tr1,trd;
 float *data,*addr,tr;
 int x,y,i,isign,j,k,l,ll,ndum,*nn,addri,size;
 unsigned char *addrc;

 nn = ivector(1,NDIM);
 data = vector(1,2*ipic.maxX*ipic.maxY);
 nn[1] = ipic.maxX;
 nn[2] = ipic.maxY;
 for (k=1;k<=ipic.maxX;k++)
    for (j=1;j<=ipic.maxY;j++) {
	l = k+((j-1)*ipic.maxY);
	ll = 2*l-1;
	tr = (float) ipic.image[(j-1)+(ipic.maxX*(k-1))];
	trd = (float) (k+j-2.0);

      /* multiply by k+j to power -1 so that the fourier image is centered */

        tr1 = pow(minusone,trd); 
	data[ll] = (float) (tr1* (ipic.image[(j-1)+(ipic.maxX*(k-1))]));
	data[ll+1] = ll+1;
    }
 isign = 1;     /* perform forward fourier transform */
 fourn(data,nn,NDIM,isign);
 for (y=1;y<=ipic.maxY;y++)
    for(x=1;x<=ipic.maxX;x++) {
        l = x + ((y-1)*ipic.maxX);
        ll = 2*l-1;
        addri = (x-1) + ((y-1)*ipic.maxX);
        *(real + addri) = data[ll]/(float) mypic.maxX;  
        *(imag + addri) = data[ll+1]/(float) mypic.maxX;
    }
 free_ivector(nn,1,NDIM);
 free_vector(data,1,2*ipic.maxX*ipic.maxY);
}

  /* This procedure replaces data by its ndim (ndimensional) discrete
     Fourier transform if isign = 1. nn[1..ndim] is an integer array containing
     the lengths of each dimension (number of complex values), which must
     be powers of 2. data is a real array of length twice the product of these
     lengths, in which data are stored as in a multidimensional complex array:
     real and imaginary parts of each element are in consecutive locations, and
     the rightmost index of the array increases most rapidly as you proceed 
     along data. For a 2D array, this means you store by rows. If isign = -1
     then data is replaced by the INVERSE transform times the product of the
     lengths of all dimensions. 
 */

fourn(data,nn,ndim,isign)
float data[];
int nn[],ndim,isign;
{ 
 float theta,wi,wpi,wpr,wr,wtemp;
 float tempi,tempr;
 int i1,i2,i3,i2rev,i3rev,ip1,ip2,ip3,ifp1,ifp2,x;
 int ibit,idim,k1,k2,n,nprev,nrem,ntot;

  ntot = 1;
  for (idim=1;idim<=ndim;idim++) ntot *= nn[idim];
  nprev = 1;
  for (idim=ndim;idim >=1;idim--) {
     n = nn[idim];
     nrem = ntot/(n*nprev);
     ip1 = nprev << 1;
     ip2 = ip1*n;
     ip3 = ip2*nrem;
     i2rev = 1;
     for (i2=1;i2<=ip2;i2+=ip1) {
	 if (i2 < i2rev) {
             for (i1=i2;i1<=i2+ip1-2;i1+=2) {
		 for (i3=i1;i3<=ip3;i3+=ip2) {
                    i3rev = i2rev + i3 - i2;
		    SWAP(data[i3],data[i3rev]);
		    SWAP(data[i3+1],data[i3rev+1]);
                 }
             }
         }
         ibit = ip2 >> 1;
	 while ((ibit >= ip1) && (i2rev > ibit)) {
	     i2rev -= ibit;
	     ibit >>= 1;
         }
         i2rev += ibit;
    }
    ifp1 = ip1;
    while (ifp1 < ip2) {
	ifp2 = ifp1 << 1;
	theta = isign * 6.28318530717959/(ifp2/ip1);
	wtemp= sin(0.5*theta);
	wpr = -2.0*wtemp*wtemp;
	wpi = sin(theta);
	wr = 1.0;
	wi = 0.0;
	for (i3=1;i3<=ifp1;i3+=ip1) {
            for (i1 = i3;i1<=i3+ip1-2;i1+=2) {
	       for (i2=i1;i2<=ip3;i2+=ifp2) {
                   k1 = i2;
		   k2 = k1 + ifp1;
		   tempr = wr*data[k2] - wi*data[k2+1];
		   tempi = wr*data[k2+1] + wi*data[k2];  
		   data[k2] = data[k1] - tempr;
		   data[k2+1] = data[k1+1] - tempi;
		   data[k1] += tempr;
		   data[k1+1] += tempi;
                }
            } 
            wr = (wtemp=wr)*wpr-wi*wpi+wr;
	    wi = wi*wpr+wtemp*wpi+wi;
         }
         ifp1 = ifp2;
    }
    nprev *= n;
 }
}

Magnitude2(MI)
float *MI;
{
 int y,x;
  
 y = max*max;
 for (x=0;x<y;x++)
 {
 MI[x] = sqrt((real[x]*real[x]+imag[x]*imag[x]));
 if (MI[x] < 0)
    x=x;
    }
}


float *frequency(S,f)
int S;
float f;
{
 int M=1, x;
 float *v;

 v = (float *)malloc(S*sizeof(float));
 if (!v) {
     printf("error in freq() \n");
     exit(-3);
 }
 for (x=0; x<S/2; x++) {
     v[x] = (float) -.5+ (x/(S*f)) ;
     v[x+S/2] = (float) ( x)/(S*f) ;
 }
 return v;
}

 /* this module reads in the image file */

ReadImage()
{
 char fname[101];

 printf("\nEnter filename for the input image : ");
 scanf("%s",fname);
 if((file = fopen(fname,"r")) != NULL) image = UCFReadPic(file); 
 else {
    printf("\nUNABLE TO OPEN THE FILE");
    exit(-1);
 }
 fclose(file);

 printf("\nEnter the Output filename : ");
 scanf("%s",OutFile);
}
 

/* allocating space in the heap */ 
  
Create_memory()
{
 unsigned int s;

 s = max*max;

 imag = (float *)calloc(max2*max2,sizeof(float));
 if( imag == NULL) {
     printf(" SCRIPT TABLE ALLOCATION FAILURE! \n");
     exit(0);
}
 real = (float *)calloc(max2*max2,sizeof(float));
 if( real == NULL) {
     printf(" SCRIPT TABLE ALLOCATION FAILURE! \n");
     exit(0);
}
 F.image = (float *)calloc(max2*max2,sizeof(float));
 if( F.image == NULL) {
     printf(" SCRIPT TABLE ALLOCATION FAILURE! \n");
     exit(0);
}
    F.type = 0;
    F.maxX = max;
    F.maxY = max;

}







