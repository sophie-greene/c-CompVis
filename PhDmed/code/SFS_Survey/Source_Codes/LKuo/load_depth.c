#include <stdio.h>

#define index(M,x,y) M[x+y*IMG_SIZE]

extern int IMG_SIZE;

load_depth(Z)
    double *Z;
{
FILE	*imagefile;
char	ifile_1[80],ofile_1[80];
int i,j,N;

 fprintf(stderr, "Input depth map filename :");
 scanf("%s",ifile_1);
 if((imagefile = fopen(ifile_1,"r")) == NULL) {
     printf("Unable to open image file.\n");
     exit(1);
  }

 N = IMG_SIZE;

  for(j=0;j<N;j++)
    for(i=0;i<N;i++)
      fscanf(imagefile,"%lf",&index(Z,i,j));
  fclose(imagefile);

}
