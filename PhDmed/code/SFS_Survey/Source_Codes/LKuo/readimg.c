#include <stdio.h>
#include <math.h>
#include "../util/ImageTools.h"

#include "DEF.h"

double Ei[ARRAY_SIZE][ARRAY_SIZE];		/* intensity         */

extern int IMG_SIZE;

readimg()
{
    int i, j;
    char filename[80];
    FILE *fp;
    PIC pic;

    fprintf(stderr, "Input image file name:");
    scanf("%s", filename);

    while ((fp = fopen(filename, "r")) == NULL) {
        fprintf(stderr, "Can't open the input image file!\n");
	fprintf(stderr, "Input image file name:");
	scanf("%s", filename);
    }
			 
    pic = UCFReadPic(fp);
    fclose(fp);

    if (pic.maxX != pic.maxY)
    {
	fprintf(stderr, "readimg:  not a square image.\n");
	exit(1);
    }
    else
    {
	IMG_SIZE = pic.maxX;
    }

    for (i = 0; i < pic.maxY; i++) {
        for (j = 0; j < pic.maxX; j++) {
	    Ei[i][j] = ( (double) pixel(j, i, &pic) ) / 255.0;
	}
    }
}
