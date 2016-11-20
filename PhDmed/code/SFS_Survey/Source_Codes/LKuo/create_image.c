#include <stdio.h>
#include <math.h>
#include "../util/ImageTools.h"

#include "DEF.h"

create_image(filename, matrix, img_size)
char *filename;
double matrix[ARRAY_SIZE*ARRAY_SIZE];
int img_size;
{
    int x, y;
    double max_val, min_val;
    FILE *outfile;
    PIC outpic;

    if ((outpic.image = (unsigned char *)malloc(img_size*img_size)) == NULL)
    {
	fprintf(stderr, "Malloc failed in create_image()\n");
	exit(1);
    }
    outpic.type = 0;
    outpic.maxX = outpic.maxY = img_size;

    if ( (outfile = fopen(filename, "w")) == NULL)
    {
	fprintf(stderr, "Can't open '%s' in create_image()\n", filename);
	exit(1);
    }

    max_val = -1e99;
    min_val = 1e99;
    for (y = 0;  y < img_size;  y++)
    {
	for (x = 0;  x < img_size;  x++)
	{
	    max_val = max(max_val, matrix[x+img_size*y]);
	    min_val = min(min_val, matrix[x+img_size*y]);
	}
    }
fprintf(stderr, "image name:  %s\n", filename);
fprintf(stderr, "\tmax_val = %lg\n", max_val);
fprintf(stderr, "\tmin_val = %lg\n", min_val);
    for (y = 0;  y < img_size;  y++)
    {
	for (x = 0;  x < img_size;  x++)
	{
	    outpic.image[x+y*img_size] =
	         (unsigned char) (((matrix[x+img_size*y]-min_val)/(max_val-min_val))*255.0 + 0.5);
	}
    }
    UCFWritePic(outpic, outfile);
    fclose(outfile);
}
