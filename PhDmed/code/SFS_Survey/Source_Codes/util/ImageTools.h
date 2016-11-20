/********************************************************/
/* ImageTools.h   Useful picture related routines.      */
/*                See ImageTools.readme for examples.   */
/********************************************************/
typedef struct
{
        short type;
	short maxX,
              maxY;
	unsigned char	*image;
}	PICIM;

typedef struct
{  int type;
   int maxX,maxY;
   int *image;
} INTIMAGE;

typedef struct
{
	int type;
	unsigned int	maxX,
			maxY;
	unsigned char	*image;
}	PIC;

typedef struct
{
	unsigned int	maxx,
			maxy;
}	type0header;

#define pixel(x,y,p)	((p)->image[(x)+(y)*((p)->maxX)])
#define width(p)	((p)->maxX)
#define height(p)	((p)->maxY)
#define ABS(x)          ((x)<0 ? -(x):(x))

#define TRUE            1
#define FALSE           0
#define sign(a)		(((a) == 0) ? 0 : ((a) > 0) ? (a) : -(a))
#define MAX(a,b)	(((a) < (b)) ? (b) : (a))
#define MIN(a,b)	(((a) > (b)) ? (b) : (a))
#define SQR(a)		((a) * (a))

typedef long int HISTTYPE[256];
typedef unsigned char BYTE;
typedef unsigned char BOOLEAN;
typedef double FLOAT;

int rotX,rotY,rotZ;          /* Next three lines for special file formats */
unsigned int maxpic,minpic;
FLOAT maxrange,minrange; 

/************************************************/
/* Function prototypes                          */
/************************************************/
PIC ReadPic();
void WritePic();
PIC UCFReadPic();
void UCFWritePic();
int DisplayPic();
void DisplayHist();

/* Other Pic Information Definitions -- Archaic */

/*
 typedef struct          
 {                      
   unsigned int gray,lngth; 
 }	rlc;                 

typedef struct stuff
{
	int		xval,
	        	yval;
	struct stuff	*next;
}	pel;

typedef struct
{
	int	number;
	float	normn;
	pel	*head,
		*tail;
}	element;

*/
