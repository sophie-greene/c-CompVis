#include <math.h>
#include "../util/ImageTools.h"
#include "DEF.h"

typedef struct VERTEX { int x, y; double z; } Vertex;

extern double Ei[ARRAY_SIZE][ARRAY_SIZE];	/* intensity         */
extern int nGrid;
extern double C[MaxGrids][ARRAY_SQ][13];	/* stiffness matrix  */
extern double Sx, Sy, Sz;
extern int IMG_SIZE;
extern int i_cntr;

calc_C_b(Z, size, h, rhs)
    double *Z, *rhs;
    int size, h;
{
    int i;

    calc_A_b(Z, h, size, rhs);
    Add_B2A(h, size);
}



calc_A_b(Z, h, size, b)
    double *Z, *b;
    int h, size;
{
    int i, j;
    int Mn;
int x,y;
    double ak, bk, al, bl, am, bm, an, bn, ao, bo, ap, bp;
    double p0, q0;
    double Ek, El, Em, En, Eo, Ep;
    double ck, cl, cm, cn, co, cp;
    double intensity(), alpha(), beta();
    double nx, ny, nz;

    Mn = size * size;

    for (i = 0; i < Mn; i++) {
	if ((((i+1) % size) != 0) && (i-size >= 0)) /* Tk */
	{
	    calc_grad(Z, h, &p0, &q0, i, i+1, i-size+1);
	    ak = alpha(p0, q0);
	    bk = beta(p0, q0);
	    Ek = intensity(Z, h, i, i-size+1, i+1);
	    ck = (Sz - Sx*p0 - Sy*q0) / sqrt(p0*p0+q0*q0+1.0) - ak*p0 - bk*q0;
	    check_ab(ak, bk, 'k', i);
	}
	else
	{
	    ak = bk = 0.0;
	    Ek = 0.0;
	    ck = 0.0;
	}
	if ((((i+1) % size) != 0) && (i-size >= 0)) /* Tl */
	{
	    calc_grad(Z, h, &p0, &q0, i, i-size+1, i-size);
	    al = alpha(p0, q0);
	    bl = beta(p0, q0);
	    El = intensity(Z, h, i, i-size+1, i-size);
	    cl = (Sz - Sx*p0 - Sy*q0) / sqrt(p0*p0+q0*q0+1.0) - al*p0 - bl*q0;
	    check_ab(al, bl, 'l', i);
	}
	else
	{
	    al = bl = 0.0;
	    El = 0.0;
	    cl = 0.0;
	}
	if (((i-size) >= 0) && ((i % size) != 0) ) /* Tm */
	{
	    calc_grad(Z, h, &p0, &q0, i, i-size, i-1);
	    am = alpha(p0, q0);
	    bm = beta(p0, q0);
	    Em = intensity(Z, h, i-size, i-1, i);
	    cm = (Sz - Sx*p0 - Sy*q0) / sqrt(p0*p0+q0*q0+1.0) - am*p0 - bm*q0;
	    check_ab(am, bm, 'm', i);
	}
	else
	{
	    am = bm = 0.0;
	    Em = 0.0;
	    cm = 0.0;
	}
	if (((i % size) != 0) && ((i+size) < (Mn)))/*Tn*/
	{
	    calc_grad(Z, h, &p0, &q0, i, i-1, i+size-1);
	    an = alpha(p0, q0);
	    bn = beta(p0, q0);
	    En = intensity(Z, h, i, i+size-1, i-1);
	    cn = (Sz - Sx*p0 - Sy*q0) / sqrt(p0*p0+q0*q0+1.0) - an*p0 - bn*q0;
	    check_ab(an, bn, 'n', i);
	}
	else
	{
	    an = bn = 0.0;
	    En = 0.0;
	    cn = 0.0;
	}
	if (((i % size) != 0) && ((i+size) < (Mn)))/*To*/
	{
	    calc_grad(Z, h, &p0, &q0, i, i+size-1, i+size);
	    ao = alpha(p0, q0);
	    bo = beta(p0, q0);
	    Eo = intensity(Z, h, i, i+size-1, i+size);
	    co = (Sz - Sx*p0 - Sy*q0) / sqrt(p0*p0+q0*q0+1.0) - ao*p0 - bo*q0;
	    check_ab(ao, bo, 'o', i);
	}
	else
	{
	    ao = bo = 0.0;
	    Eo = 0.0;
	    co = 0.0;
	}
	if ( (((i+1) % size) != 0) && ((i+size) < (Mn)) )/* Tp */
	{
	    calc_grad(Z, h, &p0, &q0, i, i+size, i+1);
	    ap = alpha(p0, q0);
	    bp = beta(p0, q0);
	    Ep = intensity(Z, h, i+size, i+1, i);
	    cp = (Sz - Sx*p0 - Sy*q0) / sqrt(p0*p0+q0*q0+1.0) - ap*p0 - bp*q0;
	    check_ab(ap, bp, 'p', i);
	}
	else
	{
	    ap = bp = 0.0;
	    Ep = 0.0;
	    cp = 0.0;
	}

	if (h == 1) {
	    b[i] = h * (-ak*(Ek-ck) - bl*(El-cl) + (am-bm)*(Em-cm) +
		    an*(En-cn) + bo*(Eo-co) + (bp-ap)*(Ep-cp));
	}

	C[h-1][i][0] = SQR(ak)+SQR(bl)+SQR(am-bm)+SQR(an)+SQR(bo)+SQR(ap-bp);

	if (((i+1) % size) != 0) /* j1 */
	    C[h-1][i][1] = ak*(bk-ak) + ap*(bp-ap);

	if ((((i+1) % size) != 0) && (i-size >= 0)) /* j2 */
	    C[h-1][i][2] = -(ak*bk + al*bl);

	if ((i-size) >= 0) /* j3 */
	    C[h-1][i][3] = bl*(al-bl) + bm*(am-bm);

	if ((i % size) != 0) /* j4 */
	    C[h-1][i][4] = am*(bm-am) + an*(bn-an);

	if (((i % size) != 0) && ((i+size) < (Mn)))/*j5*/
	    C[h-1][i][5] = -(an*bn + ao*bo);

	if ((i+size) < (Mn)) /* j6 */
	    C[h-1][i][6] = bo*(ao-bo) + bp*(ap-bp);
    }   /* end of for i */
}



/* the intensity at k is counted twice */
double intensity(Z, h, i, j, k)
    double *Z;
    int h, i, j, k;
{
    double E;
    Vertex v1, v2, v3;
    double edge, inner, corner;
    int x, y, stepX, stepY, limX, limY;

    calc_coord(Z, h, &v1, i);
    calc_coord(Z, h, &v2, j);
    calc_coord(Z, h, &v3, k);

    if (v1.x == v3.x) {
	limX = v2.x;
	limY = v1.y;
    } else {
	limX = v1.x;
	limY = v2.y;
    }

    if (limY > v3.y) stepY = 1; else stepY = -1;
    if (limX > v3.x) stepX = 1; else stepX = -1;

    inner = 0.0;
    edge = 0.0;

    for (y=v3.y;  y!=limY;  y+=stepY) {
	for (x=v3.x;  x!=limX;  x+=stepX) {
	    if ( ((x==v3.x)&&(y==v3.y)) ||
		 ((x==v2.x)&&(y==v2.y)) ||
		 ((x==v1.x)&&(y==v1.y)) ) {
		continue; /* it's a corner point! */
	    }
	    if ( (x==v3.x) || (y==v3.y) || (x==limX) ) {
		edge += Ei[y][x];
		continue;
	    }
	    inner += Ei[y][x];
	}
	limX -= stepX;
    }

    corner = Ei[v1.y][v1.x] + Ei[v2.y][v2.x] + 2.0*Ei[v3.y][v3.x];
    E = 2.0 * (inner + edge/2.0 + corner/8.0) / (h*h);
    return (E);
}




double alpha(p, q)
  double p, q;
{
    double t, t2;

    t = p*p + q*q + 1.0;
    t2 = ( (-Sx*t + p*(p*Sx + q*Sy - Sz)) / (t*sqrt(t)) );

    return(t2);
}




double beta(p, q)
  double p, q;
{
    double t, t2;

    t = p*p + q*q + 1.0;
    t2 = ( (-Sy*t + q*(p*Sx + q*Sy - Sz)) / (t*sqrt(t)) );
    return(t2);
}




calc_grad(Z, h, p, q, i, j, k)
    double *Z, *p, *q;
    int h, i, j, k;
{
    Vertex v1, v2, v3;
    Vertex u, v;
    double nx, ny, nz;

    *p = 0.0;
    *q = 0.0;

    /*
    **  global linearization for the first iteration
    */
    if (i_cntr == 0) return;

    calc_coord(Z, h, &v1, i);
    calc_coord(Z, h, &v2, j);
    calc_coord(Z, h, &v3, k);

    u.x = v2.x - v1.x;
    u.y = v2.y - v1.y;
    u.z = v2.z - v1.z;

    v.x = v3.x - v1.x;
    v.y = v3.y - v1.y;
    v.z = v3.z - v1.z;

    nx = ((double) u.y) * v.z - ((double) v.y) * u.z;
    ny = ((double) v.x) * u.z - ((double) u.x) * v.z;
    nz = ((double) u.x) * ((double) v.y) - ((double) v.x) * ((double) u.y);

    if (nz != 0.0) {
      *p = -nx / nz;
      *q = -ny / nz;
    }
}




calc_coord(Z, h, v, i)
    double *Z;
    Vertex *v;
    int h, i;
{
    int t; 	/* x, y, size; */

    t = (1 << (h-1));
    v->x = (i * t) % (int)IMG_SIZE;
    v->y = (i * t) / (int)IMG_SIZE;
    v->z = Z[((v->y) * IMG_SIZE) + v->x];
}




check_ab(alpha, beta, chr, i)
    double alpha, beta;
    char chr;
    int i;
{
    /*
    **   return immediately == NO CHECK
    */
    return;

    if (alpha == 0.0)
    {
        printf("a%c = 0.0 at %d\n", chr, i);
    }
    if (beta == 0.0)
    {
        printf("b%c = 0.0 at %d\n", chr, i);
    }
    if (alpha == beta)
    {
        printf("a%c = b%c at %d\n", chr, chr, i);
    }
}





Add_B2A(h, size)
    int h, size;
{
    int i;
    double constant;
    int Mn;

    Mn = size * size;

    constant = LAMBDA / (((double)h) * ((double)h));

    for (i = 0; i < Mn; i++) {
	/* corners */
	if (i == 0) {			/* upper left corner */
	    C[h-1][i][0] += (4.0*constant);
	    C[h-1][i][1] += (-4.0*constant);
	    C[h-1][i][6] += (-4.0*constant);

	    C[h-1][i][12] = (2.0*constant);
	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    continue;
	}

	if (i == (size-1)) {	/* upper right corner */
	    C[h-1][i][0] += (4.0*constant);
	    C[h-1][i][4] += (-4.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-4.0*constant);

	    C[h-1][i][10] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    continue;
	}

	if (i == (Mn-size)) {	/* lower left corner */
	    C[h-1][i][0] += (4.0*constant);
	    C[h-1][i][1] += (-4.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-4.0*constant);

	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][8] = (1.0*constant);
	    continue;
	}

	if (i == (Mn-1)) {		/* lower right corner */
	    C[h-1][i][0] += (4.0*constant);
	    C[h-1][i][3] += (-4.0*constant);
	    C[h-1][i][4] += (-4.0*constant);

	    C[h-1][i][8] = (1.0*constant);
	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][10] = (1.0*constant);
	    continue;
        }

	/* next to the corners */
	if (i == 1) {			/* next to the upper left corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][1] += (-6.0*constant);
	    C[h-1][i][4] += (-4.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-6.0*constant);

	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if (i == (size)) {		/* next to the upper left corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][1] += (-6.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-4.0*constant);
	    C[h-1][i][6] += (-6.0*constant);

	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if (i == (size-2)) {	/* next to the upper right corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][1] += (-4.0*constant);
	    C[h-1][i][4] += (-6.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-6.0*constant);

	    C[h-1][i][10] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if (i == (2*size-1)) {	/* next to the upper right corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][3] += (-4.0*constant);
	    C[h-1][i][4] += (-6.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-6.0*constant);

	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][10] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    continue;
	}

	if (i == (Mn-2*size)) {	/* next to the lower left corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][1] += (-6.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-6.0*constant);
	    C[h-1][i][6] += (-4.0*constant);

	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][8] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if (i == (Mn-size+1)) {	/* next to the lower left corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][1] += (-6.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-6.0*constant);
	    C[h-1][i][4] += (-4.0*constant);

	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][8] = (1.0*constant);
	    continue;
	}

	if (i == (Mn-size-1)) {	/* next to the lower right corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][3] += (-6.0*constant);
	    C[h-1][i][4] += (-6.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-4.0*constant);

	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][8] = (1.0*constant);
	    C[h-1][i][10] = (1.0*constant);
	    continue;
	}

	if (i == (Mn-2)) {	/* next to the lower right corner */
	    C[h-1][i][0] += (10.0*constant);
	    C[h-1][i][1] += (-4.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-6.0*constant);
	    C[h-1][i][4] += (-6.0*constant);

	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][8] = (1.0*constant);
	    C[h-1][i][10] = (1.0*constant);
	    continue;
	}

	/* diagnally from the corners */
	if (i == (size+1)) {	/* from the upper left corner */
	    C[h-1][i][0] += (18.0*constant);
	    C[h-1][i][1] += (-8.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-6.0*constant);
	    C[h-1][i][4] += (-6.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-8.0*constant);

	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if (i == (2*size-2)) {	/* from the upper right corner */
	    C[h-1][i][0] += (18.0*constant);
	    C[h-1][i][1] += (-6.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-6.0*constant);
	    C[h-1][i][4] += (-8.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-8.0*constant);

	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][10] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if (i == (Mn-2*size+1)) {	/* from the lower left corner */
	    C[h-1][i][0] += (18.0*constant);
	    C[h-1][i][1] += (-8.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-8.0*constant);
	    C[h-1][i][4] += (-6.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-6.0*constant);

	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][8] = (1.0*constant);
	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if (i == (Mn-size-2)) {	/* from the lower right corner */
	    C[h-1][i][0] += (18.0*constant);
	    C[h-1][i][1] += (-6.0*constant);
	    C[h-1][i][2] += (2.0*constant);
	    C[h-1][i][3] += (-8.0*constant);
	    C[h-1][i][4] += (-8.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-6.0*constant);

	    C[h-1][i][8] = (1.0*constant);
	    C[h-1][i][9] = (2.0*constant);
	    C[h-1][i][10] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	/* On the boundaries */
	if ( (i >= 2) && (i <= (size-3)) ) {		/* top row */
	    C[h-1][i][0] += (11.0*constant);
	    C[h-1][i][1] += (-6.0*constant);
	    C[h-1][i][4] += (-6.0*constant);
	    C[h-1][i][5] += (2.0*constant);
	    C[h-1][i][6] += (-6.0*constant);

	    C[h-1][i][7] = (1.0*constant);
	    C[h-1][i][10] = (1.0*constant);
	    C[h-1][i][11] = (1.0*constant);
	    C[h-1][i][12] = (2.0*constant);
	    continue;
	}

	if ( (i >= (Mn-size+2)) && (i <= (Mn-3)) ) {/* bottom row */
	    C[h-1][i][0] += (constant*11.0);
	    C[h-1][i][1] += (-constant*6.0);
	    C[h-1][i][2] += (constant*2.0);
	    C[h-1][i][3] += (-constant*6.0);
	    C[h-1][i][4] += (-constant*6.0);

	    C[h-1][i][7] = (constant*1.0);
	    C[h-1][i][8] = (constant*1.0);
	    C[h-1][i][9] = (constant*2.0);
	    C[h-1][i][10] = (constant*1.0);
	    continue;
	}

	if ( ( (i%size) == 0 )
	     && (i > size)
	     && (i < (Mn - 2 * size) ) ) {/* left column */
	    C[h-1][i][0] += (constant*11.0);
	    C[h-1][i][1] += (-constant*6.0);
	    C[h-1][i][2] += (constant*2.0);
	    C[h-1][i][3] += (-constant*6.0);
	    C[h-1][i][6] += (-constant*6.0);

	    C[h-1][i][7] = (constant*1.0);
	    C[h-1][i][8] = (constant*1.0);
	    C[h-1][i][11] = (constant*1.0);
	    C[h-1][i][12] = (constant*2.0);
	    continue;
	}

	if ( ( ((i+1)%size) == 0 )
	     && (i > (2 * size - 1))
	     && (i < (Mn - size - 1)) ) {/* right */
	    C[h-1][i][0] += (constant*11.0);
	    C[h-1][i][3] += (-constant*6.0);
	    C[h-1][i][4] += (-constant*6.0);
	    C[h-1][i][5] += (constant*2.0);
	    C[h-1][i][6] += (-constant*6.0);

	    C[h-1][i][8] = (constant*1.0);
	    C[h-1][i][9] = (constant*2.0);
	    C[h-1][i][10] = (constant*1.0);
	    C[h-1][i][11] = (constant*1.0);
	    continue;
	}

	/* Next to the border */
        if ( (i >= (size+2)) && (i <= (2*size-3)) ) {	/* top */
	    C[h-1][i][0] += (constant*19.0);
	    C[h-1][i][1] += (-constant*8.0);
	    C[h-1][i][2] += (constant*2.0);
	    C[h-1][i][3] += (-constant*6.0);
	    C[h-1][i][4] += (-constant*8.0);
	    C[h-1][i][5] += (constant*2.0);
	    C[h-1][i][6] += (-constant*8.0);

	    C[h-1][i][7] = constant*1.0;
	    C[h-1][i][9] = constant*2.0;
	    C[h-1][i][10] = constant*1.0;
	    C[h-1][i][11] = constant*1.0;
	    C[h-1][i][12] = constant*2.0;
	    continue;
	}

        if ( (i >= (Mn-2*size+2)) && (i <= (Mn-size-3)) ) {/* bottom */
	    C[h-1][i][0] += (constant*19.0);
	    C[h-1][i][1] += (-constant*8.0);
	    C[h-1][i][2] += (constant*2.0);
	    C[h-1][i][3] += (-constant*8.0);
	    C[h-1][i][4] += (-constant*8.0);
	    C[h-1][i][5] += (constant*2.0);
	    C[h-1][i][6] += (-constant*6.0);

	    C[h-1][i][7] = constant*1.0;
	    C[h-1][i][8] = constant*1.0;
	    C[h-1][i][9] = constant*2.0;
	    C[h-1][i][10] = constant*1.0;
	    C[h-1][i][12] = constant*2.0;
	    continue;
	}

	if ( (((i-1) % size) == 0) &&
	      (i > 2*size) && (i < (Mn - 2*size)) ) {/* left */
	    C[h-1][i][0] += (constant*19.0);
	    C[h-1][i][1] += (-constant*8.0);
	    C[h-1][i][2] += (constant*2.0);
	    C[h-1][i][3] += (-constant*8.0);
	    C[h-1][i][4] += (-constant*6.0);
	    C[h-1][i][5] += (constant*2.0);
	    C[h-1][i][6] += (-constant*8.0);

	    C[h-1][i][7] = constant*1.0;
	    C[h-1][i][8] = constant*1.0;
	    C[h-1][i][9] = constant*2.0;
	    C[h-1][i][11] = constant*1.0;
	    C[h-1][i][12] = constant*2.0;
	    continue;
	}

	if ( (((i+2) % size) == 0) &&
	      (i > 2*size) && (i < (Mn - 2*size)) ) {/* right */
	    C[h-1][i][0] += (constant*19.0);
	    C[h-1][i][1] += (-constant*6.0);
	    C[h-1][i][2] += (constant*2.0);
	    C[h-1][i][3] += (-constant*8.0);
	    C[h-1][i][4] += (-constant*8.0);
	    C[h-1][i][5] += (constant*2.0);
	    C[h-1][i][6] += (-constant*8.0);

	    C[h-1][i][8] = constant*1.0;
	    C[h-1][i][9] = constant*2.0;
	    C[h-1][i][10] = constant*1.0;
	    C[h-1][i][11] = constant*1.0;
	    C[h-1][i][12] = constant*2.0;
	    continue;
	}

        /* interior */
	C[h-1][i][0] += (constant*20.0);
	C[h-1][i][1] += (-constant*8.0);
	C[h-1][i][2] += (constant*2.0);
	C[h-1][i][3] += (-constant*8.0);
	C[h-1][i][4] += (-constant*8.0);
	C[h-1][i][5] += (constant*2.0);
	C[h-1][i][6] += (-constant*8.0);

	C[h-1][i][7] = (constant*1.0);
	C[h-1][i][8] = (constant*1.0);
	C[h-1][i][9] = (constant*2.0);
	C[h-1][i][10] = (constant*1.0);
	C[h-1][i][11] = (constant*1.0);
	C[h-1][i][12] = (constant*2.0);
    }    /* for i */
}
