#include <stdio.h>
#include <math.h>
#include "../util/ImageTools.h"

/*
#define IMG_SIZE 5
#define ARRAY_SIZE (IMG_SIZE * 2)
*/
#define ARRAY_SIZE 16
#define MAXR (ARRAY_SIZE * 2)

int row[MAXR];                                   /* vars for matrix inverse */
double A[ARRAY_SIZE][MAXR];

/*
** compute the matrix inverse
*/
calc_inverse(alpha)
    double alpha[ARRAY_SIZE][ARRAY_SIZE];
{
    int i, j;

    for (i = 0; i < ARRAY_SIZE; i++)
        for (j = 0; j < ARRAY_SIZE; j++)
            A[i][j] = alpha[i][j];

    for (i = 0; i < ARRAY_SIZE; i++)
        for (j = ARRAY_SIZE; j < (ARRAY_SIZE * 2); j++)
            if ((j - ARRAY_SIZE) == i)
                A[i][j] = 1.0;
            else
                A[i][j] = 0.0;

printf("Output left of A: ------\n");
for (i = 0; i < ARRAY_SIZE; i++) {
    for (j = 0; j < ARRAY_SIZE; j++) {
	printf("A[%d][%d] = %lf", i, j, A[i][j]);
    }
    printf("\n");
}

    if (inverse(ARRAY_SIZE) != 0) {
        printf("Can not invert matrix, stop.\n");
        exit(1);
    }

    for (i = 0; i < ARRAY_SIZE; i++)
        for (j = 0; j < ARRAY_SIZE; j++)
            alpha[i][j] = A[i][j+ARRAY_SIZE];
}




/*
** routine for computing the inverse of a n by n matrix
*/
inverse(n)
int n;
{
    int i, j, k, copy, p;
    double m[MAXR];

    for (i = 0;  i <= n;  i++) {
        row[i] = i;
    }

    for (i = 0;  i < n;  i++) {
        p = nonzero(i, n);
        if (A[row[p]][i] == 0.0) {
            return(1);
        }

        if (row[i] != row[p]) {
            copy = row[i];
            row[i] = row[p];
            row[p] = copy;
        }

        if ((m[0] = A[row[i]][i]) == 0.0) {
            return(1);
        }

        for (j = i;  j < (n + n);  j++) {
            A[row[i]][j] /= m[0];
        }

        for (j = 0;  j < i;  j++) {
            m[j] = A[row[j]][i];
            for (k = i;  k < (n + n);  k++) {
                A[row[j]][k] -= (A[row[i]][k] * m[j]);
            }
        }

        for (j = i + 1;  j < n;  j++) {
            m[j] = A[row[j]][i];
            for (k = i;  k < (n + n);  k++) {
                A[row[j]][k] -= (A[row[i]][k] * m[j]);
            }
        }
    }

    return(0);
}




/*
** check for the matrix rank
*/
nonzero(c,n)
int c,n;
{
    int k;

    for(k = c;  (k < n) && (A[row[k]][c] == 0.0);  k++);

    return(k);
}
