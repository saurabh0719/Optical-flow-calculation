#include <bits/stdc++.h>

using namespace std;

#define W 854				// Frame width
#define H 480				// Frame height
#define maxFeatures 4		// Maximum number of corners per frame
#define maxFeaturesH 2
#define maxFeaturesW (maxFeatures/maxFeaturesH)
#define Threshold 250000	// 5150000 for city drone video // 1250000 for beach highway drone video
#define corner_count 4
#define window 9

/*******************GAUSSIAN FILTER****************************************************************/

//Function to create Gaussian filter
void FilterCreation(double GKernel[][5])
{
    // intialising standard deviation to 1.0
    double sigma = 1.0;
    double r, s = 2.0 * sigma * sigma;

    // sum is for normalization
    double sum = 0.0;

    // generating 5x5 kernel
    for (int x = -2; x <= 2; x++) {
        for (int y = -2; y <= 2; y++) {
            r = sqrt(x * x + y * y);
            GKernel[x + 2][y + 2] = (exp(-(r * r) / s)) / (M_PI * s);
            sum += GKernel[x + 2][y + 2];
        }
    }

    // normalising the Kernel
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            GKernel[i][j] /= sum;
}

/**************************************************************************************************/

/********************TRANSPOSE OF A MATRIX*********************************************************/

// This function stores transpose of A[][] in B[][]
void transpose(int A[][2], int B[][window])
{
    int i, j;
    for (i = 0; i < N; i++)
        for (j = 0; j < N; j++)
            B[i][j] = A[j][i];
}

/**************************************************************************************************/

/**************INVERSE OF A MATRIX*****************************************************************/

// Function to get cofactor of A[p][q] in temp[][]. n is current
// dimension of A[][]
void getCofactor(int A[2][2], int temp[2][2], int p, int q, int n)
{
    int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < n; row++)
    {
        for (int col = 0; col < n; col++)
        {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q)
            {
                temp[i][j++] = A[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

/* Recursive function for finding determinant of matrix.
   n is current dimension of A[][]. */
int determinant(int A[2][2], int n)
{
    int D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1)
        return A[0][0];

    int temp[2][2]; // To store cofactors

    int sign = 1;  // To store sign multiplier

     // Iterate for each element of first row
    for (int f = 0; f < n; f++)
    {
        // Getting Cofactor of A[0][f]
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

// Function to get adjoint of A[N][N] in adj[N][N].
void adjoint(int A[2][2],int adj[2][2], int N)
{
    if (N == 1)
    {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store cofactors of A[][]
    int sign = 1, temp[N][N];

    for (int i=0; i<N; i++)
    {
        for (int j=0; j<N; j++)
        {
            // Get cofactor of A[i][j]
            getCofactor(A, temp, i, j, N);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant(temp, N-1));
        }
    }
}

// Function to calculate and store inverse, returns false if
// matrix is singular
bool inverse(int A[2][2], float inverse[2][2])
{
    // Find determinant of A[][]
    int det = determinant(A, 2);
    if (det == 0)
    {
        //cout << "Singular matrix, can't find its inverse";
        return false;
    }

    // Find adjoint
    int adj[2][2];
    adjoint(A, adj, 2);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<2; i++)
        for (int j=0; j<2; j++)
            inverse[i][j] = adj[i][j]/float(det);

    return true;
}

/**************************************************************************************************/

/*************************MATRIX MULTIPLICATION****************************************************/

//#define N 4

// This function multiplies
// mat1[][] and mat2[][], and
// stores the result in res[][]
void multiply_1(int mat1[][window],
              int mat2[][2],
              int res[2][2])
{
    int i, j, k;
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 2; j++)
        {
            res[i][j] = 0;
            for (k = 0; k < window; k++)
                res[i][j] += mat1[i][k] *
                             mat2[k][j];
        }
    }
}


void multiply_2(int mat1[][2],
              int mat2[][window],
              int res[2][window])
{
    int i, j, k;
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < window; j++)
        {
            res[i][j] = 0;
            for (k = 0; k < 2; k++)
                res[i][j] += mat1[i][k] *
                             mat2[k][j];
        }
    }
}


void multiply_final(int mat1[][window],
              int mat2[][1],
              int res[2][1])
{
    int i, j, k;
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 1; j++)
        {
            res[i][j] = 0;
            for (k = 0; k < window; k++)
                res[i][j] += mat1[i][k] *
                             mat2[k][j];
        }
    }
}

/**************************************************************************************************/


// pass int status[corner_count] = {0};
//pass the matrices for both frames;

void opticalFlow(unsigned char ref[H][W], unsigned char next[H][W], int inputquad[corner_count][2], int outputquad[corner_count][2], int status[corner_count], String err)
{

    int Ix[window] ={0};
    int Iy[window] ={0};
    int It[window][1] ={0}; // also the t vector of the equation
    //store the negative values directly in this ( -It)

    int i =0;
    while(i< corner_count)
    {
      // (u v)t = (StS)^-1 * St. tvector
      //calculate Ix[9] Iy[9] and It[9] for each corner

      int S[window][2];

      for(int i =0; i<window; i++)
      S[i][1] = Ix[i];

      for(int i =0; i<window; i++)
      S[i][2] = Iy[i];

      int ST[2][window]; // S transpose

      transpose(S,ST);

      int STS[2][2];// stores S transpose * S

      multiply_1(ST,S,STS);

      float STS_inverse[2][2];

      if(!inverse(STS,STS_inverse)
      {
        err = "Couldn't find inverse of the singular matrix";
        break;
      }

      float S_final[2][window];

      multiply_2(STS_inverse,ST, S_final);

      float res[2][1];

      multiply_final(S_final, It, res);

      outputquad[i][0] = res[0][0];
      outputquad[i][1] = res[1][0];

    }

    return 0;
}
