//to get the 3*3 transformation matrix

/*  Calculates a perspective transform from four pairs of the corresponding points.

The function calculates the 3×3 matrix of a perspective transform so that:

⎡⎣⎢⎢⎢tix′itiy′iti⎤⎦⎥⎥⎥=𝚖𝚊𝚙\_𝚖𝚊𝚝𝚛𝚒𝚡⋅⎡⎣⎢⎢xiyi1⎤⎦⎥⎥

where

dst(i)=(x′i,y′i),src(i)=(xi,yi),i=0,1,2,3 */

//top-left , top right, bottom-right, bottom-left

#define corner_count 4

/**************INVERSE OF A MATRIX*****************************************************************/

// Function to get cofactor of A[p][q] in temp[][]. n is current
// dimension of A[][]
void getCofactor(int A[8][8], int temp[8][8], int p, int q, int n)
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
int determinant(int A[8][8], int n)
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
void adjoint(int A[8][8],int adj[8][8], int N)
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
bool inverse(int A[8][8], float inverse[8][8])
{
    // Find determinant of A[][]
    int det = determinant(A, 8);
    if (det == 0)
    {
        //cout << "Singular matrix, can't find its inverse";
        return false;
    }

    // Find adjoint
    int adj[8][8];
    adjoint(A, adj, 8);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<8; i++)
        for (int j=0; j<8; j++)
            inverse[i][j] = adj[i][j]/float(det);

    return true;
}

/**************************************************************************************************/

/*************************MATRIX MULTIPLICATION****************************************************/

//#define N 4

// This function multiplies
// mat1[][] and mat2[][], and
// stores the result in res[][]
void multiply(int mat1[8][8],
              int mat2[8],
              int res[8])
{
    int i, j, k;
    for (i = 0; i < 8; i++)
    {
            for (k = 0; k < 8; k++)
                res[i] += mat1[i][k] *
                             mat2[k];
    }
}

/**************************************************************************************************/

void perspective_transform(int inputquad[corner_count][2], int outputquad[corner_count][2])
{
  int U[8] = {0};
  int C[8] = {0}; //trans mat
  int X[8][8] = {0};

  // constructing the matrices
  for(int i = 0; i<corner_count; i++)
  U[i] = outputquad[i][0];
  for(int i = 0; i<corner_count; i++)
  U[i+4] = outputquad[i][1];

  for(int i = 0; i<corner_count; i++)
  {
    X[i][0] = inputquad[i][0];
    X[i][1] = inputquad[i][1];

    X[i+4][3] = inputquad[i][0];
    X[i+4][4] = inputquad[i][1];
  }

  for(int i = 0; i<corner_count; i++)
  {
    X[i][2] = 1;
    X[i+4][5] = 1;
  }

  for(int i = 0; i<corner_count; i++)
  {
    X[i][6] = -X[i][0]*U[i];
    X[i][7] = -X[i][1]*U[i];

    X[i+4][6] = -X[i+4][3]*U[i+4];
    X[i+4][7] = -X[i+4][4]*U[i+4];
  }

  //finding inverse of X matrix

  float X_inverse[8][8] = {0};

  if(!inverse(X, X_inverse))
  cout<<"Error";

  multiply(X_inverse, U, C);

  //construct transmat from C

  float transmat[3][3] = {0};

    transmat[0][0] = C[0];
    transmat[0][1] = C[1];
    transmat[0][2] = C[2];
    transmat[1][0] = C[3];
    transmat[1][1] = C[4];
    transmat[1][2] = C[5];
    transmat[2][0] = C[6];
    transmat[2][1] = C[7];
    transmat[2][2] = 1;

}
