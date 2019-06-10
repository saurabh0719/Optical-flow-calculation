//to get the 3*3 transformation matrix

/*  Calculates a perspective transform from four pairs of the corresponding points.

The function calculates the 3×3 matrix of a perspective transform so that:

⎡⎣⎢⎢⎢tix′itiy′iti⎤⎦⎥⎥⎥=𝚖𝚊𝚙\_𝚖𝚊𝚝𝚛𝚒𝚡⋅⎡⎣⎢⎢xiyi1⎤⎦⎥⎥

where

dst(i)=(x′i,y′i),src(i)=(xi,yi),i=0,1,2,3 */

//top-left , top right, bottom-right, bottom-left

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

}
