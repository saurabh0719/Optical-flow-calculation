/*
Applies a perspective transformation to an image.

The function warpPerspective transforms the source image using the specified matrix:

𝚍𝚜𝚝(x,y)=𝚜𝚛𝚌(M11x+M12y+M13M31x+M32y+M33,M21x+M22y+M23M31x+M32y+M33)

*/

//uses 3*3 transmat from getPerspectiveTransform

#define W 854
#define H 480

unsigned char transframe[H][W];

void warp_perspective(unsigned char next[H][W], unsigned char transframe[H][W], int transmat[][3])
{
  int x,y;

  for(int i =0; i<W; i++)
  for(int j =0; j<H; j++)
  {
    x = (transmat[0][0]*i + transmat[0][1]*j + transmat[0][2])/(transmat[2][0]*i + transmat[2][1]*j + transmat[2][2]);
    y = (transmat[1][0]*i + transmat[1][1]*j + transmat[1][2])/(transmat[2][0]*i + transmat[2][1]*j + transmat[2][2]);
    transframe[i][j] = (unsigned char)(next[x][y]);
  }
}
