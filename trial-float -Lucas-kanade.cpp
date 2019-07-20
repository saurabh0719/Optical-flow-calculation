
#include <iostream>
#include<math.h>



using namespace std;

#define W 854				// Frame width
#define H 480				// Frame height
#define maxFeatures 4		// Maximum number of corners per frame
#define maxFeaturesH 2
#define maxFeaturesW (maxFeatures/maxFeaturesH)
#define Threshold 1250000	// 5150000 for city drone video // 1250000 for beach highway drone video
#define window 9

#define vl 0.0832 //drone video length 0.791s. (0.791/19frames)*2 = 0.0832

float Ix[H][W] = {0};
float Iy[H][W] = {0};
float Ixy[H][W] = {0};
char err[]="can't find inverse";

unsigned char matFrame[H][W];
unsigned char transFrame[H][W];
unsigned char refFrame[H][W];
unsigned char diffFrame[H][W];

int  inputQuad[maxFeatures][2];
float outputQuad[maxFeatures][2];





/*******************SOBEL FILTER****************************************************************/

int sobel_x[3][3]={{-1,-2,-1},{0,0,0},{1,2,1}};
int sobel_y[3][3]={{-1,0,1},{-2,0,2},{-1,0,1}};

/**************************************************************************************************/

/********************TRANSPOSE OF A MATRIX*********************************************************/

// This function stores transpose of A[][] in B[][]
void transpose(int A[][2],int B[][window])
{
    int i, j;
    for (i = 0; i < 2; i++)
        for (j = 0; j < window; j++)
            B[i][j] = A[j][i];
}

/**************************************************************************************************/

/**************INVERSE OF A MATRIX*****************************************************************/

// Function to get cofactor of A[p][q] in temp[][]. n is current
// dimension of A[][]
void getCofactor(int A[][2],int temp[][2],int p,int q,int n)
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
void adjoint(int A[][2],int adj[][2], int N)
{
    if (N == 1)
    {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store cofactors of A[][]
    int sign = 1, temp[2][2]; //changed from N to 2

    for (int i=0; i<N; i++)
    {
        for (int j=0; j<N; j++)
        {
            // Get cofactor of A[i][j]
            getCofactor(A,temp,i,j,N);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant(temp,N-1));
        }
    }
}

// Function to calculate and store inverse, returns false if
// matrix is singular
bool inverse(int A[][2], float inverse[][2])
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
    adjoint(A,adj,2);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<2; i++)
        for (int j=0; j<2; j++)
            inverse[i][j] = adj[i][j]/(float)(det);

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
              int res[][2])
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


void multiply_2(float mat1[][2],
              int mat2[][window],
              float res[][window])
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


void multiply_final(float mat1[][window],
              int mat2[window],
              float res[2][1])
{
    int i, j, k;
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 1; j++)
        {
            res[i][j] = 0;
            for (k = 0; k < window; k++)
                res[i][j] += (mat1[i][k] *
                             mat2[k]);
        }
    }
}

/**************************************************************************************************/


// pass int status[maxFeatures] = {0};
//pass the matrices for both refs;

void opticalFlow(unsigned char ref[H][W], unsigned char next[H][W], int inputquad[maxFeatures][2], float outputquad[maxFeatures][2], int status[maxFeatures])
{

    int Ix[window] ={0};
    int Iy[window] ={0};
    int It[window] ={0}; // also the t vector of the equation
    //store the negative values directly in this ( -It)


    int i =0;
    int x,y;

    int pixel_x,pixel_y;
    int pixel_t;
    while(i< maxFeatures)
    {

      // (u v)t = (StS)^-1 * St. tvector
      //calculate Ix[9] Iy[9] and It[9] for each corner

/*******************CALCULATIONS********************************/

//calculation of Ix Iy and It (using sobel filter)
	x=inputquad[i][0];
	y=inputquad[i][1];
	 int k=0;

	for(int p=x-1;p<=x+1;p++)
	{
		for(int q=y-1;q<=y+1;q++)
		{
            pixel_x = (sobel_x[0][0] * ref[p-1][q-1])
                    + (sobel_x[0][1] * ref[p-1][q])
                    + (sobel_x[0][2] * ref[p-1][q+1])
                    + (sobel_x[1][0] * ref[p][q-1])
                    + (sobel_x[1][1] * ref[p][q])
                    + (sobel_x[1][2] * ref[p][q+1])
                    + (sobel_x[2][0] * ref[p+1][q-1])
                    + (sobel_x[2][1] * ref[p+1][q])
                    + (sobel_x[2][2] * ref[p+1][q+1]);

            pixel_y = (sobel_y[0][0] * ref[p-1][q-1])
                    + (sobel_y[0][1] * ref[p-1][q])
                    + (sobel_y[0][2] * ref[p-1][q+1])
                    + (sobel_y[1][0] * ref[p][q-1])
                    + (sobel_y[1][1] * ref[p][q])
                    + (sobel_y[1][2] * ref[p][q+1])
                    + (sobel_y[2][0] * ref[p+1][q-1])
                    + (sobel_y[2][1] * ref[p+1][q])
                    + (sobel_y[2][2] * ref[p+1][q+1]);


            Ix[k]=pixel_x;
            Iy[k]=pixel_y;
            k++;
            }
       }
        k=0;
        for(int p=x-1;p<=x+1;p++)
        {
        	for(int q=y-1;q<=y+1;q++)
        	{
       			It[k]=-(next[p][q]-ref[p][q]);
       			k++;
        	}
        }

      //least squares method

      int S[window][2];

      for(int i =0; i<window; i++)
      S[i][0] = Ix[i];

      for(int i =0; i<window; i++)
      S[i][1] = Iy[i];

      int ST[2][window]; // S transpose

      transpose(S,ST);

      int STS[2][2]={0};// stores S transpose * S

      multiply_1(ST,S,STS);

      float STS_inverse[2][2]={0};

      if(!inverse(STS,STS_inverse))
      {
       	cout<<err;//storing type of error for help in debugging
        status[i]=0;//set status flag to 0
        return ;
      }

      float S_final[2][window]={0};

      multiply_2(STS_inverse,ST, S_final);

      float res[2][1]={0};

      multiply_final(S_final, It, res);

/*********************************************************************/

      outputquad[i][0] = res[0][0]; //storing final values of the corner in output quad
      outputquad[i][1] = res[1][0];

      status[i] = 1; //set status flag to 1
      i++;

    }


}

typedef struct FEATURES
{
	int x;
	int y;
	long value;
}FEATURES;

/************************************************************************************************************/



void getCofactor3(int A[][7],int temp[][7],int p,int q,int n)
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

int determinant3(int A[][7], int n)
{
    int D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1)
        return A[0][0];

    int temp[7][7]; // To store cofactors

    int sign = 1;  // To store sign multiplier

     // Iterate for each element of first row
    for (int f = 0; f < n; f++)
    {
        // Getting Cofactor of A[0][f]
        getCofactor3(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant3(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

bool check_invertibility(int x, int y, unsigned char Frame[H][W])
{
  int arr[7][7];
  for(int i =-3; i<4; i++)
  {
    for(int j =-3; j<4; j++)
    {
      arr[i+3][j+3] = (int)Frame[x+i][y+j];
    }
  }

  int res = determinant3(arr,7);

  if(res == 0)
  return false;
  else return true;
}

void shi_tomasi(FEATURES corners[maxFeatures], unsigned char Frame[H][W])
{
	int y, x, k, m, u, v, ymin, ymax, xmin, xmax, Rcount=0, i, featureIdx, tempIdx, flag;

  	long sumX, sumY;

	long M[4], detM, traceM;
	long eigenval[2], minEigenval;

	int blockSizeByTwo = 1;
	int SobelFilterSize = 3;


	// dx (sobel filter) is created such that it is already rotated by 180 for convolution
	int dx[7][7] = {{130,120,78,0,-78,-120,-130},
 				   {180,195,156,0,-156,-195,-180},
				   {234,312,390,0,-390,-312,-234},
 				   {260,390,780,0,-780,-390,-260},
				   {234,312,390,0,-390,-312,-234},
				   {180,195,156,0,-156,-195,-180},
   					{130,120,78,0,-78,-120,-130}};

	// dy (sobel filter) transpose of dx (also already rotated by 180 for convolution)
	int dy[7][7] = {{130,180,234,260,234,180,130},
   					{120,195,312,390,312,195,120},
    				 {78,156,390,780,390,156,78},
		  				   {0,0,0,0,0,0,0},
 				 {-78,-156,-390,-780,-390,-156,-78},
				{-120,-195,-312,-390,-312,-195,-120},
				{-130,-180,-234,-260,-234,-180,-130}};

	for (i=0; i<maxFeatures; i++)
	{
		corners[i].x=0;
		corners[i].y=0;
		corners[i].value = Threshold;
	}

	// Apply sobel filter and compute X^2, Y^2 and X*Y
	for (y=SobelFilterSize ; y<H-SobelFilterSize ; ++y)
		for (x=SobelFilterSize ; x<W-SobelFilterSize ; ++x)
		{
			sumX=0;
			sumY=0;
			for (k=-SobelFilterSize; k<=SobelFilterSize ; ++k)
				for (m=-SobelFilterSize; m<=SobelFilterSize ; ++m)
				{
					sumX += ( Frame[y+k][x+m] * dx[k+SobelFilterSize][m+SobelFilterSize] )/255;
					sumY += ( Frame[y+k][x+m] * dy[k+SobelFilterSize][m+SobelFilterSize] )/255;
				}

			Ix[y][x] = sumX*sumX;
			Iy[y][x] = sumY*sumY;
			Ixy[y][x] = sumX*sumY;
		}

	// Find distinct corners

	for (y=SobelFilterSize+H/5 ; y<H-SobelFilterSize ; ++y)
	{
		for (x=SobelFilterSize ; x<W-SobelFilterSize ; ++x)
		{
			M[0] = M[1] = M[2] = M[3] = 0;

			for(v=y-blockSizeByTwo ; v<=y+blockSizeByTwo ; v++)
		   		for(u=x-blockSizeByTwo; u<=x+blockSizeByTwo; u++)
		   		{
		  			M[0] += Ix[v][u];
		   			M[1] += Ixy[v][u];
		   			M[3] += Iy[v][u];
		   		}
		   	M[2] = M[1];

			traceM = M[0] + M[3];
			detM = M[0]*M[3] - M[1]*M[2];

			eigenval[0] = traceM/2 + sqrt( (traceM*traceM)/4 - detM );
			eigenval[1] = traceM/2 - sqrt( (traceM*traceM)/4 - detM );

			featureIdx = ( (y) / (H/maxFeaturesH) ) * maxFeaturesW + ( x / (W/maxFeaturesW));

			minEigenval = min(eigenval[0],eigenval[1]);

			if(corners[featureIdx].value < minEigenval && check_invertibility(x,y,Frame))
			{

				corners[featureIdx].x = x;
				corners[featureIdx].y = y;
				corners[featureIdx].value = minEigenval;

			}
		}
	}


}


int main()
{
    int x, y, count, count1=0;

   	int transfMat[3][3]={0};

   	int status[maxFeatures];



   	FEATURES corners[maxFeatures] = {0};
   	FEATURES cornersRef[maxFeatures] = {0};

	// Open an input pipe from ffmpeg and an output pipe to a second instance of ffmpeg
    FILE *pipein = popen("ffmpeg -i DroneBeach3.mp4 -loglevel panic -f image2pipe -vcodec rawvideo -pix_fmt gray -", "r");
    FILE *pipeout = popen("ffmpeg -y -f rawvideo -loglevel panic -vcodec rawvideo -pix_fmt gray -s 854x480 -r 24 -i - -f mp4 -q:v 5 -an -vcodec mpeg4 prototype_1.mp4", "w");

    // Read 1st frame
    count = fread(refFrame, 1, H*W, pipein);

	// Feature detection (Shi-tomasi corner detection) for 1st Frame
	shi_tomasi(cornersRef, refFrame);

	// Save detected corners in inputQuad variable for 1st Frame
    inputQuad[0][0] =  cornersRef[0].x;
    inputQuad[1][0] =  cornersRef[1].x;
    inputQuad[2][0] =  cornersRef[2].x;
    inputQuad[3][0] =  cornersRef[3].x;
    inputQuad[0][1] =  cornersRef[0].y;
    inputQuad[1][1] =  cornersRef[1].y;
    inputQuad[2][1] =  cornersRef[2].y;
    inputQuad[3][1] =  cornersRef[3].y;

    // Process video frames
    while(1)
    {
    	// Read a frame from the input pipe into matFrame
        count = fread(matFrame, 1, H*W, pipein);

    	// If we didn't get a frame of video, we're probably at the end
        if (count != H*W)
       		break;

       	count1++;

		// Feature Matching between refFrame (previous frame) and matFrame (current frame)
     	opticalFlow(refFrame, matFrame, inputQuad, outputQuad, status);

		// Once above function is executed successfully, inputQuad and outpuQuad will have coordinates of corresponding features

        if(status[0]==1 && status[1]==1 && status[2]==1 && status[3]==1 ) // If there is no error in feature matching
        {
					cout<<"frame number "<<count1<<endl;
					cout<<"Input quad "<<endl;
					for(int i =0; i<4; i++)
					cout<<inputQuad[i][0]<<" "<<inputQuad[i][1]<<endl;
				  cout<<"Output quad "<<endl;
			    for(int i =0; i<4; i++)
					cout<<outputQuad[i][0]<<" "<<outputQuad[i][1]<<endl;
          cout<<"result quad "<<endl;
			    for(int i =0; i<4; i++)
					cout<<(inputQuad[i][0] + outputQuad[i][0]*vl)<<" "<<(inputQuad[i][1] + outputQuad[i][1]*vl)<<endl;

		}
		else
		{
			cout << "Frame not processed: " << count1 << endl;
		}

		// copy unprocessed current frame from matFrame to refFrame
		for( x=0;x<H;x++)
		{
			for( y=0;y<W;y++)
			{
				refFrame[x][y]=matFrame[x][y];
			}
		}//refFrame = matFrame.clone();

		// Feature detection (Shi-tomasi corner detection)
		shi_tomasi(cornersRef, refFrame);

		// Save detected corners in inputQuad
    inputQuad[0][0] =  cornersRef[0].x;
    inputQuad[1][0] =  cornersRef[1].x;
    inputQuad[2][0] =  cornersRef[2].x;
    inputQuad[3][0] =  cornersRef[3].x;
    inputQuad[0][1] =  cornersRef[0].y;
    inputQuad[1][1] =  cornersRef[1].y;
    inputQuad[2][1] =  cornersRef[2].y;
    inputQuad[3][1] =  cornersRef[3].y;

    	if(count1==20)
    		break;
    }

    printf("Image registration and frame differencing completed !!!\n");

    // Flush and close input and output pipes
    fflush(pipein);
    pclose(pipein);
    fflush(pipeout);
    pclose(pipeout);

    return 0;
}
