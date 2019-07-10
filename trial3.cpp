
#include <iostream>
#include<math.h>



using namespace std;

#define W 854				// Frame width
#define H 480				// Frame height
#define maxFeatures 4		// Maximum number of corners per frame
#define maxFeaturesH 2
#define maxFeaturesW (maxFeatures/maxFeaturesH)
#define Threshold 250000	// 5150000 for city drone video // 1250000 for beach highway drone video
#define window 9

float Ix[H][W] = {0};
float Iy[H][W] = {0};
float Ixy[H][W] = {0};
char err[]="can't find inverse";

unsigned char matFrame[H][W];
unsigned char transFrame[H][W];
unsigned char refFrame[H][W];
unsigned char diffFrame[H][W];

int  inputQuad[maxFeatures][2];
int outputQuad[maxFeatures][2];




typedef struct FEATURES
{
	int x;
	int y;
	long value;
}FEATURES;


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

			if(corners[featureIdx].value < minEigenval)
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

			int frame_count = 0;
		while(1)
    {

    	// Read a frame from the input pipe into matFrame
        count = fread(refFrame, 1, H*W, pipein);

    	// If we didn't get a frame of video, we're probably at the end
        if (count != H*W)
       		break;

					++frame_count;

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

					int element;
					int flag =0;
					int flag_check = 0;
					for(int k =0; k<4; k++)
					{
						element = (int)refFrame[inputQuad[k][0]-1][inputQuad[k][1]-1];
					for(int i = -3; i<4 && flag ==0; i++)
					for(int j =-3; j<4 && flag==0; j++)
					{
						if(element != (int)refFrame[inputQuad[k][0]+i][inputQuad[k][1]+j])
						{
							flag_check+=1;
							flag=1;
						}
					}
          flag =0;
					}
					cout<<"processing frame number "<<frame_count<<endl;
					if(flag_check==4)
					cout<<"frame number "<<frame_count<<" eligible"<<endl;

					flag_check =0;

				}


		// Feature Matching between refFrame (previous frame) and matFrame (current frame)
     	// get transformation matrix
			//transfMat = getPerspectiveTransform( outputQuad, inputQuad );
		//	getPerspectiveTransform( inputQuad, outputQuad, transfMat);
			// transform matFrame (current frame) into transFrame (transformed frame)
			//warpPerspective(matFrame, transFrame, transfMat);

			// compute difference between refFrame (previous frame) and transFrame (transformed frame)
			//computeDiff(refFrame, transFrame);

			// Write difference frame to the output pipe
			//if (diffFrame.isContinuous())
			//	fwrite(diffFrame, 1, H*W, pipeout);
        	//else
        		//printf("\nWrite to disk failed !!!\n");
	  // Flush and close input and output pipes
    fflush(pipein);
    pclose(pipein);
    fflush(pipeout);
    pclose(pipeout);

    return 0;
}
