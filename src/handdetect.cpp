//============================================================================
// Name : opencv_handdetect.cpp
// Author : andol li, andol@andol.info
// Version : 0.1
// Copyright : 2012
// Description : using haartraining results to detect the hand gesture of FIST in video stream.
//
//============================================================================
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "OpenNI2Grabber.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <unistd.h> 

using namespace cv;
using namespace std;

const double scale = 1.1;

//1.0 api version
CvMemStorage* storage = 0;
CascadeClassifier cascade;
void detectAndDraw(Mat input_image);
const char* cascade_name = "fist.xml";

//define the path to cascade file
string cascadeName = "fist.xml"; /*ROBUST-fist detection haartraining file*/
int count_position[4]= {0,0,0,0};
int old_position[4]= {0,0,0,0}; 
int oldpoint[2] = {0,0};
int newpoint[2] = {0,0};
int difference[2] = {0,0};
int diff_threshold_tb = 80; 
int diff_threshold_rl = 120;


int main(int argc, char** argv)
{
	OpenNI2Grabber grabber;
	bool isRunning = false;
	const int timeoutMs = 1000;
	//1.0 api version
	Mat frame, frame_copy, image_size;
	vector<Rect> faces;
	vector< vector<Point> > contours;
	cascade.load(cascade_name);


	if( cascade.empty() ){
		fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
		return -1;
	}

	// attempt to start the grabber
	if(grabber.initialize())
	{
		if(grabber.start())
		{
	   		isRunning = grabber.isRunning();
		}
	}
	else
	{
		std::cout << "Unable to initialize OpenNI2Grabber, program terminating!" << std::endl;
		return 0;
	}

	// acquire frames until program termination
	std::cout << "Press 'q' to halt acquisition..." << std::endl;
	Mat depthImage, colorImage;
	Mat depthImageDraw;
	Mat depthImage8U;
	
	int flag = -1;
	ofstream myfile;
  	myfile.open ("direction.txt");

	while(isRunning)
   	{
        // wait for a new image frame
        if(grabber.waitForFrame(timeoutMs))
        {
            // update the display for both frames
            if(grabber.getDepthFrame(depthImage) && grabber.getColorFrame(colorImage))
            {
	    	depthImage.convertTo(depthImageDraw, -1, 32);

		depthImageDraw.convertTo(depthImageDraw, CV_8UC1, 1.0/256.0);
		imshow("8", depthImageDraw);	
		threshold(depthImageDraw,depthImageDraw,190, 255,4);	
		medianBlur(depthImageDraw,depthImageDraw,5);
		threshold(depthImageDraw,depthImageDraw,95, 255,3);
		imshow("trun2", depthImageDraw);	
		findContours(depthImageDraw,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		drawContours(colorImage,contours,-1,Scalar(0,0,255),2);
		  /// Get the moments
		  vector<Moments> mu(contours.size() );
		  vector<Point2f> mc( contours.size() );
		  vector<float> oldX( contours.size() );
		  vector<float> oldY( contours.size() );
		  for( int i = 0; i < contours.size(); i++ )
		     {
			if(contourArea(contours[i])<5500 && contourArea(contours[i])>1000){
				mu[i] = moments( contours[i], false ); 
				mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
				circle( colorImage, mc[i], 4, Scalar(0,255,0), -1, 8, 0 );
				
				newpoint[1] = mc[i].x;
				newpoint[2] = mc[i].y;
				
				difference[1] = newpoint[1]-oldpoint[1];
				difference[2] = newpoint[2]-oldpoint[2];

				if (difference[1] < -diff_threshold_rl)
				{
					cout << "RIGHT TO LEFT RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR \n";
				}
				else if (difference[1] > diff_threshold_rl) 
				{
					cout << "LEFT TO RIGHT LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n";

				}
				else if (difference[2] < -diff_threshold_tb) 
				{
					cout << "TOP TO BOTTOM TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT\n";

				}
				else if (difference[2] > diff_threshold_tb) 
				{
					cout << "BOTTOM TO TOP BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n";

				}
			
				oldpoint[1] = newpoint[1];
				oldpoint[2] = newpoint[2];				

		
			}
						
			
		}	
	
	imshow("rgb", colorImage);
            }
        }
        else
        {
            std::cout << "No new frames received in " << timeoutMs << " ms..." << std::endl;
        }

        // check for program termination
        char key = (char) cv::waitKey(1);
        if(key == 'q' || key == 'Q')
        {
            std::cout << "Terminating program..." << std::endl;
            isRunning = false;
        }
	// check for Blob 
        char key2 = (char) cv::waitKey(1);
        if(key2 == 'b' || key2 == 'B')
        {
            std::cout << "Checking for blobs" << std::endl;
            
        }
    }
	 // stop the acquisition
   	 grabber.stop();
	myfile.close();
	return 0;
}
