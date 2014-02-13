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
const int minBody=50000;
//1.0 api version
MemStorage* storage = 0;
void detectAndDraw(Mat input_image);
int count_position[4]= {0,0,0,0};
int old_position[4]= {0,0,0,0}; 
int oldpoint[2] = {0,0};
int newpoint[2] = {0,0};
int difference[2] = {0,0};
int diff_threshold_tb = 30; 
int diff_threshold_rl = 40;
int massThresh = 190;
int massTimeout = 31;
int powerTimeout = 0;
int numPoints = 0;
int main(int argc, char** argv)
{
	OpenNI2Grabber grabber;
	bool isRunning = false;
	const int timeoutMs = 1000;
	//1.0 api version
	Mat frame, frame_copy, image_size;
	vector<Rect> faces;
	vector< vector<Point> > contours;

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
	Mat depthImageDraw, depthBody;
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
		depthImageDraw.copyTo(frame_copy);
		depthImageDraw.convertTo(depthImageDraw, CV_8U, 1.0/256.0);
			
		threshold(depthImageDraw,depthImageDraw,200, 255,4);	//190
		medianBlur(depthImageDraw,depthImageDraw,5);
		//imshow("trun1", depthImageDraw);	
		threshold(depthImageDraw,depthImageDraw,90, 255,3);
		depthImageDraw.copyTo(depthBody);
		//imshow("trun2", depthImageDraw);
		findContours(depthImageDraw,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

		/*for( int i = 0; i < contours.size(); i++ ){	
			if (contourArea(contours[i])>40000 && contourArea(contours[i])<90000){	
				
				Moments mu = moments( contours[i], false ); 
				Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
				int mX =  mc.x;
				int mY = mc.y;
				int newDepth = static_cast<int>(frame_copy.at<uchar>(mX,mY));
				if(newDepth>100 && newDepth<220){
					massThresh = (.1 * newDepth) + .9 * massThresh;
					circle( colorImage, mc, 4, Scalar(0,255,0), -1, 8, 0 );
					ostringstream ss;
				ss << contourArea(contours[i]);
				putText(colorImage, ss.str(), mc, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
				}
		  	}
		}
		contours.clear();
		threshold(depthBody,depthBody,massThresh-16, 255,4);	//190*/
		threshold(depthBody,depthBody,170, 255,4);
		imshow("8", depthBody);
		findContours(depthBody,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		drawContours(colorImage,contours,-1,Scalar(0,0,255),2);
		vector<Moments> mu(contours.size() );
		vector<Point2f> mc( contours.size() );
		for( int i = 0; i < contours.size(); i++ ){	
			if(contourArea(contours[i])<6800 && contourArea(contours[i])>1500){
				
				mu[i] = moments( contours[i], false ); 
				mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
				circle( colorImage, mc[i], 4, Scalar(0,255,0), -1, 8, 0 );
				if(mc[i].y < 400){
				numPoints++;
				if(numPoints>1)
					break;
				newpoint[1] = mc[i].x;
				newpoint[2] = mc[i].y;
				if(massTimeout>=2){
					oldpoint[1] = newpoint[1];
					oldpoint[2] = newpoint[2];
				}
				massTimeout=0;
				difference[1] = newpoint[1]-oldpoint[1];
				difference[2] = newpoint[2]-oldpoint[2];

				if (difference[1] < -diff_threshold_rl/1.4 && difference[2] < -diff_threshold_tb/1.4)
				{
					circle( colorImage, Point(160,120), 40, Scalar(255,0,0), -1, 8, 0 );
					//system("notify-send Volume-Up");
					cout << "RIGHT TO LEFT RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR \n";
				}
				else if (difference[1] > diff_threshold_rl/1.4 && difference[2] < -diff_threshold_tb/1.4) 
				{
					circle( colorImage, Point(480,120), 40, Scalar(255,0,0), -1, 8, 0 );
					//system("notify-send Volume-Down");
					cout << "LEFT TO RIGHT LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n";

				}
				else if (difference[1] < -diff_threshold_rl/1.4 && difference[2] > diff_threshold_tb/1.4) 
				{
					circle( colorImage, Point(160,360), 40, Scalar(0,0,255), -1, 8, 0 );
					//system("notify-send Channel-Up'");
					cout << "TOP TO BOTTOM TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT\n";

				}
				else if (difference[1] > diff_threshold_rl/1.4 && difference[2] > diff_threshold_tb/1.4) 
				{
					circle( colorImage, Point(480,360), 40, Scalar(0,0,255), -1, 8, 0 );
					//system("notify-send Channel-Down");
					cout << "BOTTOM TO TOP BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n";

				}
				else if (difference[1] > diff_threshold_rl) 
				{
					circle( colorImage, Point(480,240), 40, Scalar(255,0,0), -1, 8, 0 );
					//system("notify-send Volume-Down");
					cout << "LEFT TO RIGHT LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n";

				}
				else if (difference[1] < -diff_threshold_rl) 
				{
					circle( colorImage, Point(160,240), 40, Scalar(255,0,0), -1, 8, 0 );
					//system("notify-send Volume-Down");
					cout << "LEFT TO RIGHT LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n";

				}
				else if (difference[2] > diff_threshold_tb) 
				{
					circle( colorImage, Point(320,360), 40, Scalar(255,0,0), -1, 8, 0 );
					//system("notify-send Volume-Down");
					cout << "LEFT TO RIGHT LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n";

				}
				else if (difference[2] < -diff_threshold_tb) 
				{
					circle( colorImage, Point(320,120), 40, Scalar(255,0,0), -1, 8, 0 );
					//system("notify-send Volume-Down");
					cout << "LEFT TO RIGHT LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n";

				}
				
			
				oldpoint[1] = newpoint[1];
				oldpoint[2] = newpoint[2];				
			
			
			}
		}			
			
		}	
			
	
	
	if(massTimeout<2)
		massTimeout++;
	if(numPoints>1){
		powerTimeout++;
	}else{powerTimeout=0;}
	if(powerTimeout>20){
		//putText(colorImage, "POWER OFF", Point(160,260), FONT_HERSHEY_COMPLEX, 2.0, Scalar(0,0,255), 1, CV_AA);
		rectangle( colorImage, Point( 0,0 ),  Point( 640, 480),  Scalar( 0, 0, 0 ),  -1, 8 );
		powerTimeout=21;
		}
	numPoints=0;
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
