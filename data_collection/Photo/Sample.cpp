
///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"
#include <sstream>
#include <string>

#define WIDTH	320
#define HEIGHT	240
#define FPS		30
RNG rng(12345);
using namespace std;
Vector <Rect> final_bound;

string numtostr(int a)
{
	stringstream ss;
	ss<<a;
	string str = ss.str();
	return str;
}

void on_trackbar_gain( int a, void* )
{
	SetGain(a);
}
void on_trackbar_exposure( int a, void* )
{
	SetExposure(a);
}
void on_trackbar_led( int a, void* )
{
	SetLed(a);
}

void Erosion(Mat src, Mat erosion_dst,int erosion_size)
{
  int erosion_elem = 0;
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  //erode( src, erosion_dst, element );
}

/** @function Dilation */
void Dilation(Mat src, Mat dilation_dst,int dilation_size)
{
  int dilation_elem = 2;
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( src, dilation_dst, element );
  //imshow( "Dilation Demo", dilation_dst );
}
Mat display;
string name;
int main(int argc, char* argv[])
{
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());

	// Open DUO camera and start capturing
	if(!OpenDUOCamera(WIDTH, HEIGHT, FPS))
	{
		printf("Could not open DUO camera\n");
		return 0;
	}
	printf("\nHit <ESC> to exit.\n");
	
	// Create OpenCV windows
	cvNamedWindow("Left");
	//cvNamedWindow("Right");

	// Create image headers for left & right frames
	IplImage *left, *right;

	// Set exposure and LED brightness
	SetGain(0);
	SetExposure(50);
	SetLed(1);
	SetUndistort(true);

	Mat small_32,small_64;
	Size sm_32(32,32),sm_64(200,200);
	bool first = true;
	char LED[5] = "LED", GAIN[5] = "GAIN", EXPOSURE[10] = "EXPOSURE";
	int led_slider,gain_slider,exposure_slider;
	createTrackbar( LED, "Left", &led_slider, 100, on_trackbar_led );
	createTrackbar( GAIN, "Left", &gain_slider, 100, on_trackbar_gain );
	createTrackbar( EXPOSURE, "Left", &exposure_slider, 100, on_trackbar_exposure );
	string name_32,name_64;
	char button;
	
    int count=1; //Set count here
     
	// Run capture loop until <Esc> key is pressed
	while((cvWaitKey(1) & 0xff) != 27)
	{
		// Capture DUO frame
		on_trackbar_exposure(exposure_slider,0);
		on_trackbar_gain(gain_slider,0);
		on_trackbar_led(led_slider,0);
		PDUOFrame pFrameData = GetDUOFrame();
		if(pFrameData == NULL) continue;

		// Set the image data
		if(first)
		{
			first = false;
			left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
			right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
		}
		left->imageData = (char*)pFrameData->leftData;
		right->imageData = (char*)pFrameData->rightData;
		Mat edge;
		Mat mat_left(left);	
		mat_left.copyTo(edge);
		medianBlur(edge,edge, 3);
		//Dilation(edge,edge,1);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		vector<int> area,final_area;

		int max_area = 0;
		//blur(edge, edge, Size(4,4));
		int edgeThresh = 20;
		Canny(edge, edge, edgeThresh, edgeThresh*3, 3,1);
		Erosion(edge,edge,15);
		Dilation(edge,edge,1);
		findContours( edge, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		vector<Rect> boundRect( contours.size() );
		Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
	  for( int i = 0; i< contours.size(); i++ )
		 {
		   Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		   boundRect[i] = boundingRect( contours[i] );
		  
			int temp;

			temp = boundRect[i].height * boundRect[i].width;
			area.push_back(temp);
			//cout<<temp<<" ";
			if(temp>max_area)
				max_area = temp;
		   drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
		}
		cout<<endl;
		for(int i = 0;i< boundRect.size();i++)
		{
			if(((area[i]/max_area)>0.2 || area[i]>1000) && area[i]<2000)
			{
				final_bound.push_back(boundRect[i]);
				final_area.push_back(area[i]);
			}	
		}
		//Histogram Equalization
		Mat hist_mat,norm_mat;
		string name1;
		char button = waitKey(30);
		Size sz(32,32);
		rectangle( drawing, final_bound[0].tl(), final_bound[0].br(), Scalar(255,255,255), 2, 8, 0 );
			if(button == ' ')
			{
			count++;
			
			//rectangle( display, final_bound[i].tl(), final_bound[i].br(), color, 2, 8, 0 );
			name = "./norm_" + numtostr(count) + ".jpg";	//saves the normal image
			norm_mat = mat_left(final_bound[0]);
			resize(norm_mat,norm_mat,sz);
			imwrite(name,norm_mat);
			equalizeHist(norm_mat,hist_mat);
			name1= "./hist_" + numtostr(count) + ".jpg";   //saves the histogram equalized image
			resize(hist_mat,hist_mat,sz);
			imwrite(name1,hist_mat);
			
			}
		
	
		// Display images
		//cvShowImage("Left", left);
		imshow("Image",mat_left);
		imshow("Contors",drawing);
		imshow("Edge",edge);
		//cvShowImage("Right", right);
		final_bound.clear();
	}
	cvDestroyAllWindows();

	// Release image headers
	cvReleaseImageHeader(&left);
	cvReleaseImageHeader(&right);

	// Close DUO camera
	CloseDUOCamera();
	return 0;
}
