///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

#define WIDTH	352
#define HEIGHT	224
#define FPS		20
RNG rng(12345);
int p_x,p_y;
string name;

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

string numtostr(int a)
{
	stringstream ss;
	ss<<a;
	string str = ss.str();
	return str;
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

struct box_params
{
	int n;
	Rect r;
	Vec3f v;
};

Vec3b HSV2RGB(float hue, float sat, float val)
{
	float x, y, z;

	if(hue == 1) hue = 0;
	else         hue *= 6;

	int i = static_cast<int>(floorf(hue));
	float f = hue - i;
	float p = val * (1 - sat);
	float q = val * (1 - (sat * f));
	float t = val * (1 - (sat * (1 - f)));

	switch(i)
	{
		case 0: x = val; y = t; z = p; break;
		case 1: x = q; y = val; z = p; break;
		case 2: x = p; y = val; z = t; break;
		case 3: x = p; y = q; z = val; break;
		case 4: x = t; y = p; z = val; break;
		case 5: x = val; y = p; z = q; break;
	}
	return Vec3b((uchar)(x * 255), (uchar)(y * 255), (uchar)(z * 255));
}
Vector <Rect> final_bound;

int main(int argc, char* argv[])
{
	// Build color lookup table for depth display
	bool ctrl_flag=0;
	box_params box_obj;
	Mat colorLut = Mat(cv::Size(256, 1), CV_8UC3);
	ofstream file;
	fstream mut,cl;
	cvNamedWindow("Left");
	char LED[5] = "LED", GAIN[5] = "GAIN", EXPOSURE[10] = "EXPOSURE";
	int led_slider,gain_slider,exposure_slider;
	createTrackbar( LED, "Left", &led_slider, 100, on_trackbar_led );
	createTrackbar( GAIN, "Left", &gain_slider, 100, on_trackbar_gain );
	createTrackbar( EXPOSURE, "Left", &exposure_slider, 100, on_trackbar_exposure );
	for(int i = 0; i < 256; i++)
		colorLut.at<Vec3b>(i) = (i==0) ? Vec3b(0, 0, 0) : HSV2RGB(i/256.0f, 1, 1);

	// Open DUO camera and start capturing
	if(!OpenDUOCamera(WIDTH, HEIGHT, FPS))
	{
		printf("Could not open DUO camera\n");
		return 1;
	}

	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());
	printf("Dense3D Version:      v%s\n", Dense3DGetLibVersion());

	// Open Dense3D
	Dense3DInstance dense3d;
	if(!Dense3DOpen(&dense3d))
	{
		printf("Could not open Dense3D library\n");
		// Close DUO camera
		CloseDUOCamera();
		return 1;
	}
	// Set the Dense3D license (visit https://duo3d.com/account)
	if(!SetDense3DLicense(dense3d, "18NA2-6VB7H-LJ6ZJ-UBM8N-HK2L8")) // <-- Put your Dense3D license
	{
		printf("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account\n");
		// Close DUO camera
		CloseDUOCamera();
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}
	// Set the image size
	if(!SetDense3DImageSize(dense3d, WIDTH, HEIGHT))
	{
		printf("Invalid image size\n");
		// Close DUO camera
		CloseDUOCamera();
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}
	// Get DUO calibration intrinsics and extrinsics
	DUO_STEREO params;
	if(!GetCameraStereoParameters(&params))
	{
		printf("Could not get DUO camera calibration data\n");
		// Close DUO camera
		CloseDUOCamera();
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}
	// Set Dense3D parameters
	SetDense3DScale(dense3d, 0);
	SetDense3DMode(dense3d, 0);
	SetDense3DCalibration(dense3d, &params);
	SetDense3DNumDisparities(dense3d, 2);
	SetDense3DSADWindowSize(dense3d, 10);
	SetDense3DPreFilterCap(dense3d, 63);
	SetDense3DUniquenessRatio(dense3d, 1);
	SetDense3DSpeckleWindowSize(dense3d, 256);
	SetDense3DSpeckleRange(dense3d, 32);
	// Set exposure, LED brightness and camera orientation
	SetExposure(100);
	SetGain(50);
	SetLed(1);
	SetVFlip(false);
	// Enable retrieval of undistorted (rectified) frames
	SetUndistort(true);
	// Create Mat for disparity and depth map
	Mat1f disparity = Mat(Size(WIDTH, HEIGHT), CV_32FC1);
	Mat3f depth3d = Mat(Size(WIDTH, HEIGHT), CV_32FC3);
	// Run capture loop until <Esc> key is pressed
	vector<int>  coordinates;
	while((cvWaitKey(1) & 0xff) != 27)
	{
		// Capture DUO frame
		on_trackbar_exposure(exposure_slider,0);
		on_trackbar_gain(gain_slider,0);
		on_trackbar_led(led_slider,0);
		//system("exec rm -r ./objects/*");
		Mat display;
		int m,temp_n;
		mut.open("mutex.txt");
		mut>>m;
		mut.close();
		//cout<<"Checking mutex\n";
		PDUOFrame pFrameData = GetDUOFrame();
		if(pFrameData == NULL) continue;
		// Create Mat for left & right frames
		Mat left = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->leftData);
		Mat right = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->rightData);
		Mat edge;
		left.copyTo(display);
		//if(ctrl_flag==0)
		if(ctrl_flag==0 && m==1)
		{

		// Process Dense3D depth map here
		if(Dense3DGetDepth(dense3d, pFrameData->leftData, pFrameData->rightData, 
						  (float*)disparity.data, (PDense3DDepth)depth3d.data))
		{
			uint32_t disparities;
			GetDense3DNumDisparities(dense3d, &disparities);
			Mat disp8;
			disparity.convertTo(disp8, CV_8UC1, 255.0/(disparities*16));
			Mat mRGBDepth;
			cvtColor(disp8, mRGBDepth, COLOR_GRAY2BGR);
			LUT(mRGBDepth, colorLut, mRGBDepth);
			imshow("Dense3D Disparity Map", mRGBDepth);
			imshow("Left",left);
			//hi.x=p_x;
			//hi.y=p_y;
			//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//rectangle( left, hi.tl(), hi.br(), color, 2, 8, 0 );
			//Vec3f val = depth3d(pos_x,pos_y);			
			//cout<<val[0]<<" "<<val[1]<<" "<<val[2]<<endl;
			//cout<<val[2]<<endl;
		}
		// Display images
		//Finding Bounding Boxes:
		

		left.copyTo(edge);
		//fastNlMeansDenoising(edge,edge,3,7,21);
		medianBlur(edge,edge, 3);
		//Dilation(edge,edge,1);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		vector<int> area,final_area;
		file.open("coordinates.txt");

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
		   drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		}
		//cout<<endl;
		for(int i = 0;i< boundRect.size();i++)
		{
			if(((area[i]/max_area)>0.2 || area[i]>800) && area[i]<3000)
			{
				final_bound.push_back(boundRect[i]);
				final_area.push_back(area[i]);
			}	
		}
		//Histogram Equalization
		Mat hist_mat;
		//left.copyTo(hist_mat);
		//equalizeHist(hist_mat,hist_mat);
		
		for(int i=0;i<final_bound.size();i++)
		{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//cout<<final_area[i]<<" ";
			rectangle( drawing, final_bound[i].tl(), final_bound[i].br(), color, 2, 8, 0 );
			rectangle( display, final_bound[i].tl(), final_bound[i].br(), color, 2, 8, 0 );
			name = "./objects/" + numtostr(i) + ".jpg";
			box_obj.n=i;
			box_obj.r=final_bound[i];
			box_obj.v= depth3d((final_bound[i].x+final_bound[i].width)/2,(final_bound[i].y+final_bound[i].height)/2);	
			file<<box_obj.n<<" "<<(box_obj.r.x+box_obj.r.width)/2<<" "<<(box_obj.r.y+box_obj.r.height)/2<<" "<<box_obj.v[0]<<" "<<box_obj.v[1]<<" "<<box_obj.v[2]<<endl;		
			hist_mat = left(final_bound[i]);
			equalizeHist(hist_mat,hist_mat);
			//imwrite(name,left(final_bound[i]));
			imwrite(name,hist_mat);
			ctrl_flag = 1;
		}
		cout<<endl;
		//cout<<endl;
		//cout<<"___________________________________\n";


		file.close();
		mut.open("mutex.txt");
		mut.seekg(0, ios::beg);
		mut<<0;
		//imshow("Right Image", right);

		imshow("Contors",drawing);
		imshow("Edge",edge);

	}
	else if(ctrl_flag==1 && m==1)
	//else if(ctrl_flag==1)
	{
		Vector <int> c;
		cl.open("c.txt");


		while(!cl.eof())
		{
			cl>>temp_n;
			c.push_back(temp_n);
		}
		cl.close();
		for(int i=0;i<final_bound.size();i++)
		{
			string text;
			if(c[i]==0)
				text = "BALL";
			else if(c[i]==1)
				text = "GLASS";
			Scalar color = Scalar(0,255,0);
			putText(display,text,final_bound[i].tl(),0,0.5,color,2,8,0); 
			cout<<"Depth to "<<text<<": "<<box_obj.v[2]<<"mm | ";
		}
		ctrl_flag = 0;
		final_bound.clear();
		imshow("Display",display);
		mut.open("mutex.txt");
		mut.seekg(0, ios::beg);
		mut<<0;
	}
	//imshow("Left Image", left);

	//cout<<"Waiting for mutex\n";
			mut.close();
	}
	destroyAllWindows();
	mut.close();
	// Close Dense3D library
	Dense3DClose(dense3d);
	// Close DUO camera
	CloseDUOCamera();
	return 0;
}
