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

#define WIDTH	368
#define HEIGHT	224
#define FPS		20

RNG rng(12345);
int p_x,p_y;

// Kalman filter variables

// State variables
double state_prev[] = {0.0, 0.0, 0.0, 0.0};
double state_next[] = {0.0, 0.0, 0.0, 0.0};

// Correlation matrix
double P_prev[] = {10.0, 10.0, 10.0, 10.0};
double P_next[] = {0.0, 0.0, 0.0, 0.0};

string name;


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == EVENT_LBUTTONDOWN )
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		p_x=x;
		p_y=y;
	}
	else if  ( event == EVENT_RBUTTONDOWN )
	{
		//cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if  ( event == EVENT_MBUTTONDOWN )
	{
		//cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
    else if ( event == EVENT_MOUSEMOVE )
    {
        //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
    }
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

string numtostr(int a)
{
	stringstream ss;
	ss<<a;
	string str = ss.str();
	return str;
}
int strtonum(string s) 
{  
    istringstream is(s);
    int n;
    is >> n;
	return n;
}

Point create_line_y(int x1, int x2, int y1, int y2, int x)
{
	Point p;
	int y;
	y = ((y2-y1)*(x-x1))/(x2-x1) + y1;
	p.x = x;
	p.y = y;
	return p;
}	
Point create_line_x(int x1, int x2, int y1, int y2, int y)
{
	Point p;
	int x;
	x = ((y - y1)*(x2-x1))/(y2-y1) + x1;
	p.x = x;
	p.y = y;
	return p;
}
bool inorout(Point pt)
{
		Point ct[4];
		Point tl,tr,bl,br; 
		tl.x = 55;
		tl.y = 172;
		tr.x = 274; 
		tr.y = 163;
		br.x = 304;
		br.y = 43;
		bl.x = 11;
		bl.y = 49;
		ct[0] = create_line_x(tr.x,br.x,tr.y,br.y,pt.y);
		ct[1] = create_line_y(tl.x,tr.x,tl.y,tr.y,pt.x);
		ct[2] = create_line_x(bl.x,tl.x,bl.y,tl.y,pt.y);
		ct[3] = create_line_y(bl.x,br.x,bl.y,br.y,pt.x);
		if(ct[1].y>pt.y && ct[3].y<pt.y && ct[0].x>pt.x && ct[2].x<pt.x) 
			return 1;
		else
			return 0;
}
struct twoval
{
	double m;
	int c;
};
twoval find_slope(double x1, double y1, double x2, double y2, int x, int y)
{
	double m;
	int c;
	twoval p;
	m = ((y2-y1)/(x2-x1));
	c = y - (m*x);
	//cout<<m<<" "<<c<<endl;
	p.m = m;
	p.c = c;
	return p;
}

Point find_intercept(twoval p1, twoval p2)
{
	int x,y;
	Point p;
	//cout<<p1<<" "<<p2<<endl;
	x = (p2.c - p1.c)/(p1.m - p2.m);
	y = (p2.m*x) + p2.c;
	p.x = x;
	p.y = y;
	return p;
}
double distance(int x1, int x2, int y1, int y2)
{
	double dist;
	dist = sqrt(pow((x2-x1),2) + pow((y2-y1),2));
	return dist;
}	

void kalman_filter(double z, int select)
{
	double A = 1;
	double B = 0;
	double u = 0;
	double Q = 0.001;
	double R = 1;
	double H = 1;
	double I = 1;
	
	double K;

	// state value estimation
	state_next[select] = A*state_prev[select] + B*u;
	P_next[select] = A*P_prev[select]*A + Q;
	
	// Kalman gain
	K = (P_next[select]*H)/(H*P_next[select]*H + R);
	
	// State value updation
	state_prev[select] = state_next[select] + K*(z -H*state_next[select]);
	P_prev[select] = (I - K*H)*P_next[select];
		
		
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
Vector <Point> final_mid;







int main(int argc, char* argv[])
{
	// Build color lookup table for depth display
	bool ctrl_flag=0;
	box_params box_obj;
	Mat colorLut = Mat(cv::Size(256, 1), CV_8UC3);
	ofstream file,kin_file;
	fstream mut,cl;
	cvNamedWindow("Control");
	char LED[5] = "LED", GAIN[5] = "GAIN", EXPOSURE[10] = "EXPOSURE";
	int led_slider = 15,gain_slider = 0 ,exposure_slider= 100;
	createTrackbar( LED, "Control", &led_slider, 100, on_trackbar_led );
	createTrackbar( GAIN, "Control", &gain_slider, 100, on_trackbar_gain );
	createTrackbar( EXPOSURE, "Control", &exposure_slider, 100, on_trackbar_exposure );
	for(int i = 0; i < 256; i++)
		colorLut.at<Vec3b>(i) = (i==0) ? Vec3b(0, 0, 0) : HSV2RGB(i/256.0f, 1, 1);
	int c_str = 0;
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
	if(!SetDense3DLicense(dense3d, "")) // <-- Put your Dense3D license
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
	SetDense3DMode(dense3d, 2);
	SetDense3DCalibration(dense3d, &params);
	SetDense3DNumDisparities(dense3d, 2);
	SetDense3DSADWindowSize(dense3d, 10);
	SetDense3DPreFilterCap(dense3d, 63);
	SetDense3DUniquenessRatio(dense3d, 1);
	SetDense3DSpeckleWindowSize(dense3d, 256);
	SetDense3DSpeckleRange(dense3d, 32);
	// Set exposure, LED brightness and camera orientation
	SetExposure(70);
	SetGain(0);
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

		char reset_button = waitKey(30);
		if(reset_button == ' ')
		{
			state_prev[0] = 0.0;
			state_prev[1] = 0.0;
			state_prev[2] = 0.0;
			state_prev[3] = 0.0;
			
			state_next[0] = 0.0;
			state_next[1] = 0.0;
			state_next[2] = 0.0;
			state_next[3] = 0.0;
			
			cout<<"RESET"<<endl;
			// Correlation matrix
			P_prev[0] = 10.0;
			P_prev[1] = 10.0;
			P_prev[2] = 10.0;
			P_prev[3] = 10.0;
			
			P_next[0] = 0.0;
			P_next[1] = 0.0;
			P_next[2] = 0.0;
			P_next[3] = 0.0;
		}
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
		Rect roi = {5, 95, 260, 112};
		//left = left(roi);
		Mat edge;
		left.copyTo(display);
		//if(ctrl_flag==0)
		if(ctrl_flag==0 && m==1)
		{
				//cout<<c_str<<endl;
		c_str++;
		// Process Dense3D depth map here
		Mat mRGBDepth;
		if(Dense3DGetDepth(dense3d, pFrameData->leftData, pFrameData->rightData, 
						  (float*)disparity.data, (PDense3DDepth)depth3d.data))
		{
			uint32_t disparities;
			GetDense3DNumDisparities(dense3d, &disparities);
			Mat disp8;
			disparity.convertTo(disp8, CV_8UC1, 255.0/(disparities*16));

			cvtColor(disp8, mRGBDepth, COLOR_GRAY2BGR);
			LUT(mRGBDepth, colorLut, mRGBDepth);

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
		vector<Point> mid;
		file.open("coordinates.txt");
		//kin_file.open("ik_file.txt");
		kin_file.open("ik_pickdrop.txt");
		int max_area = 0;
		//blur(edge, edge, Size(4,4));
		int edgeThresh = 40;
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
			Point temp_mid;
			int max_wh = max(boundRect[i].width,boundRect[i].height);
			temp_mid.x = boundRect[i].x + max_wh/2;
			temp_mid.y = 240 - (boundRect[i].y + max_wh/2);
			mid.push_back(temp_mid);
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
			bool checkin = inorout(mid[i]);
			//cout<<i<<" "<<checkin;
			//if((((area[i]/max_area)>0.2 || area[i]>300) && area[i]<3000))
			if(checkin && area[i]<3000 && area[i]>500)
			{
				final_bound.push_back(boundRect[i]);
				final_area.push_back(area[i]);
				final_mid.push_back(mid[i]);
			}	
		}
		//Histogram Equalization
		Mat hist_mat;
		//left.copyTo(hist_mat);
		//equalizeHist(hist_mat,hist_mat);
		//setMouseCallback("Dense3D Disparity Map", CallBackFunc, NULL);
		//Vec3f tc = depth3d.at<Vec3f>(p_x,p_y);
		//cout<<tc[0]<<" "<<tc[1]<<" "<<tc[2]<<endl;
		
		for(int i=0;i<final_bound.size();i++)
		{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//cout<<final_area[i]<<" ";
			rectangle( drawing, final_bound[i].tl(), final_bound[i].br(), color, 2, 8, 0 );
			rectangle( display, final_bound[i].tl(), final_bound[i].br(), color, 2, 8, 0 );
			name = "./objects/" + numtostr(i) + ".jpg";

			twoval p1,p2,p3,p4;
			Point pa,pb;
			Point center;
			center.x = final_mid[i].x;
			center.y = final_mid[i].y; 
			p1 = find_slope(245,123,259,55,center.x,center.y);
			p2 = find_slope(61,25,181,25,61,25);
			p3 = find_slope(61,25,181,25,center.x,center.y);
			p4 = find_slope(245,123,259,55,245,123);
			pa = find_intercept(p1,p2);
			pb = find_intercept(p3,p4);
			double dist1,dist2;
			int x_diff,y_diff;
			x_diff = center.x - 160;
			y_diff = center.y - 120;
			//cout<<x_diff<<" "<<y_diff<<"\n";
			//double const1=2.4876,const2=4.83;

			int box_width, box_height;
			box_width = final_bound[i].width;
			box_height = final_bound[i].height;
			//double const1=0.2572,const2=0.10318,const3=0.001458, const4 = 0.11760;
			//dist1 = (distance(pb.x,center.x,pb.y,center.y))*const1 + const2*box_width;
			//dist2 = pow(distance(pa.x,center.x,pa.y,center.y),2)*const3 + const4*box_height;
			double const1 = 0.1024, const2 = 1.15329;
			double const3 = 0.22287, const4 = 0.30521;
			dist2 = const1*(pow(distance(pa.x,center.x,pa.y,center.y),const2));
			dist1 = const3*(distance(pb.x,center.x,pb.y,center.y)) + const4*dist2;
			kalman_filter(dist1, 0);
			kalman_filter(dist2, 1);
			kalman_filter(box_width, 2);
			kalman_filter(box_height, 3);

			box_obj.n=i;
			box_obj.r=final_bound[i];
			box_obj.v=depth3d.at<Vec3f>(Point(final_bound[i].x+final_bound[i].width/2,final_bound[i].y+final_bound[i].height/2));

			file<<box_obj.n<<" "<<box_obj.r.x+box_obj.r.width/2<<" "<<box_obj.r.y+box_obj.r.height/2<<" "<<box_obj.v[0]<<" "<<box_obj.v[1]<<" "<<box_obj.v[2]<<endl;		
			//putText(mRGBDepth,"HI",center,0,0.5,color,2,8,0); 
			hist_mat = left(final_bound[i]);
			equalizeHist(hist_mat,hist_mat);
			//imwrite(name,left(final_bound[i]));
			imwrite(name,hist_mat);
			ctrl_flag = 1;
		}
		//cout<<endl;
		//cout<<endl;
		//cout<<"___________________________________\n";

		imshow("Dense3D Disparity Map", mRGBDepth);
		file.close();

		mut.open("mutex.txt");
		mut.seekg(0, ios::beg);
		mut<<0;
		string posi;
		posi = numtostr(p_x) + "," + numtostr(240 - p_y);
		//imshow("Right Image", right);
		setMouseCallback("Left", CallBackFunc, NULL);	
		Point pt;
		pt.x = p_x;
		pt.y = 240 - p_y;
		int out = inorout(pt);
		putText(edge,posi,pt,0,0.5,Scalar(255,0,0),2,8,0); 
		imshow("Contors",drawing);
		imshow("Edge",edge);
		
	}
	else if(ctrl_flag==1 && m==1)
	//else if(ctrl_flag==1)
	{
		Vector <int> c;
		cl.open("c.txt");
		string line;
		int county = 0;
		while(!cl.eof())
		{
			if(county == final_bound.size())
				break;
			county++;
			cl>>temp_n;
			c.push_back(temp_n);
		}
		cl.close();

		for(int i=0;i<final_bound.size();i++)
		{
			box_obj.n=i;
			box_obj.r=final_bound[i];
			box_obj.v=depth3d.at<Vec3f>(Point(final_bound[i].x+final_bound[i].width/2,final_bound[i].y+final_bound[i].height/2));
			double depth_const1 = 18.4, depth_const2=17.6;
			box_obj.v[0] = box_obj.v[0]/depth_const1;
			box_obj.v[1] = box_obj.v[1]/depth_const2;
			string text;
			if(c[i]==0)
			{
				text = "BALL";				
				//storing depth to ball in kin_file
				kin_file<<c[i]<<" ";
				kin_file<<(-32.5+6)<<" ";
				kin_file<<(-(box_obj.v[1] - 16.5 -13.7))<<" ";
				kin_file<<(-(box_obj.v[0] - (-0.8/3)*box_obj.v[0]))<<endl;

			}
			else if(c[i]==1)
			{	
				text = "GLASS";
				
				double tmp_ratio = max(final_bound[i].width, final_bound[i].height)/min(final_bound[i].width, final_bound[i].height);
				double pixel_shift = 0.5/tmp_ratio;
				
				double tmp_const_y = 0.22;
				double tmp_const_z = 0.13;
				
				kin_file<<c[i]<<" ";
				kin_file<<(-32.5+25)<<" ";
				kin_file<<(-(box_obj.v[1] - 16.5 - 14.7 - tmp_const_y*(box_obj.v[0])))<< " ";
				//kin_file<<(-(box_obj.v[0] - 2.5 - tmp_const*(0.5 - pixel_shift)*final_bound[i]))<<endl;
				kin_file<<(-(box_obj.v[0] - 2.5 - tmp_const_z*(-box_obj.v[0])))<<endl;
			}
			Scalar color = Scalar(0,255,0);
			putText(display,text,final_bound[i].tl(),0,0.5,color,2,8,0); 
			//cout<<"Depth to "<<text<<": "<<box_obj.v[2]<<"mm | ";
		}
		kin_file.close();
		//cout<<endl;
		ctrl_flag = 0;
		final_bound.clear();
		final_mid.clear();
		imshow("Display",display);
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
