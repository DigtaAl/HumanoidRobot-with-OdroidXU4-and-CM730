/*
 *
 *   Author: ONTA
 *
 */

#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <darwin/framework/StatusCheck.h>

#include <darwin/framework/MX28.h>
#include <darwin/framework/Head.h>
#include <darwin/framework/Action.h>
#include <darwin/framework/Walking.h>
#include <darwin/framework/BallFollower.h>
#include <darwin/framework/MotionStatus.h>
#include <darwin/framework/ImgProcess.h>
#include <darwin/framework/Magneto.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

#include <darwin/linux/LinuxDARwIn.h>

int baca;
int nani;
int mag;
int awal;
int yawkotor;
int areabola;
int tendang = 1;
bool manualll = true;
bool searchMuter = true;
bool magen = false;
bool cyan  = false;


bool kickoff = true;
bool pas = true;
bool deteksi = false;
bool C1 = true;

double 	posX;
double 	posY;
double 	centerX;
double 	centerY;
double 	offsetX;
double	offsetY;
double	arahX;
double	arahY;
bool majuArahKiri = false;
bool majuArahKanan = false;
bool majuLurus = false;
bool tendangSKanan = false;
bool tendangSKiri = false;

struct 	termios SerialPortSettings;
char 	buf[32]; 

/* Arah Kompass */
bool	isChecked = false;
bool	isGoal = false;

int mid = 102;							// Nilai tengah
int leftLimit = mid - 100;			// Batas awal arah lurus
int rightLimit = mid + 100;			// Batas akhir arah lurus
int Behind = mid + 180;					// Batas akhir arah kanan
int limitLeft = mid + 1;				// Batas putaran kiri
int limitRight = mid - 1;				// Batas putaran kanan

using namespace cv;
using namespace std;
using namespace Robot;

Point2D last_pos;
BallTracker::BallTracker() :
	ball_position(Point2D(-1.0, -1.0))
{
NoBallCount = 0;
}
 

BallTracker::~BallTracker()
{
}

BallFollower::BallFollower()
{
	m_NoBallMaxCount 	= 10;
	m_NoBallCount 		= m_NoBallMaxCount;
	m_KickBallMaxCount 	= 5;
	m_KickBallCount 	= 0;

	m_KickTopAngle 		= -5.0;
	m_KickRightAngle 	= -15.0; //awal -30
	m_KickLeftAngle 	= 15.0;  //awal 30

	m_FollowMaxFBStep 	= 15.0;
    m_FollowMinFBStep 	= 10.0;
	m_FollowMaxRLTurn 	= 40.0; // awal 20
	m_FitFBStep 		= 3.0;
	m_FitMaxRLTurn 		= 13.0;
	m_UnitFBStep 		= 0.5; //0.3
	m_UnitRLTurn 		= 1;

	m_GoalFBStep 		= 0;
	m_GoalRLTurn		= 0;
	m_FBStep 			= 0;
	m_RLTurn 			= 0;
	DEBUG_PRINT 		= false;
	KickBall 			= 0;
	
	yaw_tolerance		= 10;
}

BallFollower::~BallFollower(){}

//double hasil;
//hasil = ball_position;
Point2D Last_Pos;
Point2D ball_pos;
//Point2D ball_pos;

int 	Start_Search	= 1;
int     start_search    = 1;
//int 	mag				= 119;
double 	FB_move			= 0; 
bool 	geser;
int 	arah_serang, gawang_kawan, selisih_serang;
int 	a,b,c;

int detekbal = 0;
int angkat = 0;

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"
#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"
#define U2D_DEV_CM      "/dev/ttyUSBCM"

LinuxCM730 linux_cm730(U2D_DEV_CM);
CM730 cm730(&linux_cm730);

void Tendang()
{
//	MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
    Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
 
}
void netral()
{	
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 29.0;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
    Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Z_SWAP_AMPLITUDE = 0;
}
void jalantempat1()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	//Walking::GetInstance()->PERIOD_TIME = 6;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=15;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
	
}
void jalantempatmuter()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	//Walking::GetInstance()->PERIOD_TIME = 6;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=15;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 1.5;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
	
}
void jalantempat3()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
	
}
void jalantempat111()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}
void jalantempat1111()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->PERIOD_TIME = 650;
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
	
}
void jalantempat11111()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 685;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 1;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
	
}
void Sebelumjalan()
{
	jalantempat111();
	usleep(1000000);
	jalantempat1111();
	usleep(24000000);
	jalantempat11111();
	usleep(800000);
}

void Belokkanan()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -20;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
	
}
void Belokkiri()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
	
}
void Belokkanandaritengah()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -20;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
	
}
void Belokkiridaritengah()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 10;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
}
void lurus()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
}	
void muterkiri()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=5;
}
void muterkanan()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=5;
}

void muterkepalaKanan()
{
	Head::GetInstance()->MoveByAngle(120,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(90,40);
	jalantempatmuter();
	usleep(800000);
	Head::GetInstance()->MoveByAngle(45,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,40);
	jalantempatmuter();
	usleep(800000);
	Head::GetInstance()->MoveByAngle(-45,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-90,40);
	jalantempatmuter();
	usleep(800000);
	Head::GetInstance()->MoveByAngle(-120,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,-20);
	jalantempatmuter();
	usleep(500000);
}
void muterkepalaKiri()
{
	Head::GetInstance()->MoveByAngle(120,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(90,40);
	jalantempat1();
	usleep(800000);
	Head::GetInstance()->MoveByAngle(45,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,40);
	jalantempat1();
	usleep(800000);
	Head::GetInstance()->MoveByAngle(-45,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-90,40);
	jalantempat1();
	usleep(800000);
	Head::GetInstance()->MoveByAngle(-120,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,-20);
	jalantempat1();
	usleep(500000);
}
void Alpha(){
	cout<<"Masuk Alpha"<<endl;
	cout<<"Belok"<<endl;
	Belokkanan();
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(800000);
	cout<<"Lurus Muter"<<endl;
	lurus();
	Head::GetInstance()->MoveByAngle(80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1700000);
	cout<<"Muter tajam"<<endl;
	muterkiri();
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	jalantempat3();
	Head::GetInstance()->MoveToHome();
	StatusCheck::Check(cm730);
	usleep(500000);
	
}
void Beta(){
	cout<<"Masuk Beta"<<endl;
	jalantempat3();
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(500000);
	cout<<"Belok kanan"<<endl;
	Belokkanandaritengah();
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	cout<<"Belok kiri"<<endl;
	Belokkiridaritengah();
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	jalantempat3();
	StatusCheck::Check(cm730);
	usleep(500000);
	Head::GetInstance()->MoveByAngle(80,-15);
	muterkiri();
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,30);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveToHome();
	jalantempat3();
	StatusCheck::Check(cm730);
	usleep(500000);
}
void Charlie(){
	cout<<"Masuk Charlie"<<endl;
	cout<<"Belok"<<endl;
	Belokkiri();
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(1200000);
	Head::GetInstance()->MoveByAngle(-80,-15);
	StatusCheck::Check(cm730);
	usleep(1000000);
	Head::GetInstance()->MoveByAngle(80,-15);
	StatusCheck::Check(cm730);
	usleep(500000);
	Head::GetInstance()->MoveByAngle(80,30);
	cout<<"Lurus Muter"<<endl;
	lurus();
	StatusCheck::Check(cm730);
	usleep(2000000);
	Head::GetInstance()->MoveByAngle(-80,30);
	StatusCheck::Check(cm730);
	usleep(2500000);
	Head::GetInstance()->MoveToHome();
	cout<<"Muter tajam"<<endl;
	muterkanan();
	StatusCheck::Check(cm730);
	usleep(8000000);
}
 
 cv::Mat imgOriginal;
 cv::Mat hsvImg;
 cv::Mat hslImg;
 cv::Mat bola;
 cv::Mat tiang;
 cv::Mat lapangan;
  


void BallFollower::Process()
{	
	if (leftLimit < 0 )	leftLimit = leftLimit + 360;
				
	if (rightLimit > 360 ) rightLimit = rightLimit - 360;
				
	if (Behind > 360 ) Behind = Behind - 360;
				
	if (limitLeft < 0 )	limitLeft = limitLeft + 360;
				
	if (limitRight > 360 ) limitRight = limitRight - 360;
	
    Head::GetInstance()->MoveToHome();
	Head::GetInstance()->MoveTracking();
    
	//Mat imgOriginal, hsvImg, threshImg, ciko, dilatet, sayang, kucing, crop1;


    const int FRAME_WIDTH = 320;
    const int FRAME_HEIGHT = 240;
   
   
 int lowH = 33;       // Set Hue
 int highH = 97;

 int lowS = 30;       // Set Saturation
 int highS = 255;

 int lowV = 0;       // Set Value
 int highV = 255;
	 
 int erotion_size = 8;
 int dilation_size = 0; 
 int dilasi = 18;
//-----------------------------------------------------

 int lowH1 = 0;       // Set Hue
 int highH1 = 26;

 int lowS1 = 140;       // Set Saturation
 int highS1 = 255;

 int lowV1 = 74;       // Set Value
 int highV1 = 255;	

 int erotion_size1 = 0;
 int dilation_size1 = 0; 		
//-----------------------------------------------------

    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    

    //cv::namedWindow("Setting Bola", CV_WINDOW_AUTOSIZE);
	//cv::namedWindow("Setting Lapangan", CV_WINDOW_AUTOSIZE);
	//cv::namedWindow("Lapangan", CV_WINDOW_AUTOSIZE);
	//cv::moveWindow("Lapangan",0,0);


    	 /* Create trackbars in "threshImg" window to adjust according to object and environment.*/
	
			  cv::createTrackbar("LowH", "Setting Lapangan", &lowH, 179); //Hue (0 - 179)
			  cv::createTrackbar("HighH", "Setting Lapangan", &highH, 179);

			  cv::createTrackbar("LowS", "Setting Lapangan", &lowS, 255); //Saturation (0 - 255)
			  cv::createTrackbar("HighS", "Setting Lapangan", &highS, 255);
  
			  cv::createTrackbar("LowV", "Setting Lapangan", &lowV, 255); //Value (0 - 255)
			  cv::createTrackbar("HighV", "Setting Lapangan", &highV, 255);
			  
			  cv::createTrackbar("opening", "Setting Lapangan", &erotion_size, 255);
			  cv::createTrackbar("closing", "Setting Lapangan", &dilation_size, 255);
			  cv::createTrackbar("dilasi", "Setting Lapangan", &dilasi, 255);
			  //-------------------------------------------------------------------
			  
			  cv::createTrackbar("LowH", "Setting Bola", &lowH1, 179); //Hue (0 - 179)
			  cv::createTrackbar("HighH", "Setting Bola", &highH1, 179);

			  cv::createTrackbar("LowS", "Setting Bola", &lowS1, 255); //Saturation (0 - 255)
			  cv::createTrackbar("HighS", "Setting Bola", &highS1, 255);
  
			  cv::createTrackbar("LowL", "Setting Bola", &lowV1, 255); //Value (0 - 255)
			  cv::createTrackbar("HighL", "Setting Bola", &highV1, 255);
			
			  cv::createTrackbar("opening", "Setting Bola", &erotion_size1, 255);
			  cv::createTrackbar("closing", "Setting Bola", &dilation_size1, 255);
				
    while(true)
    {	
		cout<<"Bangunnnnnnnnnnnnnn"<<endl;
		StatusCheck::Check(cm730);
		deteksi = false;
		cap >>imgOriginal;
		Action::GetInstance()->LoadFile(MOTION_FILE_PATH);		
       
		cv::Mat gambarkopi = imgOriginal.clone();
			  cv::Mat Brighnessdec50;
			  //imgOriginal.convertTo(Brighnessdec50,-1,1,-50);  
			  imgOriginal.convertTo(Brighnessdec50,-1,0.5,-50);
			  cv::cvtColor(Brighnessdec50, hsvImg, CV_BGR2HSV);      // Convert Original Image to HSV Thresh Image
			  cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), lapangan);
			  

			// cv::GaussianBlur(lapangan, lapangan, cv::Size(3, 3), 0);   //Blur Effect
			 //------------------------------------------------------------------morphologi lapangan
			 //morphologi opening
			  Mat element = getStructuringElement(cv::MORPH_RECT,
              cv::Size(2 * erotion_size + 1, 2 * erotion_size + 1),
              cv::Point(erotion_size, erotion_size) );
				//erode(bola,bola,element1);
				
				morphologyEx(lapangan,lapangan,2,element);
			
			//morphologi closing
			Mat element1 = getStructuringElement(cv::MORPH_RECT,
              cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
              cv::Point(dilation_size, dilation_size) );
			  //dilate(bola,bola,element);
			  morphologyEx(lapangan,lapangan,3,element1);
			 
						 
					  
			//-----------------------------------------------------------------------------------------------find contour dan masking lapangan	
			
			//invert lapangan
			//cv::bitwise_not(lapangan,lapangan);
			
			
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			int largest_area = 0;
			int largest_contour_index = 0;
							
			findContours(lapangan, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

			//this will find largest contour
			for (int i = 0; i< contours.size(); i++) // iterate through each contour.
			{
				double a = contourArea(contours[i], false);  //  Find the area of contour
				if (a>largest_area)
				{
					largest_area = a;
					largest_contour_index = i;                //Store the index of largest contour
									
				}
			
			}
			//search for largest contour has end
							
			
			//------------------------------------------------------------------------------------------------masking lapangan 1
			Mat finale;
			Mat lapangan2 = lapangan.clone();
			Mat masking = Mat::zeros(lapangan2.rows, lapangan2.cols, CV_8UC1);
			//masking.setTo(Scalar(0,0,0));
			Brighnessdec50.copyTo(masking,lapangan2);
					
			
			
			//------------------------------------------------------------------------------------------------fill hole
			
			
			Mat im_floodfill = lapangan2.clone();
			floodFill(im_floodfill, cv::Point(0,0), Scalar(255));
			Mat im_floodfill_inv;
			bitwise_not(im_floodfill, im_floodfill_inv);
			Mat im_out = (lapangan2 | im_floodfill_inv);
		    //bitwise_not(im_out,im_out);
		    
		    //------------------------------------------------------------------------------------------------------masking lapangan 2
			Mat im_out2 = im_out.clone();
			 
			   //morphologi dilasi
			Mat element69 = getStructuringElement(cv::MORPH_RECT,
              cv::Size(2 * dilasi + 1, 2 * dilasi + 1),
              cv::Point(dilasi, dilasi) );
			 dilate(im_out2,im_out2,element69);
			 
			Mat masking3 = Mat::zeros(im_out2.rows, im_out2.cols, CV_8UC1);
			//masking.setTo(Scalar(0,0,0));
			Brighnessdec50.copyTo(masking3,im_out2);
		
			//--------------------------------------------------------------------------------------------------xxxx
		
			//--------------------------------------------------------------------------------------------------convert warna
			cv::cvtColor(masking3, hslImg, CV_BGR2HSV);      // Convert Original Image to HLS Thresh Image	
			cv::inRange(hslImg, cv::Scalar(lowH1, lowS1, lowV1), cv::Scalar(highH1, highS1, highV1), bola);
		   
		   //------------------------------------------------------------------------------------------------morphologi bola
			 //morphologi opening
			  Mat element_bola = getStructuringElement(cv::MORPH_ELLIPSE,
              cv::Size(2 * erotion_size1 + 1, 2 * erotion_size1 + 1),
              cv::Point(erotion_size1, erotion_size1) );
				//erode(bola,bola,element1);
				morphologyEx(bola,bola,2,element_bola);
			
			//morphologi closing
			Mat element_bola1 = getStructuringElement(cv::MORPH_ELLIPSE,
              cv::Size(2 * dilation_size1 + 1, 2 * dilation_size1 + 1),
              cv::Point(dilation_size1, dilation_size1) );
			  //dilate(bola,bola,element);
			  morphologyEx(bola,bola,3,element_bola1);
			 
		    //-----------------------------------------------------------------------------------------------find contour bola
			    //invert bola
			//cv::bitwise_not(bola,bola);
			
			Rect bounding_rect;
			vector<vector<Point> > contours1;
			vector<Vec4i> hierarchy1;
			
			int largest_area1 = 0;
			int largest_contour_index1 = 0;

			findContours(bola, contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
			
			vector<Moments> mu1(contours1.size()); //get moments
				
				
				for (int i = 0; i < contours1.size(); i++)
				{
					mu1[i] = moments(contours1[i], false);
				}
			
			//this will find largest contour
			for (int i = 0; i< contours1.size(); i++) // iterate through each contour.
			{
				double a = contourArea(contours1[i], false);  //  Find the area of contour
				if (a>largest_area1)
				{
					largest_area1 = a;
					largest_contour_index1 = i;                //Store the index of largest contour
					bounding_rect=boundingRect(contours1[i]); // Find the bounding rectangle for biggest contour
				deteksi = true;
				posX = mu1[i].m10 / mu1[i].m00;
				posY = mu1[i].m01 / mu1[i].m00;
				}

				//areabola = contourArea(contours1[0]);
				//cout <<"Area : "<<a<<endl;
			}
			//search for largest contour has end
			
			Mat gabung;
				cv::hconcat(Brighnessdec50,masking3,gabung);	
			
			if (contours1.size() > 0)
			{
				rectangle(gabung, bounding_rect,  Scalar(0,255,0),2, 8,0);

			}
							
				
			//	cout<<"bola x : "<<posX<<endl;
			//	cout<<"bola y : "<<posY<<endl;
			
			 cv::imshow("Setting Lapangan", lapangan);
			 cv::imshow("Setting Bola", bola);
		     cv::imshow("Lapangan", gabung);
		    
		    StatusCheck::Check(cm730);
		     if(StatusCheck::cek_kondisi==1)
			{
					m_NoBallMaxCount 	= 10;
					m_NoBallCount 		= m_NoBallMaxCount;
					m_KickBallMaxCount 	= 5;
					m_KickBallCount 	= 0;

					m_KickTopAngle 		= -5.0;
					m_KickRightAngle 	= -15.0; //awal -30
					m_KickLeftAngle 	= 15.0;  //awal 30

					m_FollowMaxFBStep 	= 15.0;
					m_FollowMinFBStep 	= 10.0;
					m_FollowMaxRLTurn 	= 40.0; // awal 20
					m_FitFBStep 		= 3.0;
					m_FitMaxRLTurn 		= 13.0;
					m_UnitFBStep 		= 0.5;
					m_UnitRLTurn 		= 1;

					m_GoalFBStep 		= 0;
					m_GoalRLTurn		= 0;
					m_FBStep 			= 0;
					m_RLTurn 			= 0;
					DEBUG_PRINT 		= false;
					KickBall 			= 0;
					
					yaw_tolerance		= 10;
					
				Start_Search	= 1;
				start_search    = 1;
				FB_move			= 0; 
				detekbal = 0;
				angkat = 0;
				
				MotionManager::GetInstance()->SetEnable(true);
				Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				cout<<"Returnnnn "<<endl;
				StatusCheck::cek_kondisi=0;
				magen = true;
				cyan  = true;
				manualll = true;
				searchMuter = true;
				C1 = false;
                return;
			}
			
		
	if(deteksi==false)
		{
			posX = -1;
			posY = -1;	
			
			if(NoBallCount > NoBallMaxCount)
			{	
				if(offsetX>0 && offsetY>0)
				{
					 start_search=1;//kiri atas
					 Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
					 //Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
				 }
				else if(offsetX<0 && offsetY>0)
				{
					 start_search=2;//kanan atas
					 Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
					 //Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
				 }
				else if(offsetX<0 && offsetY<0)
				{
					 start_search=6;//kanan bawah
					 Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
					 //Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
				 }
				else if(offsetX>0 && offsetY<0) 
				{
					start_search=5;//kiri bawah
					Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
				}
				else 
				{
					start_search=1;
					Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
				}
				Walking::GetInstance()->X_MOVE_AMPLITUDE=0;	
				//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;			
				//Head::GetInstance()->SetStartSearch(start_search);
				Head::GetInstance()->Start=true;
				//fprintf(stderr,"Starsearch = %d \n",start_search);
				Head::GetInstance()->MoveTracking();
				NoBallCount++;
			}
			else
			{
				Head::GetInstance()->InitTracking(start_search);
			}	
		}		
	else
		{
			NoBallCount = 0;
			if (C1 == true){
				arahX = 160; //180 tengah
				arahY = 180;
			}
			else if (majuArahKiri == true){
				arahX = 145;
				arahY = 180;
			}
			else if (majuArahKanan == true){
				arahX = 195;
				arahY = 180;
			}
			else if (majuLurus == true){
				arahX 	= 187; //187 kanan
				arahY	= 180;
			}
			else{
				arahX 	= 187; //187 kanan
				arahY	= 180;
				}
				
			centerX	= arahX;
			centerY = arahY;
			
			offsetX = (posX - centerX);
			offsetY = (posY - centerY);

			offsetX *= -1; // Inverse X-axis, Y-axis
			offsetY *= -1;
			
			offsetX *= 0.14375; // pixel per angle
			offsetY *= 0.24167; // pixel per angle
			Head::GetInstance()->MoveTracking(offsetX, offsetY);

		}
		//cout<<"ballx :"<<offsetX<<endl;
		//cout<<"bally :"<<offsetY<<endl;
		
		
if(DEBUG_PRINT == true)
		fprintf(stderr, "\r\r");

	if(posX == -1.0 || posY == -1.0)
		{
			KickBall = 0;
			move = 0;

			if(m_NoBallCount > m_NoBallMaxCount)
			{
				// can not find a ball
				m_GoalFBStep = 0;
				m_GoalRLTurn = 0;
			
				Head::GetInstance()->InitTracking(Start_Search);
				int Putar = Head::GetInstance()->GetPutar();
				//cout<<"Putar	:"<<Putar<<endl;
				if(Putar==0)
				{
					if(Walking::GetInstance()->IsRunning() == false)
					{
						Walking::GetInstance()->Start();
					}
					else{
						if(C1 == true && searchMuter == true){
							cout<<"Ga detekkkkkkkkkkkkkkkkkkkk"<<endl;
							Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
						} 
						else if (searchMuter == true && C1 == false){
							if (mag >= (mid - 20) && mag <= (mid + 20)){
								searchMuter = false;
								}
							else{
								if(mag < (mid - 20) || mag > Behind){
									Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
									Walking::GetInstance()->A_MOVE_AMPLITUDE=-17;
									}
								else{
									Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
									Walking::GetInstance()->A_MOVE_AMPLITUDE=17;
									}
								}
						} else {
							Walking::GetInstance()->X_MOVE_AMPLITUDE=15;
							Walking::GetInstance()->A_MOVE_AMPLITUDE=0;
						}
					//Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
					Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
					}
				}
				else if(Putar==1)//kiri
				{
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
					//Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
					if(Walking::GetInstance()->IsRunning() == false) 
					Walking::GetInstance()->Start();
				}
				else if(Putar==2)//kanan
				{
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
					//Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
					if(Walking::GetInstance()->IsRunning() == false) 
					Walking::GetInstance()->Start();
				}
			
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[NO BALL]");
			
			}
			else
			{
				if(offsetX>0 && offsetY>0) start_search=1;//kiri atas
				else if(offsetX<0 && offsetY>0) start_search=2;//kanan atas
				else if(offsetX<0 && offsetY<0) start_search=6;//kanan bawah
				else if(offsetX>0 && offsetY<0) start_search=5;//kiri bawah
				else start_search=1;
				
				//Head::GetInstance()->SetStartSearch(Start_Search);
				Head::GetInstance()->Start=true;
				//fprintf(stderr,"Starsearch = %d \n",Start_Search);
			
				m_NoBallCount++;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
			}
			 
		}

	else
		{
			Last_Pos=ball_pos;
			
			Head::GetInstance()->putar=0;
			m_NoBallCount = 0;	

			double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			double pan_range = Head::GetInstance()->GetLeftLimitAngle();
			double pan_percent = pan / pan_range;
			
			double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
			double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
			double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
			double tilt_percent = (tilt - tilt_min) / tilt_range;
			if(tilt_percent < 0)
			tilt_percent = -tilt_percent;
			cout<<"-------------------------------------------"<<endl;
			cout<<"tilt :"<<tilt<<endl;
			//cout<<"-------------------------------------------"<<endl;
			//cout<<"pan	:"<<pan<<endl;

			/*Syarat robot lurus kearah gawang*/
				if ((mag >= leftLimit) && (mag <= rightLimit)){
					isGoal = true; 
					}
				else {
					isGoal = false;
					}

			
	if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
		{
		if (tilt <= 5){
			if(C1 == false) {
				searchMuter = false;
					}
				}
		geser = false;
		if(isGoal == false){
						if (tilt <= 20){
							//----------------------------------------------------------------------------------------------- Jika initial kurang dari 180
							if(mag > rightLimit && mag <= Behind - 5){	// Arah Kanan
								Alpha();
							} 
							else if ((mag < leftLimit && mag >= 1)|| mag > Behind + 5){	// Arah Kiri
								Charlie();
							}else{
								Beta();
								}
							//-----------------------------------------------------------------------------------------------
							
						return;
						}
						else	// Cari Bola
						{
							geser = false;
							m_KickBallCount = 0;
							KickBall = 0;
							m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
							if(m_GoalFBStep < m_FollowMinFBStep)
								m_GoalFBStep = m_FollowMinFBStep;
							m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent * 1.4;//1.5
							if(DEBUG_PRINT == true)
								fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
								detekbal=0;
								angkat =0;
						}	
					}
		else{
				if(mag <= (mid - 70) && mag >= leftLimit) {
					majuArahKanan = true;
					tendangSKanan = true;
					majuArahKiri = false;
					tendangSKiri = false;
					majuLurus = false;
				}
				else if(mag >= (mid + 70) && mag <= rightLimit)	{
					majuArahKiri = true;
					tendangSKiri = true;
					majuArahKanan = false;
					tendangSKanan = false;
					majuLurus = false;
				}
				else if(mag > (mid - 70) && mag < (mid + 70)){
					majuLurus = true;
					majuArahKanan = false;
					tendangSKanan = false;
					majuArahKiri = false;
					tendangSKiri = false;
				}
				else{
					majuArahKanan = false;
					majuArahKiri = false;
					tendangSKanan = false;
					tendangSKiri = false;
					majuLurus = true;
					}
				
			
				if (tilt <= -5)
				{
						if(m_KickBallCount >= 0)
						{
							m_FBStep = 0;
							m_RLTurn = 0;
							if (pas == true) {
							if (C1 == true){
								jalantempat1();
								usleep(630000*2);
								Tendang();
								usleep(500000);
								Action::GetInstance()->Start(126); // 108
								while(Action::GetInstance()->IsRunning()) usleep(8*1000);
								Walking::GetInstance()->Stop();
							}
							else if (tendangSKanan == true){
								muterkepalaKanan();
								Tendang();
								usleep(500000);
								Action::GetInstance()->Start(127); // 108
								while(Action::GetInstance()->IsRunning()) usleep(8*1000);
								Action::GetInstance()->Start(14); //ganti walk
								while(Action::GetInstance()->IsRunning()) usleep(8*1000);
								searchMuter = true;
							}
							else if (tendangSKiri == true){
								muterkepalaKiri();
								Tendang();
								usleep(500000);
								Action::GetInstance()->Start(128); // 108
								while(Action::GetInstance()->IsRunning()) usleep(8*1000);
								Action::GetInstance()->Start(14); //ganti walk
								while(Action::GetInstance()->IsRunning()) usleep(8*1000);
								searchMuter = true;
							}
							else {
								jalantempat1();
								usleep(630000*2);
								Tendang();
								usleep(500000);
								Action::GetInstance()->Start(124); // 108
								while(Action::GetInstance()->IsRunning()) usleep(8*1000);
							}
							//Action::GetInstance()->Start(124); // 108
							//while(Action::GetInstance()->IsRunning()) usleep(8*1000);
							//jalantempat1();
							//usleep(63000*2);
							tendangSKanan = false;
							tendangSKiri = false;
							majuLurus = false;
							if(DEBUG_PRINT == true)
								fprintf(stderr, "[KICK]");
								detekbal = 0;
								}
							if(C1 == false){
								magen = false;
									return;
								}
						}
						else
						{
							KickBall = 0;
							if(DEBUG_PRINT == true)
								fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
						}
				}
				else
				{
					geser = false;
					m_KickBallCount = 0;
					KickBall = 0;
					m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
					if(m_GoalFBStep < m_FollowMinFBStep)
						m_GoalFBStep = m_FollowMinFBStep;
					m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent * 1.4;//1.5
					if(DEBUG_PRINT == true)
						fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
						detekbal=0;
						angkat =0;
				}
			}
		}
	else
		{
			geser = false;
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent * 1.4;//1.5
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
				detekbal=0;

	 }	
	 
	//if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0 && geser == false)
	if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0)
		{
		if(Head::GetInstance()->GetPutar()<1)
		{
			if(Walking::GetInstance()->IsRunning() == true)
				Walking::GetInstance()->Stop();
			else
			{
				if(m_KickBallCount < m_KickBallMaxCount)
					m_KickBallCount++;
			}
		
			if(DEBUG_PRINT == true)
				fprintf(stderr, " STOP");
		}
	}
	else
		{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " START");

		if(Walking::GetInstance()->IsRunning() == false)
		{
			m_FBStep = 0;
			m_RLTurn = 0;
			m_KickBallCount = 0;
			KickBall = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			Walking::GetInstance()->Start();			
		}
		else
		{
			if(detekbal==0)
			{
				//cout << "LURUUUUS"<<endl;
				Walking::GetInstance()->HIP_PITCH_OFFSET = 30.5; //28.1
				//Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
				Walking::GetInstance()->A_MOVE_AMPLITUDE = 1.5;
				Walking::GetInstance()->Z_MOVE_AMPLITUDE = 35;
				//Walking::GetInstance()->Y_OFFSET =59;
				
				
			if(m_FBStep < m_GoalFBStep)
				m_FBStep += m_UnitFBStep;
			else if(m_FBStep > m_GoalFBStep)
				m_FBStep = m_GoalFBStep;
				
			if( (m_RLTurn >= (m_FollowMaxRLTurn-5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn+5) && m_RLTurn < 0))
			{
				//fprintf(stderr,"Rl Turn : %f \n",m_RLTurn);
				FB_move= m_FBStep - 1;
			}
			else FB_move= m_FBStep;
			
			FB_move+=0.5;//kalibrasi dari setting walk tunner
			Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
			cout << "X_Move	:"<<FB_move<<endl;

			if(m_RLTurn < m_GoalRLTurn)
				m_RLTurn += m_UnitRLTurn;
			else if(m_RLTurn > m_GoalRLTurn)
				m_RLTurn -= m_UnitRLTurn;
		
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

			if(DEBUG_PRINT == true)
				fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
			}
		}
	}

	if (waitKey(30) == 27)
		{
				cout<<"keluaaar   "<<endl;
				break;
		}
	}

}
}

