/*
 *
 *   Author: 
 *   Editor : Digta
 *
 */

#include <cmath>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <termios.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <darwin/framework/Comm.h>
#include <darwin/framework/MX28.h>
#include <darwin/framework/Head.h>
#include <darwin/framework/Action.h>
#include <darwin/framework/Walking.h>
#include <darwin/framework/Magneto.h>
#include <darwin/linux/LinuxDARwIn.h>
#include <darwin/framework/Konstanta.h>
#include <darwin/framework/Interface.h>
#include <darwin/framework/VoidAction.h>
#include <darwin/framework/ImgProcess.h>
#include <darwin/framework/StatusCheck.h>
#include <darwin/framework/GameControl.h>
#include <darwin/framework/BallFollower.h>
#include <darwin/framework/MotionStatus.h>

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//--- Deteksi + Kondisi
//bool 	C1 				= true;
bool 	cyan  			= false;
bool 	magen 			= false;
bool 	deteksi 		= false;
bool 	manualll		= true;
bool 	majuLurus 		= false;
bool 	searchMuter		= true;
bool 	majuArahKiri 	= false;
bool 	tendangSKiri 	= false;
bool 	majuArahKanan 	= false;
bool 	tendangSKanan 	= false;
int TILT_0;
int TILT_1 				= -10;
int TILT_2 				= -10;
int waitcount;
int countmax			= 15;

struct 	termios SerialPortSettings; 

/* Arah Kompass */
bool	isChecked 	= false;
bool	isGoal 		= false;

int mid 		= 0;							// Nilai tengah 0
int Behind 		= mid + 180;					// Batas akhir arah kanan 180
int leftLimit 	= mid - 70;						// Batas awal arah lurus 290
int limitLeft 	= mid + 1;						// Batas putaran kiri 359
int rightLimit 	= mid + 70;						// Batas akhir arah lurus 70
int limitRight 	= mid - 1;						// Batas putaran kanan 1

//GameController
//////////////////////////////Syarat awal///////////////////////////////////////////////////////
int i = 0;

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
	m_KickRightAngle 	= -35.0; //awal -30
	m_KickLeftAngle 	= 35.0;  //awal 30

	m_FollowMaxFBStep 	= 25.0; // bisa 30
    m_FollowMinFBStep 	= 11.5;
	m_FollowMaxRLTurn 	= 30.0; // awal 20
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

Point2D Last_Pos;
Point2D ball_pos;

int 	a,b,c;
bool 	geser;
double 	FB_move			= 0; 
int 	angkat 			= 0;
int 	detekbal		= 0;
int 	Start_Search	= 1;
int     start_search    = 1;

LinuxCM730 linux_cm730(U2D_DEV_CM);
CM730 cm730(&linux_cm730);

void KoreksiArah(){
	if (Behind > 360 ) 		Behind = Behind - 360;
	if (leftLimit < 0 )		leftLimit = leftLimit + 360;
	if (limitLeft < 0 )		limitLeft = limitLeft + 360;
	if (rightLimit > 360 ) 	rightLimit = rightLimit - 360;
	if (limitRight > 360 ) 	limitRight = limitRight - 360;
}
 
 cv::Mat imgOriginal;
 cv::Mat hsvImg;
 cv::Mat hslImg;
 cv::Mat bola;
 cv::Mat tiang;
 cv::Mat lapangan;

void BallFollower::Process()
{	
	KoreksiArah();

    Head::GetInstance()->MoveToHome();
	Head::GetInstance()->MoveTracking();

    const int FRAME_WIDTH 	= 320;
    const int FRAME_HEIGHT 	= 240;
   
	int lowH 			= 33;	// Set Hue
	int highH 			= 97;

	int lowS 			= 30;	// Set Saturation
	int highS 			= 255;

	int lowV 			= 0;	// Set Value
	int highV 			= 255;
		
	int erotion_size 	= 1;
	int dilation_size 	= 0; 
	int dilasi 			= 4;
	//-----------------------------------------------------

	int lowH1 			= 0;	// Set Hue
	int highH1 			= 30;

	int lowS1 			= 100;	// Set Saturation
	int highS1 			= 255;

	int lowV1 			= 20;   // Set Value
	int highV1 			= 255;	

	int erotion_size1 	= 0;
	int dilation_size1 	= 0; 			
	//-----------------------------------------------------

    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    
    //cv::namedWindow("Setting Bola", CV_WINDOW_AUTOSIZE);
	//cv::namedWindow("Setting Lapangan", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Lapangan", CV_WINDOW_AUTOSIZE);
	cv::moveWindow("Lapangan",0,0);
				
    while(true){
		StatusCheck::Check(cm730);			
		deteksi = false;
		cap >>imgOriginal;
		Action::GetInstance()->LoadFile(const_cast<char*>(MOTION_FILE_PATH));

		cv::Mat gambarkopi = imgOriginal.clone();
		cv::Mat Brighnessdec50;
		imgOriginal.convertTo(Brighnessdec50,-1,0.5,-10);
		imgOriginal = Brighnessdec50;   
	    cv::cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);      // Convert Original Image to HSV Thresh Image
		cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), lapangan);
		// cv::GaussianBlur(lapangan, lapangan, cv::Size(3, 3), 0);   //Blur Effect

		//------------------------------------------------------------------ Morphologi Lapangan
		//Morphologi Opening
		Mat element = getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * erotion_size + 1, 2 * erotion_size + 1),
        cv::Point(erotion_size, erotion_size) );
		//erode(bola,bola,element1);
		morphologyEx(lapangan,lapangan,2,element);
			
		//Morphologi Closing
		Mat element1 = getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size) );
		//dilate(bola,bola,element);
		morphologyEx(lapangan,lapangan,3,element1);						 
				  
		//------------------------------------------------------------------Find Contour & Masking Lapangan	
		//Invert Lapangan
		//cv::bitwise_not(lapangan,lapangan);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		int largest_area = 0;
		int largest_contour_index = 0;			
		findContours(lapangan, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		//This Will Find Largest Contour
		for (size_t i = 0; i < contours.size(); i++) // Iterate through each contour.
		{
			double a = contourArea(contours[i], false);  //  Find the area of contour
			if (a>largest_area)
			{
				largest_area = a;
				largest_contour_index = i;                //Store the index of largest contour		
			}
		}

		//Aearch for Largest Contour has End				
		//------------------------------------------------------------------------------------------------Masking Lapangan 1
		Mat finale;
		Mat lapangan2 = lapangan.clone();
		Mat masking = Mat::zeros(lapangan2.rows, lapangan2.cols, CV_8UC1);
		//masking.setTo(Scalar(0,0,0));
		imgOriginal.copyTo(masking,lapangan2);

		//------------------------------------------------------------------------------------------------Fill Hole
		Mat im_floodfill = lapangan2.clone();
		floodFill(im_floodfill, cv::Point(0,0), Scalar(255));
		Mat im_floodfill_inv;
		bitwise_not(im_floodfill, im_floodfill_inv);
		Mat im_out = (lapangan2 | im_floodfill_inv);
		//bitwise_not(im_out,im_out);
		
		//------------------------------------------------------------------------------------------------------Masking Lapangan 2
		Mat im_out2 = im_out.clone();
		//morphologi dilasi
		Mat element69 = getStructuringElement(cv::MORPH_RECT,
		cv::Size(2 * dilasi + 1, 2 * dilasi + 1),
		cv::Point(dilasi, dilasi) );
		dilate(im_out2,im_out2,element69);
		Mat masking3 = Mat::zeros(im_out2.rows, im_out2.cols, CV_8UC1);
		//masking.setTo(Scalar(0,0,0));
		imgOriginal.copyTo(masking3,im_out2);
		//--------------------------------------------------------------------------------------------------
		
		//--------------------------------------------------------------------------------------------------Convert Warna
		cv::cvtColor(masking3, hslImg, CV_BGR2HSV);      // Convert Original Image to HLS Thresh Image	
		cv::inRange(hslImg, cv::Scalar(lowH1, lowS1, lowV1), cv::Scalar(highH1, highS1, highV1), bola);
		
		//------------------------------------------------------------------------------------------------Morphologi Bola
		//Morphologi opening
		Mat element_bola = getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * erotion_size1 + 1, 2 * erotion_size1 + 1),
		cv::Point(erotion_size1, erotion_size1) );
		//erode(bola,bola,element1);
		morphologyEx(bola,bola,2,element_bola);

		//Morphologi closing
		Mat element_bola1 = getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * dilation_size1 + 1, 2 * dilation_size1 + 1),
		cv::Point(dilation_size1, dilation_size1) );
		//dilate(bola,bola,element);
		morphologyEx(bola,bola,3,element_bola1);
		  
		//-----------------------------------------------------------------------------------------------fFind Contour Bola
		//Invert bola
		//cv::bitwise_not(bola,bola);
		Rect bounding_rect;
		vector<vector<Point> > contours1;
		vector<Vec4i> hierarchy1;

		int largest_area1 = 0;
		int largest_contour_index1 = 0;

		findContours(bola, contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
		vector<Moments> mu1(contours1.size()); //Get moments

		for (size_t i = 0; i < contours1.size(); i++)
		{
			mu1[i] = moments(contours1[i], false);
		}
	
		//this will find largest contour
		for (size_t i = 0; i< contours1.size(); i++) // Iterate through each contour.
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

		//Search for Largest Contour has End
		Mat gabung;
		cv::hconcat(imgOriginal,masking3,gabung);	
		if (contours1.size() > 0)
		{
			rectangle(gabung, bounding_rect,  Scalar(0,255,0),2, 8,0);
		}					
		//cout<<"bola x : "<<posX<<endl;
		//cout<<"bola y : "<<posY<<endl;
		cv::imshow("Setting Lapangan", lapangan);
		cv::imshow("Setting Bola", bola);
		cv::imshow("Lapangan", gabung);

		StatusCheck::Check(cm730);
		if(StatusCheck::cek_kondisi == 1){
			m_NoBallMaxCount 	= 10;
			m_NoBallCount 		= m_NoBallMaxCount;
			m_KickBallMaxCount 	= 5;
			m_KickBallCount 	= 0;

			m_KickTopAngle 		= -5.0;
			m_KickRightAngle 	= -35.0; //awal -30
			m_KickLeftAngle 	= 35.0;  //awal 30

			m_FollowMaxFBStep 	= 17.0;
			m_FollowMinFBStep 	= 11.7;
			m_FollowMaxRLTurn 	= 25.0; // awal 20
			m_FitFBStep 		= 3.0;
			m_FitMaxRLTurn 		= 13.0;
			m_UnitFBStep 		= 0.5;
			m_UnitRLTurn 		= 0.8;

			m_GoalFBStep 		= 0;
			m_GoalRLTurn		= 0;
			m_FBStep 			= 0;
			m_RLTurn 			= 0;
			DEBUG_PRINT 		= false;
			KickBall 			= 0;
			
			yaw_tolerance		= 10;
			
		angkat 			= 0;
		FB_move			= 0; 
		detekbal 		= 0;
		Start_Search	= 1;
		start_search    = 1;
		
		MotionManager::GetInstance()->SetEnable(true);
		Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
		Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
		StatusCheck::cek_kondisi = 0;
		//C1 			= false;
		cyan  		= true;
		magen 		= true;
		manualll 	= true;
		searchMuter = true;
		jalanManual = true;
		cout<<"Returnnnn "<<endl;
		return;
		}

		if(deteksi == false){
			mode = 9;
			waitcount = 0;
			cekSend2 = false;
			if(cekSend1 == false){
				shouldSend 		= true;
				numberToSend	= 0;
				cekSend1		= true;
			}
			posX = -1;
			posY = -1;	
			if(NoBallCount > NoBallMaxCount){	
				if(offsetX > 0 && offsetY > 0)
				{
					start_search = 1;//kiri atas
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
					}
				else if(offsetX < 0 && offsetY > 0)
				{
					start_search = 2;//kanan atas
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
					}
				else if(offsetX < 0 && offsetY < 0)
				{
					start_search = 6;//kanan bawah
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
					}
				else if(offsetX > 0 && offsetY < 0) 
				{
					start_search = 5;//kiri bawah
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
					}
				else 
				{
					start_search = 1;
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
					//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
				}
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;	
				//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;			
				//Head::GetInstance()->SetStartSearch(start_search);
				Head::GetInstance()->Start = true;
				//fprintf(stderr,"Starsearch = %d \n",start_search);
				Head::GetInstance()->MoveTracking();
				NoBallCount++;
				}
			else
			{
				Head::GetInstance()->InitTracking(start_search);
			}	
		}		
		else{
			mode = 8;
			if((receivedNumber == STAT_ROBOT1) || (receivedNumber == STAT_ROBOT2) || (receivedNumber == STAT_ROBOT4)){
				std::cout<<" Robot 3 tidak maju " <<endl;
				m_FBStep 	= 0;
				FB_move 	= 0;
			}
			NoBallCount = 0;
			if (majuLurus == true){
				arahX = 180;
				TILT_0 = TILT_1;
			}			 
			else if (majuArahKiri == true){
				arahX = 135;
				TILT_0 = TILT_2;
			} 	
			else if (majuArahKanan == true){
				arahX = 195;
				TILT_0 = TILT_2;
			}	
			else{
				arahX = 180;
				TILT_0 = TILT_2;
			} 							
				
			centerX	= arahX;
			centerY = 140;
			
			offsetX = (posX - centerX);
			offsetY = (posY - centerY);

			offsetX *= -1; 		// Inverse X-axis, Y-axis
			offsetY *= -1;
			
			offsetX *= 0.14375; // Pixel per angle
			offsetY *= 0.24167; // Pixel per angle
			Head::GetInstance()->MoveTracking(offsetX, offsetY);
		}
		//cout<<"ballx :"<<offsetX<<endl;
		//cout<<"bally :"<<offsetY<<endl;
		if(DEBUG_PRINT == true) fprintf(stderr, "\r\r");

		switch(StatusCheck::m_cur_mode){
		case CYAN:
			if(buf[9]== Initial ){
				if( initial == true && ready == false && set1 == false && play == false){
					cout<<"Initial CYAN"<<endl;
					Walking::GetInstance()->Stop();
					Head::GetInstance()->MoveToHome();
					jalanCyan = true;
				}
			}
			else if(buf[9]== Ready){
				if( initial == false && ready == true && set1 == false && play == false){
					cout<<"Ready CYAN"<<endl;
					Walking::GetInstance()->Stop();
					Head::GetInstance()->MoveToHome();
					usleep(1000000);
					if(jalanCyan == true)
					{
						JalanCyanAsli();
						jalanCyan = false;
					}
					Walking::GetInstance()->Stop();
				}
			}
			else if(buf[9]== Set){
				if( initial == false && ready == false && set1 == true && play == false){
					jalanCyan = true;
					cout<<"Set CYAN"<<endl;
					Walking::GetInstance()->Stop();
					//Head::GetInstance()->MoveToHome();
				}
			}
			else if(buf[9] == Play){
				if( initial == false && ready == false && set1 == false && play == true){
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					if(jalanCyan == true){
						//while(1){
						//putarkepalaMagen();
						//i += 1;
						//if(i >= 5){
						//	i = 0;
						//	break;
						//	}
						//}
						jalantempat();
						usleep(8000000);
						jalantempat2();
						usleep(800000);
						jalantempat22();
						usleep(800000);
						jalanCyan = false;
						Head::GetInstance()->MoveToHome();
					}
					if(posX == -1.0 || posY == -1.0){
				KickBall 	= 0;
				move 		= 0;
				if(m_NoBallCount > m_NoBallMaxCount){
					// can not find a ball
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;
					Head::GetInstance()->InitTracking(Start_Search);
					int Putar = Head::GetInstance()->GetPutar();
					//cout<<"Putar	:"<<Putar<<endl;
					if(Putar == 0){
						if(Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
						else{
							if (searchMuter == true){
								if (mag >= (leftLimit + 50) || mag <= (rightLimit - 50)) searchMuter = false;
								else{
									if(mag < (leftLimit + 50) && mag > Behind){
										Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
										Walking::GetInstance()->A_MOVE_AMPLITUDE = -15;
										}
									else{
										Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
										Walking::GetInstance()->A_MOVE_AMPLITUDE = 15;
										}
									}
							} else {
								Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
							}
						Walking::GetInstance()->HIP_PITCH_OFFSET = 30; //28.1
						Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
						}
					}
					else if(Putar == 1)//kiri
					{
						Walking::GetInstance()->HIP_PITCH_OFFSET = 30; //28.1
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
						if(Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
					}
					else if(Putar == 2)//kanan
					{
						Walking::GetInstance()->HIP_PITCH_OFFSET = 30; //28.1
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
						if(Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
					}
				
					if(DEBUG_PRINT == true) fprintf(stderr, "[NO BALL]");
				
				}
				else{
					if		(offsetX>0 && offsetY>0) 	start_search = 1;	//kiri atas
					else if (offsetX<0 && offsetY>0) 	start_search = 2;	//kanan atas
					else if (offsetX<0 && offsetY<0) 	start_search = 6;	//kanan bawah
					else if (offsetX>0 && offsetY<0) 	start_search = 5;	//kiri bawah
					else								start_search = 1;
					
					//Head::GetInstance()->SetStartSearch(Start_Search);
					Head::GetInstance()->Start=true;
					m_NoBallCount++;
					if(DEBUG_PRINT == true) fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
				}
					
			}
			else{
				Last_Pos=ball_pos;
				
				Head::GetInstance()->putar 	= 0;
				m_NoBallCount 				= 0;	

				double pan 			= MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
				double pan_range 	= Head::GetInstance()->GetLeftLimitAngle();
				double pan_percent 	= pan / pan_range;
				
				double tilt 		= MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
				double tilt_min 	= Head::GetInstance()->GetBottomLimitAngle();		
				double tilt_range 	= Head::GetInstance()->GetTopLimitAngle() - tilt_min;
				double tilt_percent = (tilt - tilt_min) / tilt_range;
				if(tilt_percent < 0) tilt_percent = -tilt_percent;
				//cout<<"-------------------------------------------"<<endl;
				//cout<<"tilt :"<<tilt<<endl;
				//cout<<"-------------------------------------------"<<endl;
				//cout<<"pan	:"<<pan<<endl;

				/*Syarat robot lurus kearah gawang*/
				if ((mag >= leftLimit) && (mag <= rightLimit)) isGoal = true; 
				else isGoal = false;
				
				if(pan > m_KickRightAngle && pan < m_KickLeftAngle){
					if((mag <= leftLimit) && (mag > Behind)) {
						majuArahKanan	= true;
						tendangSKanan 	= true;
						majuArahKiri 	= false;
						tendangSKiri 	= false;
						majuLurus 		= false;
					}
					else if((mag >= rightLimit) && (mag <= Behind))	{
						majuArahKiri 	= true;
						tendangSKiri 	= true;
						majuArahKanan 	= false;
						tendangSKanan 	= false;
						majuLurus 		= false;
					}
					else if((mag > leftLimit) || (mag < rightLimit)){
						majuLurus 		= true;
						majuArahKanan 	= false;
						tendangSKanan 	= false;
						majuArahKiri 	= false;
						tendangSKiri 	= false;
					}
					else{
						majuArahKanan 	= false;
						majuArahKiri 	= false;
						tendangSKanan 	= false;
						tendangSKiri 	= false;
						majuLurus 		= true;
					}

					if(tilt_percent <= 0.76){
						waitcount += 1;
						cekSend1 	= false;
						if(cekSend2 == false){
							shouldSend 		= true;
							numberToSend 	= STAT_ROBOT3;
							cekSend2 		= true;
						}
					}else{
						waitcount = 0;
						cekSend2 	= false;
						if(cekSend1 == false){
							shouldSend 		= true;
							numberToSend 	= STAT_NETRAL;
							cekSend1 		= true;
						}
					}
							
					if (tilt <= -11){
						if(waitcount >= countmax){
							if(m_KickBallCount >= 0)
							{
								m_FBStep = 0;
								m_RLTurn = 0;
								if (tendangSKanan == true){
									muterkepalaKanan();
									jalantempat1();
									usleep(630000*2);
									Tendang();
									usleep(500000);
									Action::GetInstance()->Start(LS_KICK); // 108
									while(Action::GetInstance()->IsRunning()) usleep(8*1000);
									searchMuter = true;
									return;
								}
								else if (tendangSKiri == true){
									muterkepalaKiri();
									jalantempat1();
									usleep(630000*2);
									Tendang();
									usleep(500000);
									Action::GetInstance()->Start(RS_KICK); // 108
									while(Action::GetInstance()->IsRunning()) usleep(8*1000);
									searchMuter = true;
									return;
								}
								else{
									jalantempat1();
									usleep(630000*2);
									Tendang();
									usleep(500000);
									Action::GetInstance()->Start(R_KICK); // 108
									while(Action::GetInstance()->IsRunning()) usleep(8*1000);
									return;
								}
								if(DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
								detekbal 		= 0;
							}
							else{
								KickBall 		= 0;
								if(DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
							}
						}
					}
					else{
						geser 				= false;
						m_KickBallCount 	= 0;
						KickBall 			= 0;
						m_GoalFBStep 		= m_FollowMaxFBStep * tilt_percent;
						if(m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
						m_GoalRLTurn 		= m_FollowMaxRLTurn * pan_percent * 1.5;//1.5
						if(DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
						detekbal			= 0;
						angkat				= 0;
					}
				}
				else{
					geser 			= false;
					m_KickBallCount = 0;
					KickBall 		= 0;
					m_GoalFBStep 	= 0;
					m_GoalRLTurn 	= m_FollowMaxRLTurn * pan_percent * 1.5;//1.5
					if(DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
					detekbal		= 0;

				}	
				if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0){
					if(Head::GetInstance()->GetPutar()<1)
					{
						if(Walking::GetInstance()->IsRunning() == true) Walking::GetInstance()->Stop();
						else
						{
							if(m_KickBallCount < m_KickBallMaxCount) m_KickBallCount++;
						}
						if(DEBUG_PRINT == true) fprintf(stderr, " STOP");
					}
				}
				else{
					if(DEBUG_PRINT == true) fprintf(stderr, " START");
					if(Walking::GetInstance()->IsRunning() == false)
					{
						m_FBStep 		= 0;
						m_RLTurn 		= 0;
						m_KickBallCount = 0;
						KickBall 		= 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
						Walking::GetInstance()->Start();			
					}
					else
					{
						if(detekbal == 0)
						{
						  //cout << "LURUUUUS"<<endl;
						  if((FB_move > 10) || (m_RLTurn != 3)){
							Walking::GetInstance()->HIP_PITCH_OFFSET 	= 33; // bisa 35.5
							Walking::GetInstance()->Z_MOVE_AMPLITUDE 	= 30;
							
						  }
						  else{
							Walking::GetInstance()->HIP_PITCH_OFFSET 	= 31; //28.1
							Walking::GetInstance()->Z_MOVE_AMPLITUDE 	= 30;
						  }
							//Walking::GetInstance()->HIP_PITCH_OFFSET 	= 33; //28.1
							//Walking::GetInstance()->Z_MOVE_AMPLITUDE 	= 30;
						  //Walking::GetInstance()->X_MOVE_AMPLITUDE 	= 5;
							Walking::GetInstance()->A_MOVE_AMPLITUDE 	= 3.0; //0.5
						  //Walking::GetInstance()->Y_OFFSET 			= 59;
							
						if(m_FBStep < m_GoalFBStep) m_FBStep += m_UnitFBStep;
						else if(m_FBStep > m_GoalFBStep) m_FBStep = m_GoalFBStep;
							
						if( (m_RLTurn >= (m_FollowMaxRLTurn-5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn+5) && m_RLTurn < 0))
						{
							FB_move= m_FBStep - 1;
						}
						else FB_move= m_FBStep;
						
						FB_move += 0.5; // Kalibrasi dari setting walk tunner
						Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
						//cout << "X_Move	:"<<FB_move<<endl;

						if(m_RLTurn < m_GoalRLTurn) m_RLTurn += m_UnitRLTurn;
						else if(m_RLTurn > m_GoalRLTurn) m_RLTurn -= m_UnitRLTurn;
					
						Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

						if(DEBUG_PRINT == true) fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
						}
					}
				}
				if (waitKey(30) == 27){
							cout<<"keluaaar bang   "<<endl;
							break;
				}
			}			
				}
			}
			else if(buf[9] == Finish){
				m_FBStep 	= 0;
				FB_move 	= 0;
				Walking::GetInstance()->Stop();
				Head::GetInstance()->MoveToHome();
			}
			break;
			
		case MAGEN:
			if(buf[9]== Initial ){
				if( initial == true && ready == false && set1 == false && play == false){
					cout<<"Initial Magen"<<endl;
					Walking::GetInstance()->Stop();
					Head::GetInstance()->MoveToHome();
				}
			}
			else if(buf[9]== Ready){
				if( initial == false && ready == true && set1 == false && play == false){
					cout<<"Ready Magen"<<endl;
					Walking::GetInstance()->Stop();
					Head::GetInstance()->MoveToHome();
					if(jalanMagen == true){
						ReadyS2();
						jalanMagen = false;
						puterkepalaMagen = true;
						}
					Walking::GetInstance()->Stop();
				}
			}
			else if(buf[9]== Set){
				if( initial == false && ready == false && set1 == true && play == false){
					cout<<"Set Magen"<<endl;
					Walking::GetInstance()->Stop();
					jalanMagen = true;
				}
			}
			else if(buf[9] == Play){
				if( initial == false && ready == false && set1 == false && play == true){
					MotionManager::GetInstance()->SetEnable(true);
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					cout<<"Play Magen"<<endl;
					searchMuter = true;
					if(puterkepalaMagen == true){
						//while(1){
						//putarkepalaMagen();
						//i += 1;
						//if(i >= 5){
						//	i = 0;
						//	break;
						//	}
						//}
						jalantempat();
						usleep(1000000);
						jalantempat2();
						usleep(800000);
						jalantempat22();
						usleep(800000);
						puterkepalaMagen = false;
					}
					
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
										if (searchMuter == true){
											if (mag >= (mid - 20) && mag <= (mid + 20)){
												searchMuter = false;
												}
											else{
												if(mag < (mid - 20) && mag > Behind){
													Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
													Walking::GetInstance()->A_MOVE_AMPLITUDE=-15;
													}
												else{
													Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
													Walking::GetInstance()->A_MOVE_AMPLITUDE=15;
													}
												}
										} else {
											Walking::GetInstance()->X_MOVE_AMPLITUDE=15;
											Walking::GetInstance()->A_MOVE_AMPLITUDE=0;
										}
									Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
									Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
									}
								}
								else if(Putar==1)//kiri
								{
									Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
									Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
									//Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
									if(Walking::GetInstance()->IsRunning() == false) 
									Walking::GetInstance()->Start();
								}
								else if(Putar==2)//kanan
								{
									Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
									Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
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
							if(mag <= (mid - 70) && mag > Behind) {
								majuArahKanan = true;
								tendangSKanan = true;
								majuArahKiri = false;
								tendangSKiri = false;
								majuLurus = false;
							}
							else if(mag >= (mid + 70) || mag <= Behind)	{
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
								
							if (tilt <= -11.5){
								if(m_KickBallCount >= 0)
								{
									m_FBStep = 0;
									m_RLTurn = 0;
									if (tendangSKanan == true){
										//muterkepalaKanan();
										jalantempat1();
										usleep(630000*2);
										Tendang();
										usleep(500000);
										Action::GetInstance()->Start(LS_KICK); // 108
										while(Action::GetInstance()->IsRunning()) usleep(8*1000);
										searchMuter = true;
									}
									else if (tendangSKiri == true){
										//muterkepalaKiri();
										jalantempat1();
										usleep(630000*2);
										Tendang();
										usleep(500000);
										Action::GetInstance()->Start(RS_KICK); // 108
										while(Action::GetInstance()->IsRunning()) usleep(8*1000);
										searchMuter = true;
									}
									else {
										jalantempat1();
										usleep(630000*2);
										Tendang();
										usleep(500000);
										Action::GetInstance()->Start(R_KICK); // 108
										while(Action::GetInstance()->IsRunning()) usleep(8*1000);
									}
									return;
									if(DEBUG_PRINT == true)
										fprintf(stderr, "[KICK]");
										detekbal = 0;
								}
								else
								{
									KickBall = 0;
									if(DEBUG_PRINT == true)
										fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
								}
							}
							else{
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
					else{
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
								Walking::GetInstance()->HIP_PITCH_OFFSET = 30; //28.1
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
				
			break;
			
		case MANUAL:
			if(jalanManual == true){
				//ReadyS3();
				jalantempat();
				usleep(1000000);
				jalantempat2();
				usleep(800000);
				jalantempat22();
				usleep(800000);
				jalanManual = false;
			}
			MotionManager::GetInstance()->SetEnable(true);
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			searchMuter = true;
			if(posX == -1.0 || posY == -1.0){
				KickBall 	= 0;
				move 		= 0;
				if(m_NoBallCount > m_NoBallMaxCount){
					// can not find a ball
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;
					Head::GetInstance()->InitTracking(Start_Search);
					int Putar = Head::GetInstance()->GetPutar();
					//cout<<"Putar	:"<<Putar<<endl;
					if(Putar == 0){
						if(Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
						else{
							if (searchMuter == true){
								if (mag >= (leftLimit + 50) || mag <= (rightLimit - 50)) searchMuter = false;
								else{
									if(mag < (leftLimit + 50) && mag > Behind){
										Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
										Walking::GetInstance()->A_MOVE_AMPLITUDE = -15;
										}
									else{
										Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
										Walking::GetInstance()->A_MOVE_AMPLITUDE = 15;
										}
									}
							} else {
								Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
							}
						Walking::GetInstance()->HIP_PITCH_OFFSET = 30; //28.1
						Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
						}
					}
					else if(Putar == 1)//kiri
					{
						Walking::GetInstance()->HIP_PITCH_OFFSET = 30; //28.1
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
						if(Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
					}
					else if(Putar == 2)//kanan
					{
						Walking::GetInstance()->HIP_PITCH_OFFSET = 30; //28.1
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
						if(Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
					}
				
					if(DEBUG_PRINT == true) fprintf(stderr, "[NO BALL]");
				
				}
				else{
					if		(offsetX>0 && offsetY>0) 	start_search = 1;	//kiri atas
					else if (offsetX<0 && offsetY>0) 	start_search = 2;	//kanan atas
					else if (offsetX<0 && offsetY<0) 	start_search = 6;	//kanan bawah
					else if (offsetX>0 && offsetY<0) 	start_search = 5;	//kiri bawah
					else								start_search = 1;
					
					//Head::GetInstance()->SetStartSearch(Start_Search);
					Head::GetInstance()->Start=true;
					m_NoBallCount++;
					if(DEBUG_PRINT == true) fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
				}
					
			}
			else{
				Last_Pos=ball_pos;
				
				Head::GetInstance()->putar 	= 0;
				m_NoBallCount 				= 0;	

				double pan 			= MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
				double pan_range 	= Head::GetInstance()->GetLeftLimitAngle();
				double pan_percent 	= pan / pan_range;
				
				double tilt 		= MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
				double tilt_min 	= Head::GetInstance()->GetBottomLimitAngle();		
				double tilt_range 	= Head::GetInstance()->GetTopLimitAngle() - tilt_min;
				double tilt_percent = (tilt - tilt_min) / tilt_range;
				if(tilt_percent < 0) tilt_percent = -tilt_percent;
				//cout<<"-------------------------------------------"<<endl;
				//cout<<"tilt :"<<tilt<<endl;
				//cout<<"-------------------------------------------"<<endl;
				//cout<<"pan	:"<<pan<<endl;

				/*Syarat robot lurus kearah gawang*/
				if ((mag >= leftLimit) && (mag <= rightLimit)) isGoal = true; 
				else isGoal = false;
				
				if(pan > m_KickRightAngle && pan < m_KickLeftAngle){
					if((mag <= leftLimit) && (mag > Behind)) {
						majuArahKanan	= true;
						tendangSKanan 	= true;
						majuArahKiri 	= false;
						tendangSKiri 	= false;
						majuLurus 		= false;
					}
					else if((mag >= rightLimit) && (mag <= Behind))	{
						majuArahKiri 	= true;
						tendangSKiri 	= true;
						majuArahKanan 	= false;
						tendangSKanan 	= false;
						majuLurus 		= false;
					}
					else if((mag > leftLimit) || (mag < rightLimit)){
						majuLurus 		= true;
						majuArahKanan 	= false;
						tendangSKanan 	= false;
						majuArahKiri 	= false;
						tendangSKiri 	= false;
					}
					else{
						majuArahKanan 	= false;
						majuArahKiri 	= false;
						tendangSKanan 	= false;
						tendangSKiri 	= false;
						majuLurus 		= true;
					}

					if(tilt_percent <= 0.76){
						waitcount += 1;
						cekSend1 	= false;
						if(cekSend2 == false){
							shouldSend 		= true;
							numberToSend 	= STAT_ROBOT3;
							cekSend2 		= true;
						}
					}else{
						waitcount = 0;
						cekSend2 	= false;
						if(cekSend1 == false){
							shouldSend 		= true;
							numberToSend 	= STAT_NETRAL;
							cekSend1 		= true;
						}
					}
							
					if (tilt <= TILT_0){
						if(waitcount >= countmax){
							if(m_KickBallCount >= 0)
							{
								m_FBStep = 0;
								m_RLTurn = 0;
								if (tendangSKanan == true){
									//muterkepalaKanan();
									jalantempat1();
									usleep(630000*2);
									Tendang();
									usleep(500000);
									Action::GetInstance()->Start(LS_KICK); // 108
									while(Action::GetInstance()->IsRunning()) usleep(8*1000);
									searchMuter = true;
									return;
								}
								else if (tendangSKiri == true){
									//muterkepalaKiri();
									jalantempat1();
									usleep(630000*2);
									Tendang();
									usleep(500000);
									Action::GetInstance()->Start(RS_KICK); // 108
									while(Action::GetInstance()->IsRunning()) usleep(8*1000);
									searchMuter = true;
									return;
								}
								else{
									jalantempat1();
									usleep(630000*2);
									Tendang();
									usleep(500000);
									Action::GetInstance()->Start(R_KICK); // 108
									while(Action::GetInstance()->IsRunning()) usleep(8*1000);
									return;
								}
								if(DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
								detekbal 		= 0;
							}
							else{
								KickBall 		= 0;
								if(DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
							}
						}
					}
					else{
						geser 				= false;
						m_KickBallCount 	= 0;
						KickBall 			= 0;
						m_GoalFBStep 		= m_FollowMaxFBStep * tilt_percent;
						if(m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
						m_GoalRLTurn 		= m_FollowMaxRLTurn * pan_percent * 1.4;//1.5
						if(DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
						detekbal			= 0;
						angkat				= 0;
					}
				}
				else{
					geser 			= false;
					m_KickBallCount = 0;
					KickBall 		= 0;
					m_GoalFBStep 	= 0;
					m_GoalRLTurn 	= m_FollowMaxRLTurn * pan_percent * 1.4;//1.5
					if(DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
					detekbal		= 0;

				}	
				if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0){
					if(Head::GetInstance()->GetPutar()<1)
					{
						if(Walking::GetInstance()->IsRunning() == true) Walking::GetInstance()->Stop();
						else
						{
							if(m_KickBallCount < m_KickBallMaxCount) m_KickBallCount++;
						}
						if(DEBUG_PRINT == true) fprintf(stderr, " STOP");
					}
				}
				else{
					if(DEBUG_PRINT == true) fprintf(stderr, " START");
					if(Walking::GetInstance()->IsRunning() == false)
					{
						m_FBStep 		= 0;
						m_RLTurn 		= 0;
						m_KickBallCount = 0;
						KickBall 		= 0;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
						Walking::GetInstance()->Start();			
					}
					else
					{
						if(detekbal == 0)
						{
						  //cout << "LURUUUUS"<<endl;
						  if((FB_move > 10) || (m_RLTurn != 3)){
							Walking::GetInstance()->HIP_PITCH_OFFSET 	= 33; // bisa 35.5
							Walking::GetInstance()->Z_MOVE_AMPLITUDE 	= 30;
							
						  }
						  else{
							Walking::GetInstance()->HIP_PITCH_OFFSET 	= 31; //28.1
							Walking::GetInstance()->Z_MOVE_AMPLITUDE 	= 30;
						  }
							//Walking::GetInstance()->HIP_PITCH_OFFSET 	= 33; //28.1
							//Walking::GetInstance()->Z_MOVE_AMPLITUDE 	= 30;
						  //Walking::GetInstance()->X_MOVE_AMPLITUDE 	= 5;
							Walking::GetInstance()->A_MOVE_AMPLITUDE 	= 0.5; //3.5
						  //Walking::GetInstance()->Y_OFFSET 			= 59;
							
						if(m_FBStep < m_GoalFBStep) m_FBStep += m_UnitFBStep;
						else if(m_FBStep > m_GoalFBStep) m_FBStep = m_GoalFBStep;
							
						if( (m_RLTurn >= (m_FollowMaxRLTurn-5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn+5) && m_RLTurn < 0))
						{
							FB_move= m_FBStep - 1;
						}
						else FB_move= m_FBStep;
						
						FB_move += 0.5; // Kalibrasi dari setting walk tunner
						Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
						//cout << "X_Move	:"<<FB_move<<endl;

						if(m_RLTurn < m_GoalRLTurn) m_RLTurn += m_UnitRLTurn;
						else if(m_RLTurn > m_GoalRLTurn) m_RLTurn -= m_UnitRLTurn;
					
						Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

						if(DEBUG_PRINT == true) fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
						}
					}
				}
				if (waitKey(30) == 27){
							cout<<"keluaaar bang   "<<endl;
							break;
				}
			}			
		break;
		}
	}
}