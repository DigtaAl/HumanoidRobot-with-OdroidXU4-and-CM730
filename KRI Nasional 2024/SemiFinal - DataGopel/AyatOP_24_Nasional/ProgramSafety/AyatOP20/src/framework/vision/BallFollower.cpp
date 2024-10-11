/*

     Author:
     Editor : Digta

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
bool jalanCyan 		  	= true;
bool jalanMagen 		  = true;
bool puterkepalaMagen = true;
bool prepmuter	 	  	= false;
bool muterkr	 	    	= false;
bool  deteksi    	    = false;
double TILT_0;
double TILT_1        = -2;
double TILT_2        = -19;
double PAN_MID       = 0.2;           // (Makin besar kiri)
double commDistance  = 0.8;           // (Makin besar jauh)
int ballmidPosition  = 173;
int ballleftPosition = 135;
int ballrightPosition= 195;
int countmax         = 8;
int waitcount;

struct  termios SerialPortSettings;

/* Arah Kompass */
bool  isChecked   = false;
bool  isGoal    = false;

int mid     	  = 0;                  // Nilai tengah 0
int Behind    	= mid + 180;          // Batas akhir arah kanan 180
int leftLimit   = mid - 20;           // Batas awal arah lurus 290
int limitLeft   = mid + 1;            // Batas putaran kiri 359
int rightLimit  = mid + 20;           // Batas akhir arah lurus 70
int limitRight  = mid - 1;            // Batas putaran kanan 1
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

BallTracker::~BallTracker(){
}

BallFollower::BallFollower()
{
  m_NoBallMaxCount    = 10;
  m_NoBallCount    	  = m_NoBallMaxCount;
  m_KickBallMaxCount  = 5;
  m_KickBallCount     = 0;

  m_KickTopAngle      = -5.0;
  m_KickRightAngle    = -35.0;         // awal -30
  m_KickLeftAngle     = 35.0;          // awal 30

  m_FollowMaxFBStep   = 25.0;          // bisa 30
  m_FollowMinFBStep   = 11.5;
  m_FollowMaxRLTurn   = 30.0;          // awal 20
  m_FitFBStep         = 3.0;
  m_FitMaxRLTurn      = 13.0;
  m_UnitFBStep        = 0.5;           // awal 0.3
  m_UnitRLTurn        = 6;

  m_GoalFBStep        = 0;
  m_GoalRLTurn        = 0;
  m_FBStep       	    = 0;
  m_RLTurn        	  = 0;
  DEBUG_PRINT         = false;
  KickBall        	  = 0;

  yaw_tolerance       = 10;
}

BallFollower::~BallFollower(){}

Point2D Last_Pos;
Point2D ball_pos;

bool  geser;
int   a, b, c;
int   angkat      	= 0;
int   detekbal    	= 0;
int   Start_Search  = 1;
int   start_search  = 1;
double  FB_move     = 0;

LinuxCM730 linux_cm730(U2D_DEV_CM);
CM730 cm730(&linux_cm730);

void KoreksiArah() {
  if (Behind > 360 )      Behind = Behind - 360;
  if (leftLimit < 0 )     leftLimit = leftLimit + 360;
  if (limitLeft < 0 )     limitLeft = limitLeft + 360;
  if (rightLimit > 360 )  rightLimit = rightLimit - 360;
  if (limitRight > 360 )  limitRight = limitRight - 360;
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

  const int FRAME_WIDTH   = 320;
  const int FRAME_HEIGHT  = 240;

  int lowH        	= 35; // Set Hue
  int highH       	= 100;

  int lowS        	= 20; // Set Saturation
  int highS       	= 255;

  int lowV        	= 10; // Set Value
  int highV       	= 255;

  int erotion_size  = 1;
  int dilation_size = 0;
  int dilasi      	= 4;
  //-----------------------------------------------------

  int lowH1       	= 0;  // Set Hue
  int highH1      	= 15;

  int lowS1       	= 255; // Set Saturation
  int highS1      	= 255;

  int lowV1       	= 10; //24;   // Set Value
  int highV1      	= 255;

  int erotion_size1  = 0;
  int dilation_size1 = 0;
  //-----------------------------------------------------

  VideoCapture cap(0);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

  //cv::namedWindow("Setting Bola", CV_WINDOW_AUTOSIZE);
  //cv::namedWindow("Setting Lapangan", CV_WINDOW_AUTOSIZE);
  //cv::namedWindow("Lapangan", CV_WINDOW_AUTOSIZE);
  //cv::moveWindow("Lapangan", 0, 0);

  while (true) {
    StatusCheck::Check(cm730);

    deteksi = false;
    cap >> imgOriginal;
    Action::GetInstance()->LoadFile(const_cast<char*>(MOTION_FILE_PATH));

    cv::Mat gambarkopi = imgOriginal.clone();
    cv::Mat Brighnessdec50;
    imgOriginal.convertTo(Brighnessdec50, -1, 0.5, -10);
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
    morphologyEx(lapangan, lapangan, 2, element);

    //Morphologi Closing
    Mat element1 = getStructuringElement(cv::MORPH_RECT,
                                         cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                         cv::Point(dilation_size, dilation_size) );
    //dilate(bola,bola,element);
    morphologyEx(lapangan, lapangan, 3, element1);

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
      if (a > largest_area)
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
    imgOriginal.copyTo(masking, lapangan2);

    //------------------------------------------------------------------------------------------------Fill Hole
    Mat im_floodfill = lapangan2.clone();
    floodFill(im_floodfill, cv::Point(0, 0), Scalar(255));
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
    dilate(im_out2, im_out2, element69);
    Mat masking3 = Mat::zeros(im_out2.rows, im_out2.cols, CV_8UC1);
    //masking.setTo(Scalar(0,0,0));
    imgOriginal.copyTo(masking3, im_out2);
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
    morphologyEx(bola, bola, 2, element_bola);

    //Morphologi closing
    Mat element_bola1 = getStructuringElement(cv::MORPH_ELLIPSE,
                        cv::Size(2 * dilation_size1 + 1, 2 * dilation_size1 + 1),
                        cv::Point(dilation_size1, dilation_size1) );
    //dilate(bola,bola,element);
    morphologyEx(bola, bola, 3, element_bola1);

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
    for (size_t i = 0; i < contours1.size(); i++) // Iterate through each contour.
    {
      double a = contourArea(contours1[i], false);  //  Find the area of contour
      if (a > largest_area1)
      {
        largest_area1 = a;
        largest_contour_index1 = i;                //Store the index of largest contour
        bounding_rect = boundingRect(contours1[i]); // Find the bounding rectangle for biggest contour
        deteksi = true;
        posX = mu1[i].m10 / mu1[i].m00;
        posY = mu1[i].m01 / mu1[i].m00;
      }
      //areabola = contourArea(contours1[0]);
      //cout <<"Area : "<<a<<endl;
    }

    //Search for Largest Contour has End
    Mat gabung;
    cv::hconcat(imgOriginal, masking3, gabung);
    if (contours1.size() > 0)
    {
      rectangle(gabung, bounding_rect,  Scalar(0, 255, 0), 2, 8, 0);
    }
    //cout<<"bola x : "<<posX<<endl;
    //cout<<"bola y : "<<posY<<endl;
    //cv::imshow("Setting Lapangan", lapangan);
    //cv::imshow("Setting Bola", bola);
    //cv::imshow("Lapangan", gabung);

    StatusCheck::Check(cm730);

    if (deteksi == false) {
      waitcount = 0;
      if(receivedNumber == STAT_ROBOT3){
        cekSend2 = false;
        if (cekSend1 == false) {
          shouldSend    = true;
          numberToSend  = STAT_NETRAL;
          cekSend1    = true;
        }
      }
      else if(receivedNumber == 32){
        std::cout << " Robot 3 majuuuu tidak deteksi" << endl;
        Head::GetInstance()->MoveByAngle(85,-10);
        Walking::GetInstance()->X_MOVE_AMPLITUDE = 13;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
        m_RLTurn  = 0;
      }
      else if (receivedNumber == 33){
        Head::GetInstance()->MoveByAngle(85,-10);
        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = 13;
      }
      else if (receivedNumber == 34){
        Head::GetInstance()->MoveByAngle(85,-10);
        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = -13;
      }
      posX = -1;
      posY = -1;
      if (NoBallCount > NoBallMaxCount) {
        if (offsetX > 0 && offsetY > 0)
        {
          start_search = 1; //kiri atas
          Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        }
        else if (offsetX < 0 && offsetY > 0)
        {
          start_search = 2; //kanan atas
          Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        }
        else if (offsetX < 0 && offsetY < 0)
        {
          start_search = 6; //kanan bawah
          Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        }
        else if (offsetX > 0 && offsetY < 0)
        {
          start_search = 5; //kiri bawah
          Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        }
        else
        {
          start_search = 1;
          Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        }
        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        Head::GetInstance()->Start = true;
        Head::GetInstance()->MoveTracking();
        NoBallCount++;
      }
      else
      {
        Head::GetInstance()->InitTracking(start_search);
      }
    }
    else {
      if ((receivedNumber == STAT_ROBOT1) || (receivedNumber == STAT_ROBOT2) || (receivedNumber == STAT_ROBOT4)) {
        std::cout << " Robot 3 tidak maju " << endl;
        m_FBStep  = 0;
        FB_move   = 0;
        StatusCheck::Check(cm730);
      }
      else if (receivedNumber == STAT_KICKOFF) {
        std::cout << " Robot 3 tunggu kickoff " << endl;
        m_FBStep  = 0;
        FB_move   = 0;
        m_RLTurn  = 0;
        StatusCheck::Check(cm730);
      }
      else if(receivedNumber == 32){
        std::cout << " Robot 3 majuuuu deteksi" << endl;
        Head::GetInstance()->MoveByAngle(-85,-10);
        Walking::GetInstance()->X_MOVE_AMPLITUDE = 13;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
      }
      else if (receivedNumber == 33){
        Head::GetInstance()->MoveByAngle(-85,-10);
        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = 13;
      }
      else if (receivedNumber == 34){
        Head::GetInstance()->MoveByAngle(-85,-10);
        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = -13;
      }
      
      NoBallCount = 0;
      if (VoidAction::majuLurus == true) {
        arahX = ballmidPosition;
        TILT_0 = TILT_2;
      }
      else if (VoidAction::majuArahKiri == true) {
        arahX = ballleftPosition;
        TILT_0 = TILT_2;
      }
      else if (VoidAction::majuArahKanan == true) {
        arahX = ballrightPosition;
        TILT_0 = TILT_2;
      }
      else {
        arahX = ballmidPosition;
        TILT_0 = TILT_2;
      }

      centerX = arahX;
      centerY = 140;

      offsetX = (posX - centerX);
      offsetY = (posY - centerY);

      offsetX *= -1;    // Inverse X-axis, Y-axis
      offsetY *= -1;

      offsetX *= 0.14375; // Pixel per angle
      offsetY *= 0.24167; // Pixel per angle
      Head::GetInstance()->MoveTracking(offsetX, offsetY);
    }
    //cout<<"ballx :"<<offsetX<<endl;
    //cout<<"bally :"<<offsetY<<endl;

    if (DEBUG_PRINT == true) fprintf(stderr, "\r\r");

    switch (StatusCheck::m_cur_mode) {
      case CYAN:
        CyanGameControl();
      break;

      case MAGEN:
        MagenGameControl();
      break;

      case CYAN2:
        Cyan2GameControl();
      break;

      case CYAN3:
        Cyan3GameControl();
      break;

      case MANUAL:
        ManualProses();
      break;
    }

    if (waitKey(30) == 27) {
      cout << "keluaaar bang   " << endl;
      break;
    }

  }
}


void BallFollower::CyanGameControl(){
  if (buf[9] == Initial ) {
    if ( initial == true && ready == false && set1 == false && play == false) {
      cout << "Initial CYAN" << endl;
      Walking::GetInstance()->Stop();
      jalanCyan = true;
    }
  }
  else if (buf[9] == Ready) {
    if ( initial == false && ready == true && set1 == false && play == false) {
      cout << "Ready CYAN" << endl;
      Walking::GetInstance()->Stop();
      if (StatusCheck::trigready == 1) {
        VoidAction::firstkick_ = 0;
        Head::GetInstance()->MoveToHome();
        //usleep(1000000);
        VoidAction::JalanCyanluruskiri();
        StatusCheck::trigready = 0;
      }
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Set) {
    if ( initial == false && ready == false && set1 == true && play == false) {
      jalanCyan = true;
      cout << "Set CYAN" << endl;
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Play) {
    if ( initial == false && ready == false && set1 == false && play == true) {
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
      if (jalanCyan == true) {
        VoidAction::jalantempat();
        usleep(2000000);
        VoidAction::jalantempat2();
        usleep(800000);
        VoidAction::jalantempat22();
        usleep(800000);
        jalanCyan = false;
        Head::GetInstance()->MoveToHome();
      }
      if (VoidAction::prepwalking == true) {
        VoidAction::prepjalan();
        Head::GetInstance()->MoveToHome();
        VoidAction::prepwalking = false;
      }
      if (posX == -1.0 || posY == -1.0) {
        KickBall  = 0;
        move    	= 0;
        if (m_NoBallCount > m_NoBallMaxCount) {
          // can not find a ball
          m_GoalFBStep = 0;
          m_GoalRLTurn = 0;
          Head::GetInstance()->InitTracking(Start_Search);
          int Putar = Head::GetInstance()->GetPutar();
          //cout<<"Putar  :"<<Putar<<endl;
          if (Putar == 0) {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, -3, 8, 0);
          }
          else if (Putar == 1) //kiri
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, -3, 8, 0);
          }
          else if (Putar == 2) //kanan
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, -3, 8, 0);
          }
          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL]");

        }
        else {
          if    (offsetX > 0 && offsetY > 0)    start_search = 1; //kiri atas
          else if (offsetX < 0 && offsetY > 0)  start_search = 2; //kanan atas
          else if (offsetX < 0 && offsetY < 0)  start_search = 6; //kanan bawah
          else if (offsetX > 0 && offsetY < 0)  start_search = 5; //kiri bawah
          else                  				        start_search = 1;

          //Head::GetInstance()->SetStartSearch(Start_Search);
          Head::GetInstance()->Start = true;
          m_NoBallCount++;
          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
        }

      }
      else {
        Last_Pos = ball_pos;

        Head::GetInstance()->putar  = 0;
        m_NoBallCount         = 0;

        double pan      	  = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double pan_range    = Head::GetInstance()->GetLeftLimitAngle();
        double pan_percent  = pan / pan_range;

        double tilt         = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        double tilt_min     = Head::GetInstance()->GetBottomLimitAngle();
        double tilt_range   = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        double tilt_percent = (tilt - tilt_min) / tilt_range;
        if (tilt_percent < 0) tilt_percent = -tilt_percent;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"tilt :"<<tilt<<endl;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"pan  :"<<pan<<endl;

        if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
          VoidAction::majuLurus       = true;
          //updateBallPosition(leftLimit,rightLimit,Behind);
        
          if ((receivedNumber == STAT_NETRAL) || (receivedNumber == STAT_ROBOT3)) {
            if (tilt_percent <= commDistance) {
              waitcount += 1;
              cekSend1  = false;
              if (cekSend2 == false) {
                shouldSend    = true;
                numberToSend  = STAT_ROBOT3;
                cekSend2    = true;
              }
            } else {
              waitcount = 0;
              cekSend2  = false;
              if (cekSend1 == false) {
                shouldSend    = true;
                numberToSend  = STAT_NETRAL;
                cekSend1    = true;
              }
            }
          }

          if ((tilt <= TILT_1)) {
            if(VoidAction::firstkick_ == 0){
              if ((mag < 5) || (mag > Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskiri();
                while (1) {
                  //std::cout << "Putarkiri" << std::endl;
                  VoidAction::cobaluruskiri();
                  StatusCheck::Check(cm730);
                  if ((mag >= 10) && (mag < 30)) {
                    VoidAction::revprepluruskiri();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else if ((mag > 70) && (mag < Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskanan();
                while (1) {
                  //std::cout << "Putarkanan" << std::endl;
                  VoidAction::cobaluruskanan();
                  StatusCheck::Check(cm730);
                  if ((mag > 10) && (mag < 50)) {
                    VoidAction::revprepluruskanan();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else {
                if (tilt <= TILT_2) {
                  if (waitcount >= countmax) {
                    if (m_KickBallCount >= 0)
                    {
                      m_FBStep = 0;
                      m_RLTurn = 0;
                      VoidAction::handleKick(pan,PAN_MID);
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
                      cekSend2 = false;
                      if (cekSend1 == false) {
                        shouldSend    = true;
                        numberToSend  = STAT_NETRAL;
                        cekSend1    = true;
                      }
                      detekbal    = 0;
                      return;
                    }
                    else {
                      KickBall    = 0;
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
                    }
                  }
                }
                else {
                  geser       		= false;
                  m_KickBallCount = 0;
                  KickBall      	= 0;
                  m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
                  if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
                  m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
                  if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
                  detekbal      	= 0;
                  angkat        	= 0;
                }
              }
            }
            else if(VoidAction::firstkick_ > 0){
              if ((mag < leftLimit) && (mag > Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskiri();
                while (1) {
                  //std::cout << "Putarkiri" << std::endl;
                  VoidAction::cobaluruskiri();
                  StatusCheck::Check(cm730);
                  if (mag > (leftLimit + 5) || mag < rightLimit) {
                    VoidAction::revprepluruskiri();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else if ((mag > rightLimit) && (mag < Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskanan();
                while (1) {
                  //std::cout << "Putarkanan" << std::endl;
                  VoidAction::cobaluruskanan();
                  StatusCheck::Check(cm730);
                  if (mag > leftLimit || mag < (rightLimit - 5)) {
                    VoidAction::revprepluruskanan();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else {
                if (tilt <= TILT_2) {
                  if (waitcount >= countmax) {
                    if (m_KickBallCount >= 0)
                    {
                      m_FBStep = 0;
                      m_RLTurn = 0;
                      VoidAction::handleKick(pan,PAN_MID);
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
                      cekSend2 = false;
                      if (cekSend1 == false) {
                        shouldSend    = true;
                        numberToSend  = STAT_NETRAL;
                        cekSend1    = true;
                      }
                      detekbal    = 0;
                      return;
                    }
                    else {
                      KickBall    = 0;
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
                    }
                  }
                }
                else {
                  geser       		= false;
                  m_KickBallCount = 0;
                  KickBall      	= 0;
                  m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
                  if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
                  m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
                  if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
                  detekbal      	= 0;
                  angkat        	= 0;
                }
              }
            }
          }
          else {
            geser       = false;
            m_KickBallCount   = 0;
            KickBall      	  = 0;
            m_GoalFBStep    	= m_FollowMaxFBStep * tilt_percent;
            if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
            m_GoalRLTurn    	= m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
            if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
            detekbal      	  = 0;
            angkat        	  = 0;
          }
        }
        else {
          geser       	  = false;
          m_KickBallCount = 0;
          KickBall    	  = 0;
          m_GoalFBStep  	= 0;
          m_GoalRLTurn  	= m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5; //1.5
          if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
          detekbal    	  = 0;

        }
        if (m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0) {
          if (Head::GetInstance()->GetPutar() < 1)
          {
            if (Walking::GetInstance()->IsRunning() == true) Walking::GetInstance()->Stop();
            else
            {
              if (m_KickBallCount < m_KickBallMaxCount) m_KickBallCount++;
            }
            if (DEBUG_PRINT == true) fprintf(stderr, " STOP");
          }
        }
        else {
          if (DEBUG_PRINT == true) fprintf(stderr, " START");
          if (Walking::GetInstance()->IsRunning() == false)
          {
            m_FBStep      	= 0;
            m_RLTurn      	= 0;
            m_KickBallCount = 0;
            KickBall      	= 0;
            Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
            Walking::GetInstance()->Start();
          }
          else
          {
            if (detekbal == 0)
            {
              //cout << "LURUUUUS"<<endl;
              if ((FB_move > 10) || (m_RLTurn != -0.5)) {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; // bisa 35.5
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              }
              else {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 31; //28.1
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              }
              Walking::GetInstance()->A_MOVE_AMPLITUDE    = -1; //0.5
              //Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; //28.1
              //Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              //Walking::GetInstance()->X_MOVE_AMPLITUDE  = 5;
              //Walking::GetInstance()->Y_OFFSET      	  = 59;

              if((receivedNumber != 32)){
                if (m_FBStep < m_GoalFBStep) m_FBStep += m_UnitFBStep;
                else if (m_FBStep > m_GoalFBStep) m_FBStep = m_GoalFBStep;
              }

              if ( (m_RLTurn >= (m_FollowMaxRLTurn - 5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn + 5) && m_RLTurn < 0))
              {
                FB_move = m_FBStep - 1;
              }
              else FB_move = m_FBStep;

              FB_move += 0.5; // Kalibrasi dari setting walk tunner
              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34))Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
              //cout << "X_Move :"<<FB_move<<endl;

              if((receivedNumber != STAT_KICKOFF)){
                if (m_RLTurn < m_GoalRLTurn) m_RLTurn += m_UnitRLTurn;
                else if (m_RLTurn > m_GoalRLTurn) m_RLTurn -= m_UnitRLTurn;
              }

              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

              if (DEBUG_PRINT == true) fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
            }
          }
        }
      }
    }
  }
  else if (buf[9] == Finish) {
    m_FBStep  = 0;
    FB_move   = 0;
    Walking::GetInstance()->Stop();
    Head::GetInstance()->MoveToHome();
  }
}

void BallFollower::Cyan2GameControl(){
  if (buf[9] == Initial ) {
    if ( initial == true && ready == false && set1 == false && play == false) {
      cout << "Initial CYAN2" << endl;
      Walking::GetInstance()->Stop();
      jalanCyan = true;
    }
  }
  else if (buf[9] == Ready) {
    if ( initial == false && ready == true && set1 == false && play == false) {
      cout << "Ready CYAN2" << endl;
      Walking::GetInstance()->Stop();
      if (StatusCheck::trigready == 1) {
        VoidAction::firstkick_ = 0;
        Head::GetInstance()->MoveToHome();
        VoidAction::JalanCyanBaru();
        StatusCheck::trigready = 0;
      }
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Set) {
    if ( initial == false && ready == false && set1 == true && play == false) {
      jalanCyan = true;
      cout << "Set CYAN2" << endl;
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Play) {
    if ( initial == false && ready == false && set1 == false && play == true) {
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
      if (jalanCyan == true) {
        VoidAction::jalantempat();
        usleep(2000000);
        VoidAction::jalantempat2();
        usleep(800000);
        VoidAction::jalantempat22();
        usleep(800000);
        jalanCyan = false;
        Head::GetInstance()->MoveToHome();
      }
      if (VoidAction::prepwalking == true) {
        VoidAction::prepjalan();
        Head::GetInstance()->MoveToHome();
        VoidAction::prepwalking = false;
      }
      if (posX == -1.0 || posY == -1.0) {
        KickBall  = 0;
        move    	= 0;
        if (m_NoBallCount > m_NoBallMaxCount) {
          // can not find a ball
          m_GoalFBStep = 0;
          m_GoalRLTurn = 0;
          Head::GetInstance()->InitTracking(Start_Search);
          int Putar = Head::GetInstance()->GetPutar();
          //cout<<"Putar  :"<<Putar<<endl;
          if (Putar == 0) {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, 3, 8, 0);
          }
          else if (Putar == 1) //kiri
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, 3, 8, 0);
          }
          else if (Putar == 2) //kanan
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, 3, 8, 0);
          }

          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL]");

        }
        else {
          if    (offsetX > 0 && offsetY > 0)    start_search = 1; //kiri atas
          else if (offsetX < 0 && offsetY > 0)  start_search = 2; //kanan atas
          else if (offsetX < 0 && offsetY < 0)  start_search = 6; //kanan bawah
          else if (offsetX > 0 && offsetY < 0)  start_search = 5; //kiri bawah
          else                  				        start_search = 1;

          //Head::GetInstance()->SetStartSearch(Start_Search);
          Head::GetInstance()->Start = true;
          m_NoBallCount++;
          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
        }

      }
      else {
        Last_Pos = ball_pos;

        Head::GetInstance()->putar  = 0;
        m_NoBallCount         = 0;

        double pan      	  = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double pan_range    = Head::GetInstance()->GetLeftLimitAngle();
        double pan_percent  = pan / pan_range;

        double tilt         = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        double tilt_min     = Head::GetInstance()->GetBottomLimitAngle();
        double tilt_range   = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        double tilt_percent = (tilt - tilt_min) / tilt_range;
        if (tilt_percent < 0) tilt_percent = -tilt_percent;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"tilt :"<<tilt<<endl;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"pan  :"<<pan<<endl;

        if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
          VoidAction::majuLurus       = true;
          //updateBallPosition(leftLimit,rightLimit,Behind);

          if ((receivedNumber == STAT_NETRAL) || (receivedNumber == STAT_ROBOT3)) {
            if (tilt_percent <= commDistance) {
              waitcount += 1;
              cekSend1  = false;
              if (cekSend2 == false) {
                shouldSend    = true;
                numberToSend  = STAT_ROBOT3;
                cekSend2    = true;
              }
            } else {
              waitcount = 0;
              cekSend2  = false;
              if (cekSend1 == false) {
                shouldSend    = true;
                numberToSend  = STAT_NETRAL;
                cekSend1    = true;
              }
            }
          }

          if ((tilt <= TILT_1)) {
            if(VoidAction::firstkick_ == 0){
              if ((mag > 350) || (mag < Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskanan();
                while (1) {
                  //std::cout << "Putarkanan" << std::endl;
                  VoidAction::cobaluruskanan();
                  StatusCheck::Check(cm730);
                  if ((mag >= 290) && (mag <= 345)) {
                    VoidAction::revprepluruskanan();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else if ((mag < 290) && (mag > Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskiri();
                while (1) {
                  //std::cout << "Putarkanan" << std::endl;
                  VoidAction::cobaluruskiri();
                  StatusCheck::Check(cm730);
                  if ((mag >= 295) && (mag <= 350)) {
                    VoidAction::revprepluruskiri();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else {
                if (tilt <= TILT_2) {
                  if (waitcount >= countmax) {
                    if (m_KickBallCount >= 0)
                    {
                      m_FBStep = 0;
                      m_RLTurn = 0;
                      VoidAction::handleKick(pan,PAN_MID);
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
                      if (cekSend1 == false) {
                        shouldSend    = true;
                        numberToSend  = STAT_NETRAL;
                        cekSend1    = true;
                      }
                      detekbal    = 0;
                      return;
                    }
                    else {
                      KickBall    = 0;
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
                    }
                  }
                }
                else {
                  geser       		= false;
                  m_KickBallCount   = 0;
                  KickBall      	= 0;
                  m_GoalFBStep    	= m_FollowMaxFBStep * tilt_percent;
                  if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
                  m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
                  if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
                  detekbal      	= 0;
                  angkat        	= 0;
                }
              }
            }
            else if(VoidAction::firstkick_ > 0){
              if ((mag < leftLimit) && (mag > Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskiri();
                while (1) {
                  //std::cout << "Putarkiri" << std::endl;
                  VoidAction::cobaluruskiri();
                  StatusCheck::Check(cm730);
                  if (mag > (leftLimit + 5) || mag < rightLimit) {
                    VoidAction::revprepluruskiri();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else if ((mag > rightLimit) && (mag < Behind)) {
                VoidAction::netral();
                usleep(800000);
                VoidAction::prepluruskanan();
                while (1) {
                  //std::cout << "Putarkanan" << std::endl;
                  VoidAction::cobaluruskanan();
                  StatusCheck::Check(cm730);
                  if (mag > leftLimit || mag < (rightLimit - 5)) {
                    VoidAction::revprepluruskanan();
                    VoidAction::netral();
                    usleep(800000);
                    Head::GetInstance()->MoveToHome();
                    waitcount = 0;
                    break;
                  }
                }
              }
              else {
                if (tilt <= TILT_2) {
                  if (waitcount >= countmax) {
                    if (m_KickBallCount >= 0)
                    {
                      m_FBStep = 0;
                      m_RLTurn = 0;
                      VoidAction::handleKick(pan,PAN_MID);
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
                      if (cekSend1 == false) {
                        shouldSend    = true;
                        numberToSend  = STAT_NETRAL;
                        cekSend1    = true;
                      }
                      detekbal    = 0;
                      return;
                    }
                    else {
                      KickBall    = 0;
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
                    }
                  }
                }
                else {
                  geser       		= false;
                  m_KickBallCount = 0;
                  KickBall      	= 0;
                  m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
                  if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
                  m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
                  if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
                  detekbal      	= 0;
                  angkat        	= 0;
                }
              }
            }
          }
          else {
            geser           = false;
            m_KickBallCount = 0;
            KickBall      	= 0;
            m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
            if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
            m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
            if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
            detekbal      	= 0;
            angkat        	= 0;
          }
        }
        else {
          geser       	  = false;
          m_KickBallCount = 0;
          KickBall    	  = 0;
          m_GoalFBStep  	= 0;
          m_GoalRLTurn  	= m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5; //1.5
          if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
          detekbal    	  = 0;

        }
        if (m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0) {
          if (Head::GetInstance()->GetPutar() < 1)
          {
            if (Walking::GetInstance()->IsRunning() == true) Walking::GetInstance()->Stop();
            else
            {
              if (m_KickBallCount < m_KickBallMaxCount) m_KickBallCount++;
            }
            if (DEBUG_PRINT == true) fprintf(stderr, " STOP");
          }
        }
        else {
          if (DEBUG_PRINT == true) fprintf(stderr, " START");
          if (Walking::GetInstance()->IsRunning() == false)
          {
            m_FBStep      	= 0;
            m_RLTurn      	= 0;
            m_KickBallCount = 0;
            KickBall      	= 0;
            Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
            Walking::GetInstance()->Start();
          }
          else
          {
            if (detekbal == 0)
            {
              //cout << "LURUUUUS"<<endl;
              if ((FB_move > 10) || (m_RLTurn != -0.5)) {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; // bisa 35.5
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;

              }
              else {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 31; //28.1
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              }
              Walking::GetInstance()->A_MOVE_AMPLITUDE  = -1; //0.5
              //Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; //28.1
              //Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              //Walking::GetInstance()->X_MOVE_AMPLITUDE  = 5;
              //Walking::GetInstance()->Y_OFFSET      	= 59;

              if((receivedNumber != 32)){
                if (m_FBStep < m_GoalFBStep) m_FBStep += m_UnitFBStep;
                else if (m_FBStep > m_GoalFBStep) m_FBStep = m_GoalFBStep;
              }

              if ( (m_RLTurn >= (m_FollowMaxRLTurn - 5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn + 5) && m_RLTurn < 0))
              {
                FB_move = m_FBStep - 1;
              }
              else FB_move = m_FBStep;

              FB_move += 0.5; // Kalibrasi dari setting walk tunner
              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
              //cout << "X_Move :"<<FB_move<<endl;

              if(receivedNumber != STAT_KICKOFF){
                if (m_RLTurn < m_GoalRLTurn) m_RLTurn += m_UnitRLTurn;
                else if (m_RLTurn > m_GoalRLTurn) m_RLTurn -= m_UnitRLTurn;
              }

              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

              if (DEBUG_PRINT == true) fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
            }
          }
        }
      }
    }
  }
  else if (buf[9] == Finish) {
    m_FBStep  = 0;
    FB_move   = 0;
    Walking::GetInstance()->Stop();
    Head::GetInstance()->MoveToHome();
  }
}

void BallFollower::Cyan3GameControl(){
  if (buf[9] == Initial ) {
    if ( initial == true && ready == false && set1 == false && play == false) {
      cout << "Initial CYAN3" << endl;
      Walking::GetInstance()->Stop();
      jalanCyan = true;
    }
  }
  else if (buf[9] == Ready) {
    if ( initial == false && ready == true && set1 == false && play == false) {
      cout << "Ready CYAN3" << endl;
      Walking::GetInstance()->Stop();
      if (StatusCheck::trigready == 1) {
        VoidAction::firstkick_ = 0;
        Head::GetInstance()->MoveToHome();
        VoidAction::JalanCyanAsli();
        StatusCheck::trigready = 0;
      }
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Set) {
    if ( initial == false && ready == false && set1 == true && play == false) {
      jalanCyan = true;
      cout << "Set CYAN3" << endl;
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Play) {
    if ( initial == false && ready == false && set1 == false && play == true) {
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
      if (VoidAction::prepwalking == true) {
        VoidAction::prepjalan();
        Head::GetInstance()->MoveToHome();
        VoidAction::prepwalking = false;
      }
      if (posX == -1.0 || posY == -1.0) {
        KickBall  = 0;
        move    	= 0;
        if (m_NoBallCount > m_NoBallMaxCount) {
          // can not find a ball
          m_GoalFBStep = 0;
          m_GoalRLTurn = 0;
          Head::GetInstance()->InitTracking(Start_Search);
          int Putar = Head::GetInstance()->GetPutar();
          //cout<<"Putar  :"<<Putar<<endl;
          if (Putar == 0) {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, 0, 8, 0);
          }
          else if (Putar == 1) //kiri
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, 0, 8, 0);
          }
          else if (Putar == 2) //kanan
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(2, 0, 8, 0);
          }

          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL]");

        }
        else {
          if    (offsetX > 0 && offsetY > 0)    start_search = 1; //kiri atas
          else if (offsetX < 0 && offsetY > 0)  start_search = 2; //kanan atas
          else if (offsetX < 0 && offsetY < 0)  start_search = 6; //kanan bawah
          else if (offsetX > 0 && offsetY < 0)  start_search = 5; //kiri bawah
          else                  				        start_search = 1;

          //Head::GetInstance()->SetStartSearch(Start_Search);
          Head::GetInstance()->Start = true;
          m_NoBallCount++;
          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
        }

      }
      else {
        Last_Pos = ball_pos;

        Head::GetInstance()->putar  = 0;
        m_NoBallCount       = 0;

        double pan      	  = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double pan_range    = Head::GetInstance()->GetLeftLimitAngle();
        double pan_percent  = pan / pan_range;

        double tilt         = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        double tilt_min     = Head::GetInstance()->GetBottomLimitAngle();
        double tilt_range   = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        double tilt_percent = (tilt - tilt_min) / tilt_range;
        if (tilt_percent < 0) tilt_percent = -tilt_percent;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"tilt :"<<tilt<<endl;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"pan  :"<<pan<<endl;

        if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
          VoidAction::majuLurus       = true;
          //updateBallPosition(leftLimit,rightLimit,Behind);

          if ((receivedNumber == STAT_NETRAL) || (receivedNumber == STAT_ROBOT3)) {
            if (tilt_percent <= commDistance) {
              waitcount += 1;
              cekSend1  = false;
              if (cekSend2 == false) {
                shouldSend    = true;
                numberToSend  = STAT_ROBOT3;
                cekSend2      = true;
              }
            } else {
              waitcount = 0;
              cekSend2  = false;
              if (cekSend1 == false) {
                shouldSend    = true;
                numberToSend  = STAT_NETRAL;
                cekSend1      = true;
              }
            }
          }

          if ((tilt <= TILT_1)) {
            if ((mag < leftLimit) && (mag > Behind)) {
              VoidAction::netral();
              usleep(800000);
              VoidAction::prepluruskiri();
              while (1) {
                //std::cout << "Putarkiri" << std::endl;
                VoidAction::cobaluruskiri();
                StatusCheck::Check(cm730);
                if (mag > (leftLimit + 5) || mag < rightLimit) {
                  VoidAction::revprepluruskiri();
                  VoidAction::netral();
                  usleep(800000);
                  Head::GetInstance()->MoveToHome();
                  waitcount = 0;
                  break;
                }
              }
            }
            else if ((mag > rightLimit) && (mag < Behind)) {
              VoidAction::netral();
              usleep(800000);
              VoidAction::prepluruskanan();
              while (1) {
                //std::cout << "Putarkanan" << std::endl;
                VoidAction::cobaluruskanan();
                StatusCheck::Check(cm730);
                if (mag > leftLimit || mag < (rightLimit - 5)) {
                  VoidAction::revprepluruskanan();
                  VoidAction::netral();
                  usleep(800000);
                  Head::GetInstance()->MoveToHome();
                  waitcount = 0;
                  break;
                }
              }
            }
            else {
              if (tilt <= TILT_2) {
                if (waitcount >= countmax) {
                  if (m_KickBallCount >= 0){
                    m_FBStep = 0;
                    m_RLTurn = 0;
                    VoidAction::handleKick(pan,PAN_MID);
                    if (DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
                    if (cekSend1 == false) {
                        shouldSend    = true;
                        numberToSend  = STAT_NETRAL;
                        cekSend1    = true;
                      }
                    detekbal    = 0;
                    return;
                  }
                  else {
                    KickBall    = 0;
                    if (DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
                  }
                }
              }
              else {
                geser       		= false;
                m_KickBallCount = 0;
                KickBall      	= 0;
                m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
                if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
                m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
                if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
                detekbal      	= 0;
                angkat        	= 0;
              }
            }
          }
          else {
            geser           = false;
            m_KickBallCount = 0;
            KickBall      	= 0;
            m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
            if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
            m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5;//1.5
            if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
            detekbal      	= 0;
            angkat        	= 0;
          }
        }
        else {
          geser       	  = false;
          m_KickBallCount = 0;
          KickBall    	  = 0;
          m_GoalFBStep  	= 0;
          m_GoalRLTurn  	= m_FollowMaxRLTurn * (pan_percent - 0.0455) * 1.5; //1.5
          if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
          detekbal    	  = 0;

        }
        if (m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0) {
          if (Head::GetInstance()->GetPutar() < 1)
          {
            if (Walking::GetInstance()->IsRunning() == true) Walking::GetInstance()->Stop();
            else
            {
              if (m_KickBallCount < m_KickBallMaxCount) m_KickBallCount++;
            }
            if (DEBUG_PRINT == true) fprintf(stderr, " STOP");
          }
        }
        else {
          if (DEBUG_PRINT == true) fprintf(stderr, " START");
          if (Walking::GetInstance()->IsRunning() == false)
          {
            m_FBStep      	= 0;
            m_RLTurn      	= 0;
            m_KickBallCount = 0;
            KickBall      	= 0;
            Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
            Walking::GetInstance()->Start();
          }
          else
          {
            if (detekbal == 0)
            {
              //cout << "LURUUUUS"<<endl;
              if ((FB_move > 10) || (m_RLTurn != -0.5)) {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; // bisa 35.5
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;

              }
              else {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 31; //28.1
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              }
              Walking::GetInstance()->A_MOVE_AMPLITUDE  = -1; //0.5
              //Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; //28.1
              //Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              //Walking::GetInstance()->X_MOVE_AMPLITUDE  = 5;
              //Walking::GetInstance()->Y_OFFSET      	= 59;

              if((receivedNumber != 32)){
                if (m_FBStep < m_GoalFBStep) m_FBStep += m_UnitFBStep;
                else if (m_FBStep > m_GoalFBStep) m_FBStep = m_GoalFBStep;
              }

              if ( (m_RLTurn >= (m_FollowMaxRLTurn - 5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn + 5) && m_RLTurn < 0))
              {
                FB_move = m_FBStep - 1;
              }
              else FB_move = m_FBStep;

              FB_move += 0.5; // Kalibrasi dari setting walk tunner
              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
              //cout << "X_Move :"<<FB_move<<endl;

              if(receivedNumber != STAT_KICKOFF){
                if (m_RLTurn < m_GoalRLTurn) m_RLTurn += m_UnitRLTurn;
                else if (m_RLTurn > m_GoalRLTurn) m_RLTurn -= m_UnitRLTurn;
              }

              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

              if (DEBUG_PRINT == true) fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
            }
          }
        }
      }
    }
  }
  else if (buf[9] == Finish) {
    m_FBStep  = 0;
    FB_move   = 0;
    Walking::GetInstance()->Stop();
    Head::GetInstance()->MoveToHome();
  }
}

void BallFollower::MagenGameControl(){
  if (buf[9] == Initial ) {
    if ( initial == true && ready == false && set1 == false && play == false) {
      cout << "Initial MAGEN" << endl;
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Ready) {
    if ( initial == false && ready == true && set1 == false && play == false) {
      cout << "Ready Magen" << endl;
      Walking::GetInstance()->Stop();
      if (StatusCheck::trigready == 1) {
        Head::GetInstance()->MoveToHome();
        if (jalanMagen == true) {
          VoidAction::ReadyS2();
          jalanMagen = false;
          puterkepalaMagen = true;
        }
        StatusCheck::trigready = 0;
      }
      Walking::GetInstance()->Stop();
    }
  }
  else if (buf[9] == Set) {
    if ( initial == false && ready == false && set1 == true && play == false) {
      cout << "Set Magen" << endl;
      Walking::GetInstance()->Stop();
      jalanMagen = true;
    }
  }
  else if (buf[9] == Play) {
    if ( initial == false && ready == false && set1 == false && play == true) {
      MotionManager::GetInstance()->SetEnable(true);
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
      cout << "Play Magen" << endl;
      VoidAction::searchMuter = true;
      if (puterkepalaMagen == true) {
        VoidAction::jalantempat();
        usleep(7000000);
        VoidAction::jalantempat2();
        usleep(800000);
        VoidAction::jalantempat22();
        usleep(800000);
        Head::GetInstance()->MoveToHome();
        puterkepalaMagen = false;
      }

      if (VoidAction::prepwalking == true) {
        VoidAction::prepjalan();
        Head::GetInstance()->MoveToHome();
        VoidAction::prepwalking = false;
      }

      if (posX == -1.0 || posY == -1.0) {
        KickBall  = 0;
        move      = 0;
        if (m_NoBallCount > m_NoBallMaxCount) {
          // can not find a ball
          m_GoalFBStep = 0;
          m_GoalRLTurn = 0;
          Head::GetInstance()->InitTracking(Start_Search);
          int Putar = Head::GetInstance()->GetPutar();
          //cout<<"Putar  :"<<Putar<<endl;
          if (Putar == 0) {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(8, 0, 8, 0);
          }
          else if (Putar == 1) //kiri
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(8, 0, 8, 0);
          }
          else if (Putar == 2) //kanan
          {
            if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
            else SearchingWalkControl(8, 0, 8, 0);
          }

          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL]");

        }
        else {
          if    (offsetX > 0 && offsetY > 0)    start_search = 1; //kiri atas
          else if (offsetX < 0 && offsetY > 0)  start_search = 2; //kanan atas
          else if (offsetX < 0 && offsetY < 0)  start_search = 6; //kanan bawah
          else if (offsetX > 0 && offsetY < 0)  start_search = 5; //kiri bawah
          else                  				        start_search = 1;

          Head::GetInstance()->Start = true;
          m_NoBallCount++;
          if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
        }

      }
      else {
        Last_Pos = ball_pos;

        Head::GetInstance()->putar  = 0;
        m_NoBallCount         = 0;

        double pan          = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double pan_range    = Head::GetInstance()->GetLeftLimitAngle();
        double pan_percent  = pan / pan_range;

        double tilt         = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        double tilt_min     = Head::GetInstance()->GetBottomLimitAngle();
        double tilt_range   = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        double tilt_percent = (tilt - tilt_min) / tilt_range;
        if (tilt_percent < 0) tilt_percent = -tilt_percent;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"tilt :"<<tilt<<endl;
        //cout<<"-------------------------------------------"<<endl;
        //cout<<"pan  :"<<pan<<endl;

        if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
          VoidAction::majuLurus       = true;
          //updateBallPosition(leftLimit,rightLimit,Behind);

          if ((receivedNumber == STAT_NETRAL) || (receivedNumber == STAT_ROBOT3)) {
            if (tilt_percent <= commDistance) {
              waitcount += 1;
              cekSend1  = false;
              if (cekSend2 == false) {
                shouldSend    = true;
                numberToSend  = STAT_ROBOT3;
                cekSend2    = true;
              }
            } else {
              waitcount = 0;
              cekSend2  = false;
              if (cekSend1 == false) {
                shouldSend    = true;
                numberToSend  = STAT_NETRAL;
                cekSend1    = true;
              }
            }
          }

          if ((tilt <= TILT_1)) {
            if ((mag < leftLimit) && (mag > Behind)) {
              VoidAction::netral();
              usleep(800000);
              VoidAction::prepluruskiri();
              while (1) {
                //std::cout << "Putarkiri" << std::endl;
                VoidAction::cobaluruskiri();
                StatusCheck::Check(cm730);
                if (mag > (leftLimit + 5) || mag < rightLimit) {
                  VoidAction::revprepluruskiri();
                  VoidAction::netral();
                  usleep(800000);
                  break;
                }
              }
            }
            else if ((mag > rightLimit) && (mag < Behind)) {
              VoidAction::netral();
              usleep(800000);
              VoidAction::prepluruskanan();
              while (1) {
                //std::cout << "Putarkanan" << std::endl;
                VoidAction::cobaluruskanan();
                StatusCheck::Check(cm730);
                if (mag > leftLimit || mag < (rightLimit - 5)) {
                  VoidAction::revprepluruskanan();
                  VoidAction::netral();
                  usleep(800000);
                  break;
                }
              }
            }
            else {
              if (tilt <= TILT_2) {
                if (waitcount >= countmax) {
                  if (m_KickBallCount >= 0){
                    m_FBStep = 0;
                    m_RLTurn = 0;
                    VoidAction::handleKick(pan,PAN_MID);
                    if (DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
                    if (cekSend1 == false) {
                        shouldSend    = true;
                        numberToSend  = STAT_NETRAL;
                        cekSend1    = true;
                      }
                    detekbal    = 0;
                    return;
                  }
                  else {
                    KickBall    	  = 0;
                    if (DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
                  }
                }
              }
              else {
                geser         	= false;
                m_KickBallCount = 0;
                KickBall      	= 0;
                m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
                if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
                m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.045) * 1.5;//1.5
                if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
                detekbal      	= 0;
                angkat        	= 0;
              }
            }
          }
          else {
            geser         	= false;
            m_KickBallCount = 0;
            KickBall      	= 0;
            m_GoalFBStep    = m_FollowMaxFBStep * tilt_percent;
            if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
            m_GoalRLTurn    = m_FollowMaxRLTurn * (pan_percent - 0.045) * 1.5;//1.5
            if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
            detekbal      	= 0;
            angkat        	= 0;
          }
        }
        else {
          geser       	  = false;
          m_KickBallCount = 0;
          KickBall    	  = 0;
          m_GoalFBStep  	= 0;
          m_GoalRLTurn  	= m_FollowMaxRLTurn * (pan_percent - 0.045) * 1.5; //1.5
          if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
          detekbal    	  = 0;

        }
        if (m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0) {
          if (Head::GetInstance()->GetPutar() < 1)
          {
            if (Walking::GetInstance()->IsRunning() == true) Walking::GetInstance()->Stop();
            else
            {
              if (m_KickBallCount < m_KickBallMaxCount) m_KickBallCount++;
            }
            if (DEBUG_PRINT == true) fprintf(stderr, " STOP");
          }
        }
        else {
          if (DEBUG_PRINT == true) fprintf(stderr, " START");
          if (Walking::GetInstance()->IsRunning() == false)
          {
            m_FBStep      	= 0;
            m_RLTurn      	= 0;
            m_KickBallCount = 0;
            KickBall      	= 0;
            Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
            Walking::GetInstance()->Start();
          }
          else
          {
            if (detekbal == 0)
            {
              //cout << "LURUUUUS"<<endl;
              if ((FB_move > 10) || (m_RLTurn != -0.5)) {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; // bisa 35.5
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;

              }
              else {
                Walking::GetInstance()->HIP_PITCH_OFFSET  = 31; //28.1
                Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              }
              Walking::GetInstance()->A_MOVE_AMPLITUDE  = -1; //0.5
              //Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; //28.1
              //Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
              //Walking::GetInstance()->X_MOVE_AMPLITUDE  = 5;
              //Walking::GetInstance()->Y_OFFSET      = 59;

              if((receivedNumber != 32)){
                if (m_FBStep < m_GoalFBStep) m_FBStep += m_UnitFBStep;
                else if (m_FBStep > m_GoalFBStep) m_FBStep = m_GoalFBStep;
              }

              if ( (m_RLTurn >= (m_FollowMaxRLTurn - 5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn + 5) && m_RLTurn < 0))
              {
                FB_move = m_FBStep - 1;
              }
              else FB_move = m_FBStep;

              FB_move += 0.5; // Kalibrasi dari setting walk tunner
              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
              //cout << "X_Move :"<<FB_move<<endl;

              if (m_RLTurn < m_GoalRLTurn) m_RLTurn += m_UnitRLTurn;
              else if (m_RLTurn > m_GoalRLTurn) m_RLTurn -= m_UnitRLTurn;

              if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

              if (DEBUG_PRINT == true) fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
            }
          }
        }
      }
    }
  }
  else if (buf[9] == Finish) {
    m_FBStep  = 0;
    FB_move   = 0;
    Walking::GetInstance()->Stop();
    Head::GetInstance()->MoveToHome();
  }
}

void BallFollower::ManualProses(){
  MotionManager::GetInstance()->SetEnable(true);
  Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
  Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
  cout << "Manual" << endl;
  VoidAction::searchMuter = true;
  // if (StatusCheck::trigready == 1) {
  //   VoidAction::ReadyS3();
  //   VoidAction::jalantempat();
  //   usleep(1000000);
  //   VoidAction::jalantempat2();
  //   usleep(800000);
  //   VoidAction::jalantempat22();
  //   usleep(800000);
  //   StatusCheck::trigready = 0;
  // }

  if (VoidAction::prepwalking == true) {
    VoidAction::prepjalan();
    Head::GetInstance()->MoveToHome();
    VoidAction::prepwalking = false;
  }

  if (posX == -1.0 || posY == -1.0) {
    KickBall  = 0;
    move    = 0;
    if (m_NoBallCount > m_NoBallMaxCount) {
      // can not find a ball
      m_GoalFBStep = 0;
      m_GoalRLTurn = 0;
      Head::GetInstance()->InitTracking(Start_Search);
      int Putar = Head::GetInstance()->GetPutar();
      //cout<<"Putar  :"<<Putar<<endl;
      if (Putar == 0) {
        if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
        else SearchingWalkControl(8, 0, 8, 0);
      }
      else if (Putar == 1) //kiri
      {
        if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
        else SearchingWalkControl(8, 0, 8, 0);
      }
      else if (Putar == 2) //kanan
      {
        if (Walking::GetInstance()->IsRunning() == false) Walking::GetInstance()->Start();
        else SearchingWalkControl(8, 0, 8, 0);
      }

      if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL]");

    }
    else {
      if    (offsetX > 0 && offsetY > 0)    start_search = 1; //kiri atas
      else if (offsetX < 0 && offsetY > 0)  start_search = 2; //kanan atas
      else if (offsetX < 0 && offsetY < 0)  start_search = 6; //kanan bawah
      else if (offsetX > 0 && offsetY < 0)  start_search = 5; //kiri bawah
      else                  				        start_search = 1;

      Head::GetInstance()->Start = true;
      m_NoBallCount++;
      if (DEBUG_PRINT == true) fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
    }

  }
  else {
    Last_Pos = ball_pos;

    Head::GetInstance()->putar  = 0;
    m_NoBallCount         = 0;

    double pan      = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
    double pan_range    = Head::GetInstance()->GetLeftLimitAngle();
    double pan_percent  = pan / pan_range;

    double tilt       = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
    double tilt_min     = Head::GetInstance()->GetBottomLimitAngle();
    double tilt_range   = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
    double tilt_percent   = (tilt - tilt_min) / tilt_range;
    if (tilt_percent < 0) tilt_percent = -tilt_percent;
    //cout<<"-------------------------------------------"<<endl;
    //cout<<"tilt :"<<tilt<<endl;
    //cout<<"-------------------------------------------"<<endl;
    //cout<<"pan  :"<<pan<<endl;

    if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
      VoidAction::majuLurus       = true;
      //updateBallPosition(leftLimit,rightLimit,Behind);

      if ((receivedNumber == STAT_NETRAL) || (receivedNumber == STAT_ROBOT3)) {
            if (tilt_percent <= commDistance) {
              waitcount += 1;
              cekSend1  = false;
              if (cekSend2 == false) {
                shouldSend    = true;
                numberToSend  = STAT_ROBOT3;
                cekSend2    = true;
              }
            } else {
              waitcount = 0;
              cekSend2  = false;
              if (cekSend1 == false) {
                shouldSend    = true;
                numberToSend  = STAT_NETRAL;
                cekSend1    = true;
              }
            }
          }

      if ((tilt <= TILT_1)) {
        if ((mag < leftLimit) && (mag > Behind)) {
          VoidAction::netral();
          usleep(800000);
          VoidAction::prepluruskiri();
          while (1) {
            //std::cout << "Putarkiri" << std::endl;
            VoidAction::cobaluruskiri();
            StatusCheck::Check(cm730);
            if (mag > (leftLimit + 5) || mag < rightLimit) {
              VoidAction::revprepluruskiri();
              VoidAction::netral();
              usleep(800000);
              break;
            }
          }
        }
        else if ((mag > rightLimit) && (mag < Behind)) {
          VoidAction::netral();
          usleep(800000);
          VoidAction::prepluruskanan();
          while (1) {
            //std::cout << "Putarkanan" << std::endl;
            VoidAction::cobaluruskanan();
            StatusCheck::Check(cm730);
            if (mag > leftLimit || mag < (rightLimit - 5)) {
              VoidAction::revprepluruskanan();
              VoidAction::netral();
              usleep(800000);
              break;
            }
          }
        }
        else {
          if (tilt <= TILT_2) {
            if (waitcount >= countmax) {
              if (m_KickBallCount >= 0)
                    {
                      m_FBStep = 0;
                      m_RLTurn = 0;
                      VoidAction::handleKick(pan,PAN_MID);
                      if (DEBUG_PRINT == true) fprintf(stderr, "[KICK]");
                      if (cekSend1 == false) {
                        shouldSend    = true;
                        numberToSend  = STAT_NETRAL;
                        cekSend1    = true;
                      }
                      detekbal    = 0;
                      return;
                    }
              else {
                KickBall    	= 0;
                if (DEBUG_PRINT == true) fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
              }
            }
          }
          else {
            geser         	= false;
            m_KickBallCount   = 0;
            KickBall      	= 0;
            m_GoalFBStep    	= m_FollowMaxFBStep * tilt_percent;
            if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
            m_GoalRLTurn    	= m_FollowMaxRLTurn * (pan_percent - 0.045) * 1.5;//1.5
            if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
            detekbal      	= 0;
            angkat        	= 0;
          }
        }
      }
      else {
        geser         	= false;
        m_KickBallCount   = 0;
        KickBall      	= 0;
        m_GoalFBStep    	= m_FollowMaxFBStep * tilt_percent;
        if (m_GoalFBStep < m_FollowMinFBStep) m_GoalFBStep = m_FollowMinFBStep;
        m_GoalRLTurn    	= m_FollowMaxRLTurn * (pan_percent - 0.045) * 1.5;//1.5
        if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
        detekbal      	= 0;
        angkat        	= 0;
      }
    }
    else {
      geser       	= false;
      m_KickBallCount = 0;
      KickBall    	= 0;
      m_GoalFBStep  	= 0;
      m_GoalRLTurn  	= m_FollowMaxRLTurn * (pan_percent - 0.045) * 1.5; //1.5
      if (DEBUG_PRINT == true) fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
      detekbal    	= 0;

    }
    if (m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0) {
      if (Head::GetInstance()->GetPutar() < 1)
      {
        if (Walking::GetInstance()->IsRunning() == true) Walking::GetInstance()->Stop();
        else
        {
          if (m_KickBallCount < m_KickBallMaxCount) m_KickBallCount++;
        }
        if (DEBUG_PRINT == true) fprintf(stderr, " STOP");
      }
    }
    else {
      if (DEBUG_PRINT == true) fprintf(stderr, " START");
      if (Walking::GetInstance()->IsRunning() == false)
      {
        m_FBStep      	= 0;
        m_RLTurn      	= 0;
        m_KickBallCount = 0;
        KickBall      	= 0;
        Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
        Walking::GetInstance()->Start();
      }
      else
      {
        if (detekbal == 0)
        {
          //cout << "LURUUUUS"<<endl;
          if ((FB_move > 10) || (m_RLTurn != -0.5)) {
            Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; // bisa 35.5
            Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;

          }
          else {
            Walking::GetInstance()->HIP_PITCH_OFFSET  = 31; //28.1
            Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
          }
          Walking::GetInstance()->A_MOVE_AMPLITUDE  = -1; //0.5
          //Walking::GetInstance()->HIP_PITCH_OFFSET  = 33; //28.1
          //Walking::GetInstance()->Z_MOVE_AMPLITUDE  = 30;
          //Walking::GetInstance()->X_MOVE_AMPLITUDE  = 5;
          //Walking::GetInstance()->Y_OFFSET      = 59;

          if((receivedNumber != 32)){
            if (m_FBStep < m_GoalFBStep) m_FBStep += m_UnitFBStep;
            else if (m_FBStep > m_GoalFBStep) m_FBStep = m_GoalFBStep;
          }

          if ( (m_RLTurn >= (m_FollowMaxRLTurn - 5) && m_RLTurn > 0) || (m_RLTurn <= (-m_FollowMaxRLTurn + 5) && m_RLTurn < 0))
          {
            FB_move = m_FBStep - 1;
          }
          else FB_move = m_FBStep;

          FB_move += 0.5; // Kalibrasi dari setting walk tunner
          if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->X_MOVE_AMPLITUDE = FB_move;
          //cout << "X_Move :"<<FB_move<<endl;

          if (m_RLTurn < m_GoalRLTurn) m_RLTurn += m_UnitRLTurn;
          else if (m_RLTurn > m_GoalRLTurn) m_RLTurn -= m_UnitRLTurn;

          if ((receivedNumber != 32) || (receivedNumber != 33) || (receivedNumber != 34)) Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

          if (DEBUG_PRINT == true) fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
        }
      }
    }
  }
}

void BallFollower::SearchingWalkControl(double x1, double a1, double x2, double a2){
  if ((mag >= (leftLimit - 5)) || (mag <= (rightLimit + 5))){
    if(VoidAction::firstkick_ == 0){
      Walking::GetInstance()->X_MOVE_AMPLITUDE = x1;
      Walking::GetInstance()->A_MOVE_AMPLITUDE = a1;
    }
    else{
      Walking::GetInstance()->X_MOVE_AMPLITUDE = x2;
      Walking::GetInstance()->A_MOVE_AMPLITUDE = a2;
    }
  }
  else if((mag < (leftLimit - 5)) && (mag > Behind)){
      Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
      Walking::GetInstance()->A_MOVE_AMPLITUDE = -8;
  }
  else{
      Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
      Walking::GetInstance()->A_MOVE_AMPLITUDE = 8;
  }
  Walking::GetInstance()->HIP_PITCH_OFFSET = 31; //28.1
  Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
}