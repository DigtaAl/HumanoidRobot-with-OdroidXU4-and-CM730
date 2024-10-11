#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"


#include <iostream>
#include <fstream>
#include <ostream>
#include <unistd.h>

using namespace std;
using namespace cv;
 
int lowH 			= 35;	// Set Hue
int highH 			= 100;

int lowS 			= 20;	// Set Saturation
int highS 			= 255;

int lowV 			= 10;	// Set Value
int highV 			= 255;
	
int erotion_size 	= 1;
int dilation_size 	= 0; 
int dilasi 			= 4;
//-----------------------------------------------------

int lowH1 			= 0;	// Set Hue
int highH1 			= 23;

int lowS1 			= 170;	// Set Saturation
int highS1 			= 255;

int lowV1 			= 10;   // Set Value
int highV1 			= 255;	

int erotion_size1 	= 0;
int dilation_size1 	= 0; 			
//-----------------------------------------------------

bool posisi;
 double 	posX;
 double 	posY;
 double 	posX1;
 double 	posY1;

void save()
{
	ofstream saveFile;
	saveFile.open("data_kalibrasi.txt");
    
    saveFile << lowH << endl;
    saveFile << highH << endl;
    saveFile << lowS << endl;
    saveFile << highS << endl;
    saveFile << lowV << endl;
    saveFile << highV << endl;
     saveFile << lowH1 << endl;
    saveFile << highH1 << endl;
    saveFile << lowS1 << endl;
    saveFile << highS1 << endl;
    saveFile << lowV1 << endl;
    saveFile << highV1 << endl;
    saveFile << erotion_size << endl;
    saveFile << dilation_size << endl;
    saveFile << dilasi << endl;
    saveFile << erotion_size1 << endl;
    saveFile << dilation_size1 << endl;
	
    cout << "                        Berhasil menyimpan nilai kalibrasi" << endl;
    saveFile.close();    
}

void load()
{
	ifstream loadFile;
   
    loadFile.open ("data_kalibrasi.txt");
	
	loadFile >> lowH;
	loadFile >> highH;
	loadFile >> lowS;
	loadFile >> highS;
	loadFile >> lowV;
	loadFile >> highV;
	loadFile >> lowH1;
	loadFile >> highH1;
	loadFile >> lowS1;
	loadFile >> highS1;
	loadFile >> lowV1;
	loadFile >> highV1;
	loadFile >> erotion_size;
	loadFile >> dilation_size;
	loadFile >> dilasi;
	loadFile >> erotion_size1;
	loadFile >> dilation_size1;
	
    
    cout << "                          Berhasil memuat data kalibrasi" << endl;
    loadFile.close();
}



int main () 
{
           
     cout << "\n------------------------Program Kalibrasi Al-Aadiyaat v1.0----------------------" << endl;
     cout << "----------------------------------by digtanibossss-------------------------------\n" << endl;
    
 //load();
 const int width = 320; // ukuran lebar jendela
 const int height = 280;// ukuran tinggi jendela

 cv::VideoCapture capWebcam(0);  

 /*Setingan parameter kamera*/
 capWebcam.set(CV_CAP_PROP_FRAME_WIDTH,width);
 capWebcam.set(CV_CAP_PROP_FRAME_HEIGHT,height);
 
 if (capWebcam.isOpened() == false)  
 {    
  std::cout << "eror: Koneksi webcam gagal\n"; 
  return(0);            
 }

 cv::Mat imgOriginal;
 
 cv::Mat hsvImg;
 cv::Mat hslImg;
 cv::Mat bola;
 cv::Mat tiang;
 cv::Mat lapangan;
  
 std::vector<cv::Vec3f> v3fCircles;	 
 char keyboard = 0;
 
 while (keyboard != 27 && capWebcam.isOpened()) 
 {    
 bool blnFrameReadSuccessfully = capWebcam.read(imgOriginal);  

  if (!blnFrameReadSuccessfully || imgOriginal.empty()) 
  {    
  std::cout << "eror: sambungan kamera terputus, hubungkan kembali usb \n";      
   break;               
  }
			  cv::Mat gambarkopi = imgOriginal.clone();
			  cv::Mat Brighnessdec50;      // Convert Original Image to HSV Thresh Image
			  imgOriginal.convertTo(Brighnessdec50,-1,0.5,-10);
			  imgOriginal = Brighnessdec50;  
			  cv::cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);
			  
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
			imgOriginal.copyTo(masking,lapangan2);
					
			
			
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
			imgOriginal.copyTo(masking3,im_out2);
		
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
					posX = mu1[i].m10 / mu1[i].m00;
				posY = mu1[i].m01 / mu1[i].m00;
				
				}

				

			}
			//search for largest contour has end
			
			Mat gabung;
				cv::hconcat(gambarkopi,masking3,gabung);	
			
			if (contours1.size() > 0)
			{
				rectangle(gabung, bounding_rect,  Scalar(0,255,0),2, 8,0);

			}
							
				
				cout<<"bola x : "<<posX<<endl;
				cout<<"bola y : "<<posY<<endl; 
					
					
		
			 //-----------------------------------------------------------------------------------------------find contour bola	
							 
			 
			  // declare windows
			 // cv::namedWindow("Lapangan", CV_WINDOW_AUTOSIZE);
			 // cv::namedWindow("bola", CV_WINDOW_AUTOSIZE);
			  //cv::namedWindow("Kamera Robot", CV_WINDOW_AUTOSIZE); 
			  cv::namedWindow("Setting Bola", CV_WINDOW_AUTOSIZE);
			  cv::namedWindow("Setting Lapangan", CV_WINDOW_AUTOSIZE);
			  cv::namedWindow("Lapangan", CV_WINDOW_AUTOSIZE);
			  //cv::namedWindow("Crop2", CV_WINDOW_AUTOSIZE);
			 
			  cv::moveWindow("Lapangan",0,0);
			//  cv::moveWindow("Setting Bola",15,30);
			  //cv::moveWindow("Setting Lapangan",15,30);
			
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
				
			// cv::imshow("Kamera Robot", imgOriginal);     // show windows
			 //cv::imshow("bola", gambarkopi);     // show windows
			 cv::imshow("Setting Lapangan", lapangan);
			 cv::imshow("Setting Bola", bola);
			 //cv::imshow("Setting Bright", Brighnessdec50);
		     cv::imshow("Lapangan", gabung);
		  // cv::imshow("Crop2", im_out2);
		   
  
  keyboard = cv::waitKey(1);    

     
    
    
 }
    save();
    return 0;
}
