#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include "tracker.h"

#include <darwin/framework/Magneto.h>
#include <darwin/framework/BallFollower.h>

//tambahan wifi
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h> 
#define BUFLEN 30	//Max length of buffer 1024
#define PORT 3838	//The port on which to listen for incoming data

using namespace Robot;
using namespace std;

char buf[BUFLEN];
short int hit=0;
struct sockaddr_in si_me, si_other;	
int s,recv_len;
socklen_t slen;

void wifi();

//================================
//#include "mjpg_streamer.h"
#include <darwin/linux/LinuxDARwIn.h>

#include <darwin/framework/StatusCheck.h>
//#include "VisionMode.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

int Magneto::Yaw=0;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    exit(0);
}

//////////////////////////////////tambahan untuk wifi///////////////////////////////////////////
bool manual = false;
bool play=false;
bool set=false;
int newHalf;

//const char header1 = 85, header2 = 78, header3 = 89;//header1-header2-heaader3 = U-N-Y
const char GameHeader1 = 0x52, GameHeader2 = 0x47, GameHeader3 = 0x6D, GameHeader4 = 0x65;    //Buffer 0-3
const char Version = 07;                                                                //buffer 4

//============= State ==================                                                //buffer 9
const char Initial  = 0;
const char Ready    = 1;
const char Set      = 2;
const char Play     = 4;
const char Finish   = 05;

int scale(int Input, int Min_Input, int Max_Input, int Min_Output, int Max_Output);
int pow (int nilai, int pangkat);
int yaw ();
void UDP_client();

int main(void)
{
	
	//tambahan WIFI
	s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));
		
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
		
	//bind socket to port
	bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) ;
	
	//==========================
    fprintf(stderr, "Program Start \n");
	signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

	
    BallFollower follower = BallFollower();
	trackers tracker = trackers();
    
    //////////////////// Framework Initialize ////////////////////////////
    
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);
	
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	
  //  LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
   // motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    
    MotionManager::GetInstance()->LoadINISettings(ini);
	
    int firm_ver = 0;
    if(cm730.ReadByte(JointData::ID_L_KNEE, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_L_KNEE);
        exit(0);
    }
	

    if(0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher atas\n\n");
		fprintf(stderr, "Check Version 28 fail \n");
        exit(0);
#endif
    }
    else if(27 <= firm_ver)
    {
#ifdef MX28_1024
        fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
        fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
		fprintf(stderr, "Check Version 18 fail \n");
        exit(0);
#else
        Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
#endif
    }
    else
        exit(0);
	

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);
	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
	Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);
		
    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01, NULL);
	cm730.WriteWord(CM730::P_LED_HEAD_L, 0x20, NULL);
	cm730.WriteWord(CM730::P_LED_EYE_L, 0x20, NULL);
			
    //LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    
    Action::GetInstance()->Start(15); //ganti walk
    //Action::GetInstance();
    //while(Action::GetInstance()->IsRunning()) usleep(8*1000);
	
    while(1)
    {
		//cout<<"asdasdasda"<<endl;
		wifi();
		StatusCheck::Check(cm730);
				
		Magneto::Yaw=yaw();
		//fprintf(stderr,"Yaw : %d \n",yaw());	
		
		if(StatusCheck::m_cur_mode == READY && buf[9] == Ready)
		//if(StatusCheck::m_cur_mode == READY || buf[9] == Ready) //tambahan wifi
        {
			               
			MotionManager::GetInstance()->SetEnable(true);
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			play=false;
			set=false;	
			  
		} 
        else if((StatusCheck::m_cur_mode == SOCCER && buf[1]== '3')||(StatusCheck::m_cur_mode == SOCCER && buf[1]== '4'))
        {
        	
//play=true;
 
				if(Action::GetInstance()->IsRunning() == 0)
			    {
				MotionManager::GetInstance()->SetEnable(true);
				Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			    cout<< "Masuk Soccer"<<endl;
        	   follower.Process();
			    }
        	
        	
	    }

        if(StatusCheck::m_is_started == 0)
            continue;

        switch(StatusCheck::m_cur_mode)
        {
        case READY:
			play=false;
            break;
        case SOCCER:
				if((Action::GetInstance()->IsRunning() == 0 && buf[1] == 4) || (Action::GetInstance()->IsRunning() == 0 && buf[1] == 3))
				{ 
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                    cout<<"opencv image bawah"<<endl;
                     		
              		follower.Process();
				  
					if(follower.KickBall != 0)
					{
						cout<<"bawah"<<endl;
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
						Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

						/*if(follower.KickBall == -1)
						{
							Action::GetInstance()->Start(8);   // RIGHT KICK
							//Action::GetInstance()->Start(93);
							fprintf(stderr, "RightKick! \n");
							follower.KickBall=0;
						}
						else if(follower.KickBall == 1)
						{
							Action::GetInstance()->Start(14);   // LEFT KICK
							//Action::GetInstance()->Start(94);
							fprintf(stderr, "LeftKick! \n");
							follower.KickBall=0;
						}*/
					}
			}
			
			//break;
        }
   
    //return 0;
}
}

int magneto_data[8];
int value=0;
int error=0;
	
int yaw ()
{	
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC2_L, &value, &error) == CM730::SUCCESS) magneto_data[0]=value; 
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC3_L, &value, &error) == CM730::SUCCESS) magneto_data[1]=value; 
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC4_L, &value, &error) == CM730::SUCCESS) magneto_data[2]=value; 
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC5_L, &value, &error) == CM730::SUCCESS) magneto_data[3]=value; 
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC6_L, &value, &error) == CM730::SUCCESS) magneto_data[4]=value; 
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC7_L, &value, &error) == CM730::SUCCESS) magneto_data[5]=value; 
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC8_L, &value, &error) == CM730::SUCCESS) magneto_data[6]=value; 
	if(cm730.ReadWord(CM730::ID_CM, CM730::P_ADC15_L, &value, &error) == CM730::SUCCESS) magneto_data[7]=value; 

	int bit[8];
	for(int i=0;i<8;i++) 
	{
		if(magneto_data[i]>=512) bit[i]=1; else bit[i]=0;
		if(magneto_data[7]>=60) bit[7]=1; else bit[7]=0;
	}
	
	int dec=0;
	for (int i=0; i < 8; i++) dec += bit[i] * pow(2, i);
	
    double Yaw=0;
	Yaw = scale(dec, 0, 255, 0, 360);
	
	return Yaw;
	
}

 int scale(int Input, int Min_Input, int Max_Input, int Min_Output, int Max_Output)
 {
            double miu_naik = (double)(Input - Min_Input) / (double)(Max_Input - Min_Input);
            double miu_turun = (double)(Max_Input - Input) / (double)(Max_Input - Min_Input);

            double z_naik = (double)(Max_Output - Min_Output) * miu_naik + (double)Min_Output;
            double z_turun = (double)Max_Output - (double)(Max_Output - Min_Output) * miu_turun;

            return (((z_naik * miu_naik + z_turun * miu_turun) / (miu_naik + miu_turun)));
}

int pow (int nilai, int pangkat)
{
	int hasil=1;
	for(int i=0;i<pangkat;i++) hasil=hasil*nilai;
	return hasil;
}

void die(char *s)
{
	perror(s);
	exit(1);
}


int set_count=0;
void wifi()
{	

			printf("Waiting for data...\n");
			fflush(stdout);
			printf("Try Receive.....\n");	
			StatusCheck::m_cur_mode = SOCCER;
	//		cout<<"bufff " << buf<<endl;
			//try to receive some data, this is a blocking call
			recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) ;
			set_count=0;
		

	if(StatusCheck::m_cur_mode == SOCCER)
	{
		
			fprintf(stderr, "Siap masuk\n");
			//if(buf[4] == Version)
			//{
				fprintf(stderr, "Versi\n");
				if (buf[1] == '0' || buf[1] =='5')
				{	
					set=false;
					play=false;
					fprintf(stderr, "Ready\n");
					//Action::GetInstance()->Start(9);
				}
				else if (buf[1] == '1' || buf[1] =='2') 
				{
					set=false;
					play=false;
					fprintf(stderr, "Set\n");
					//Action::GetInstance()->Start(9);
				}
				else if (buf[1] == '3' || buf[1]=='4') 
				{	
					play=false;
					set=true;
					fprintf(stderr, "Play\n");
				}

		//	}
		//}
	}
}

 

