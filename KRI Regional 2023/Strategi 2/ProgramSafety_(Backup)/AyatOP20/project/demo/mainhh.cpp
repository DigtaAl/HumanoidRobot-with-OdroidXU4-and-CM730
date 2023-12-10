#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

//#include <darwin/framework/Magneto.h>
#include <darwin/framework/BallFollower.h>

////////tambahan wifi/////////////////////
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
#include <darwin/linux/LinuxDARwIn.h>

#include "StatusCheck.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxArbotixPro linux_arbotixpro(U2D_DEV_NAME0);
ArbotixPro arbotixpro(&linux_arbotixpro);

//int Magneto::Yaw=0;

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
const char Play     = 3;
const char Finish   = 4;

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
    
    //////////////////// Framework Initialize ////////////////////////////
    
	if(MotionManager::GetInstance()->Initialize(&arbotixpro) == false)
    {
        linux_arbotixpro.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&arbotixpro) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }
    
    Walking::GetInstance()->LoadINISettings(ini);
	
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	
    LinuxMotionTimer linuxMotionTimer;
	linuxMotionTimer.Initialize(MotionManager::GetInstance());
	linuxMotionTimer.Start();
    /////////////////////////////////////////////////////////////////////
    
	MotionManager::GetInstance()->LoadINISettings(ini);
	
    int firm_ver = 0;
    if(arbotixpro.ReadByte(JointData::ID_L_KNEE, MXDXL::P_VERSION, &firm_ver, 0)  != ArbotixPro::SUCCESS)
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
		
    arbotixpro.WriteByte(ArbotixPro::P_LED_PANNEL, 0x01, NULL);
	arbotixpro.WriteWord(ArbotixPro::P_LED_HEAD_L, 0x20, NULL);
	arbotixpro.WriteWord(ArbotixPro::P_LED_EYE_L, 0x20, NULL);
    
    Action::GetInstance()->Start(15); //ganti walk
    Action::GetInstance();
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
	
    while(1)
    {
		wifi();
		StatusCheck::Check(arbotixpro);
				
		//Magneto::Yaw=yaw();
		
		MotionManager::GetInstance()->SetEnable(true);
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);	
		//usleep(10000000);
		//follower.Process();
		
		if(StatusCheck::m_cur_mode == READY)
        {
			               
			MotionManager::GetInstance()->SetEnable(true);
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			play=false;
			set=false;	
			
			//follower.Process();
			  
		} 
        else if(StatusCheck::m_cur_mode == SOCCER && buf[9]== Play)
        {
        	//play=true;
        	if(set==false && play==true)
			{
				if(Action::GetInstance()->IsRunning() == 0)
			    {
				MotionManager::GetInstance()->SetEnable(true);
				Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			    cout<< "Masuk Soccer"<<endl;
        	    follower.Process();
			    }
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
				if(set==true)
				{
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                    cout<<"opencv image bawah"<<endl;
                     		
              		follower.Process();
				}

			
			//break;
        }
    }
    //return 0;
}


int magneto_data[8];
int value=0;
int error=0;
	

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
	//keep listening for data
	if(play==false || set==true)
	{
		if((set==true && set_count>=5) || (play==false && set==false)){
			printf("Waiting for data...\n");
			fflush(stdout);
			printf("Try Receive.....\n");	
			//try to receive some data, this is a blocking call
			recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) ;
			set_count=0;
		}
		set_count++;
	}
	if(StatusCheck::m_cur_mode == SOCCER){
		if(buf[0] == GameHeader1 && buf[1] == GameHeader2 && buf[2] == GameHeader3 && buf[3] == GameHeader4)
		{
			//fprintf(stderr, "Buff[4]:%04x\n", buf[0]);
			fprintf(stderr, "Siap masuk\n");
			if(buf[4] == Version)
			{
				fprintf(stderr, "Versi\n");
				if (buf[9] == Initial)
				{	
					set=false;
					play=false;
					fprintf(stderr, "Inisial\n");
					//Action::GetInstance()->Start(9);
				}
				else if (buf[9] == Ready) 
				{
					set=false;
					play=false;
					fprintf(stderr, "Ready\n");
					//Action::GetInstance()->Start(9);
				}
				else if (buf[9] == Set) 
				{	
					play=false;
					set=true;
					fprintf(stderr, "Set\n");
				}
				else if (buf[9] == Play) 
				{
					play=true;
					set=false;
					fprintf(stderr, "Play\n");
				}
			}
		}
	}
}

 

