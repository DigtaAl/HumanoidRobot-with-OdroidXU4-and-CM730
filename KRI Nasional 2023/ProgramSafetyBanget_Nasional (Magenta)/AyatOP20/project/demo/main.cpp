#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include "tracker.h"
#include <pthread.h>
#include <pthread.h>
#include <termios.h>
#include <math.h>
#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <darwin/framework/StatusCheck.h>

//#include <darwin/framework/Magneto.h>
#include <darwin/framework/BallFollower.h>
#include <darwin/framework/MXDXL.h>
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

//void wifi();

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
#define U2D_DEV_CM    	    "/dev/ttyUSBCM"

LinuxCM730 linux_cm730(U2D_DEV_CM);
CM730 cm730(&linux_cm730);

//int Magneto::Yaw=0;

void cetak(const char *mystring)
{
	cout << mystring << endl;
	fprintf(stderr, mystring);
	fprintf(stderr, "\n");
}

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
bool initial = false;
bool ready = false;
bool play=false;
bool set1=false;
int newHalf;
bool kintil = false;

//////////////////////////////syarat awal///////////////////////////////////////////////////////
bool cyan = true;
bool magen = true;
bool jalanawalmagen = true;
bool checkMagen = true;
bool manualll = true;
bool tenwal = true;
bool awal = false;
int i = 0;

//const char header1 = 85, header2 = 78, header3 = 89;//header1-header2-heaader3 = U-N-Y
const char GameHeader1 = 0x52, GameHeader2 = 0x47, GameHeader3 = 0x6D, GameHeader4 = 0x65;    //Buffer 0-3
const int Version = 7; //82                                                                //buffer 4

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

int baca;
int mag;
int nani;
char buf2[32];

void IMUData()
{
	baca = open("/dev/ttyUSBCOMPASS", O_RDWR | O_NOCTTY);
	struct termios toptions;
	tcgetattr(baca, &toptions);
	cfsetispeed(&toptions, B115200);
	cfsetospeed(&toptions, B115200);
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	toptions.c_cflag &= ~CRTSCTS;
	toptions.c_cflag |= CREAD | CLOCAL;
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	toptions.c_oflag &= ~OPOST;
	toptions.c_cc[VMIN] = 1;
	toptions.c_cc[VTIME] = 0;
	tcsetattr(baca, TCSANOW, &toptions);
	// tcflush(baca, TCIOFLUSH);
}

void *thread_function(void *n)
{
	IMUData();
	while (true)
	{

		nani = read(baca, &buf2, 32);
		mag = atoi(buf2);
		//std::cout << "asdasdl: " << mag << std::endl;
	}
}


void waitData(){
	//keep listening for data
	printf("Waiting for data...\n");
	fflush(stdout);
	printf("Try Receive.....\n");	
	//try to receive some data, this is a blocking call
	recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) ;
}
void *wifi(void *n)
{	
	while(1){
		StatusCheck::Check(cm730);
		waitData();
		if(StatusCheck::m_cur_mode == CYAN || StatusCheck::m_cur_mode == MAGEN || StatusCheck::m_cur_mode == MANUAL){
		if(buf[0] == GameHeader1 && buf[1] == GameHeader2 && buf[2] == GameHeader3 && buf[3] == GameHeader4)
		{
			fprintf(stderr, "Siap masuk\n");
			if((int)buf[4] == Version)
			{
				//fprintf(stderr, "Versi\n");
				if (buf[9] == Initial)
				{	
					initial = true;
					ready	= false;	
					set1		= false;
					play	= false;
					fprintf(stderr, "Inisial\n");
					
				}
				else if (buf[9] == Ready) 
				{
					initial = false;
					ready	= true;	
					set1		= false;
					play	= false;
					fprintf(stderr, "Ready\n");
				}
				else if (buf[9] == Set) 
				{	
					initial = false;
					ready	= false;	
					set1		= true;
					play	= false;
					fprintf(stderr, "Set\n");
				}
				else if (buf[9] == Play) 
				{
					initial = false;
					ready	= false;	
					set1		= false;
					play	= true;
					fprintf(stderr, "Play\n");
				}
			}
		}
	}
	}
}

void Tendang()
{
//	MotionManager::GetInstanceablm()->SetEnable(true);
    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
    Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	//Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);	
}
void netral1()
{	
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 32.5;
	Walking::GetInstance()->PERIOD_TIME = 700;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->R_OFFSET = 11.8;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
    Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 0;
	
}

void luruskiri()	// lurus kekiri
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
	Walking::GetInstance()->PERIOD_TIME = 750;
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -1;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 9;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void kekiri2()	// Putar kekiri
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
	Walking::GetInstance()->PERIOD_TIME = 750;
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Walking::GetInstance()->R_OFFSET = 20;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -8;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 3;
}

void luruskanan()	//lurus kekanan
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 28.0;
	Walking::GetInstance()->PERIOD_TIME = 750;
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Walking::GetInstance()->R_OFFSET = 20;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -10;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -35;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void kekanan2()	//Putar kekanan
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
	Walking::GetInstance()->PERIOD_TIME = 750;
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Walking::GetInstance()->R_OFFSET = 20;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 5; //lurus kanan = -10
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -20; //-30
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -3; //-7
}

void putarkepalaMagen(){
	Tendang();
	Action::GetInstance()->Start(14); //ganti walk
	while(Action::GetInstance()->IsRunning()) usleep(350000);
	Action::GetInstance()->Start(64); //ganti walk
	while(Action::GetInstance()->IsRunning()) usleep(350000);
	Action::GetInstance()->Start(62); //ganti walk
	while(Action::GetInstance()->IsRunning()) usleep(350000);
	Action::GetInstance()->Start(65); //ganti walk
	while(Action::GetInstance()->IsRunning()) usleep(350000);
}

void *thread_function(void *data);
void *wifi(void *data);
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
	//trackers tracker = trackers();
    
    //////////////////// Framework Initialize ////////////////////////////
    
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_CM);
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
	
    LinuxMotionTimer linuxMotionTimer;
	linuxMotionTimer.Initialize(MotionManager::GetInstance());
	linuxMotionTimer.Start();
    /////////////////////////////////////////////////////////////////////
    
    MotionManager::GetInstance()->LoadINISettings(ini);
	
    int firm_ver = 0;
    if(cm730.ReadByte(JointData::ID_L_KNEE, MXDXL::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
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
	//   cm730.WriteByte(CM730::P_LED_PANNEL, 0x01, NULL);
	//cm730.WriteWord(CM730::P_LED_HEAD_L, 0x20, NULL);
	//cm730.WriteWord(CM730::P_LED_EYE_L, 0x20, NULL);
			
    //LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    
    Action::GetInstance()->Start(14); //ganti walk
    //Action::GetInstance();
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

	pthread_t thread_1, thread_2;
	pthread_create(&thread_1, NULL, thread_function, NULL);
	pthread_create(&thread_2, NULL, wifi, NULL);
	
    while(1)
    {
	//cout<<"asdasdasda"<<endl; krsbi h: 104
	//krsbi b: 282
	StatusCheck::Check(cm730);
	
	follower.Process();
	
}
}

int magneto_data[8];
int value=0;
int error=0;
	
/*int yaw ()
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
	
}*/

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




 

