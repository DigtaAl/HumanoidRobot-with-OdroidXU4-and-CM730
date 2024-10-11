/*
 * VoidAction.cpp
 *
 *  Created on: 2024. 4. 20.
 *      Author: Digta
 */

#include <cmath>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <termios.h>
#include "tracker.h"
#include <arpa/inet.h>
#include <sys/socket.h>

#include <darwin/framework/Head.h>
#include <darwin/framework/Action.h>
#include <darwin/framework/Walking.h>
#include <darwin/linux/LinuxDARwIn.h>
#include <darwin/framework/Konstanta.h>
#include <darwin/framework/VoidAction.h>
#include <darwin/framework/StatusCheck.h>
#include <darwin/framework/GameControl.h>
#include <darwin/framework/MotionStatus.h>
#include <darwin/framework/MotionManager.h>

using namespace Robot;
using namespace std;

LinuxCM730 linux_cm(U2D_DEV_CM);
CM730 cm(&linux_cm);

//////////////// Mixx (Main, Status C, Ball F) //////////////
//============= Syarat Awal ==================
int i = 0;
bool cyan 			= true;
bool magen 			= true;
bool manualll 		= true;
bool jalanawalmagen = true;


/////////////////////////// Void ////////////////////////////
void sighandler(int sig){
    exit(0);
}

void change_current_dir(){
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void cetak(const char *mystring){
    std::cout << mystring << std::endl;
    fprintf(stderr, "%s\n", mystring);
}

void Tendang(){
    //MotionManager::GetInstanceablm()->SetEnable(true);
    Action::GetInstance()->LoadFile(const_cast<char*>(MOTION_FILE_PATH));
    Action::GetInstance()->LoadFile(const_cast<char*>(MOTION_FILE_PATH));
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	//Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);	
}

void netral1(){	
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 32.5;
	Walking::GetInstance()->PERIOD_TIME = 700;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->R_OFFSET = 11.8;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
    Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 0;
	
}

void luruskiri(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->PERIOD_TIME 	 = 750;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -1;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 9;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void kekiri2(){ 	// Putar kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 20;
	Walking::GetInstance()->PERIOD_TIME 	 = 750;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -8;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 3;
}

void luruskanan(){ 	//lurus kekanan
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 20;
	Walking::GetInstance()->PERIOD_TIME 	 = 750;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 28.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -10;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -35;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void kekanan2(){ 	//Putar kekanan
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 20;
	Walking::GetInstance()->PERIOD_TIME 	 = 750;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 30.0;
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

void jalantempat1(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	//Walking::GetInstance()->PERIOD_TIME = 6;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=15;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void jalantempatawal(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 685;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=5;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void jalantempat(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void jalantempat2(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->PERIOD_TIME = 650;
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 1;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
}

void jalantempat22(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 650;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 1;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
	
}

void maju(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 29.5;
	Walking::GetInstance()->PERIOD_TIME = 700;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=5;
}

void majuu(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 29.5;
	Walking::GetInstance()->PERIOD_TIME = 700;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.7;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
}

void majuuu(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 29.5;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
    Walking::GetInstance()->PERIOD_TIME = 700;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=15;
}

void majuuuu(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 29.5;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
    Walking::GetInstance()->PERIOD_TIME = 700;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.4;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=15;
}

void ReadyS1(){
	Tendang();
	Action::GetInstance()->Start(14); //ganti walk
	usleep(2000000);
	jalantempat();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempat2();
	usleep(800000);
	StatusCheck::Check(cm);
	jalantempat22();
	usleep(800000);
	StatusCheck::Check(cm);
	maju();
	usleep(1000000);
	StatusCheck::Check(cm);
	usleep(1000000);
	StatusCheck::Check(cm);
	majuu();
	usleep(1000000);
	StatusCheck::Check(cm);
	usleep(1000000);
	StatusCheck::Check(cm);
	majuuu();
    usleep(3000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
	usleep(4000000);
	/*majuuuu();
	StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
	usleep(1500000);
	StatusCheck::Check(cm);
	usleep(1500000);*/
    jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void ReadyS2(){
	Tendang();
	Action::GetInstance()->Start(14); //ganti walk
	usleep(2000000);
	jalantempat();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempat2();
	usleep(800000);
	StatusCheck::Check(cm);
	jalantempat22();
	usleep(800000);
	StatusCheck::Check(cm);
	maju();
	usleep(1000000);
	StatusCheck::Check(cm);
	usleep(1000000);
	StatusCheck::Check(cm);
	majuu();
	usleep(1000000);
	StatusCheck::Check(cm);
	usleep(1000000);
	StatusCheck::Check(cm);
	usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
	/*majuuu();
    usleep(2000000);
    StatusCheck::Check(cm730);
    usleep(2000000);
    StatusCheck::Check(cm730);
    usleep(2000000);
    StatusCheck::Check(cm730);
    usleep(2000000);
    StatusCheck::Check(cm730);
	usleep(2000000);*/
    jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void ReadyS3(){
	Tendang();
	Action::GetInstance()->Start(66); //ganti walk
	usleep(2000000);
	jalantempat();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempat2();
	usleep(800000);
	StatusCheck::Check(cm);
	jalantempat22();
	usleep(800000);
	StatusCheck::Check(cm);
	maju();
	usleep(1000000);
	StatusCheck::Check(cm);
	usleep(1000000);
	StatusCheck::Check(cm);
	majuu();
	usleep(1000000);
	StatusCheck::Check(cm);
	usleep(1000000);
	StatusCheck::Check(cm);
	majuuu();
    usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
    usleep(2000000);
    StatusCheck::Check(cm);
	usleep(2000000);;
    jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void jalantempatwhs(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -1;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void jalantempatwhs1(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 32.0;
	Walking::GetInstance()->PERIOD_TIME = 620;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void jalantempatwhs2(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 33.0;
	Walking::GetInstance()->PERIOD_TIME = 640;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
}

void jalantempatwhs3(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 33.0;
	Walking::GetInstance()->PERIOD_TIME = 650;
	Walking::GetInstance()->BALANCE_ENABLE   = true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
}

void JalanCyanAsli(){
	jalantempatwhs();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs1();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs2();
	usleep(2000000);
	StatusCheck::Check(cm);
	jalantempatwhs3();
	usleep(2000000);
	StatusCheck::Check(cm);
	usleep(3000000);
	StatusCheck::Check(cm);
	usleep(4000000);
	StatusCheck::Check(cm);
	usleep(4000000);
	StatusCheck::Check(cm);
	usleep(4000000);
	StatusCheck::Check(cm);
	usleep(4000000);
	StatusCheck::Check(cm);
	usleep(2000000);
	StatusCheck::Check(cm);
	jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void JalanAwal(){
	jalantempatwhs();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs1();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs2();
	usleep(2000000);
	StatusCheck::Check(cm);
	jalantempatwhs3();
	usleep(2000000);
	StatusCheck::Check(cm);
	usleep(2000000);
	StatusCheck::Check(cm);
	usleep(2000000);
	StatusCheck::Check(cm);
	usleep(2000000);
	StatusCheck::Check(cm);
	usleep(2000000);
	StatusCheck::Check(cm);
	usleep(4000000);
	StatusCheck::Check(cm);
	jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void jalantempatbangun1(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void jalantempatbangun2(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->PERIOD_TIME = 650;
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
	
}

void jalantempatbangun3(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 685;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
}

void prepjalan(){
		Head::GetInstance()->MoveToHome();
		MotionManager::GetInstance()->SetEnable(true);
		Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
		Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
		jalantempatbangun1();
		usleep(1000000);
		jalantempatbangun2();
		usleep(1000000);
		jalantempatbangun3();
		usleep(1000000);
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

void muterkepalaKanan()
{
	Head::GetInstance()->MoveByAngle(45,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-45,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-90,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-120,40);
	jalantempatmuter();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,-20);
	jalantempatmuter();
	usleep(350000);
}
void muterkepalaKiri()
{
	Head::GetInstance()->MoveByAngle(45,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-45,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-90,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(-120,40);
	jalantempat1();
	usleep(350000);
	Head::GetInstance()->MoveByAngle(0,-20);
	jalantempat1();
	usleep(350000);
}
