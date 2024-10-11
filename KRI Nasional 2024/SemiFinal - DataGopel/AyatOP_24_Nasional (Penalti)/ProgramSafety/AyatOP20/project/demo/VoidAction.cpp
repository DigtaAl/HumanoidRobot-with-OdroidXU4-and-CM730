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
#include <darwin/framework/Interface.h>
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

//============= Syarat Awal ==================
int VoidAction::i = 0;
int VoidAction::firstkick_    	= 0;
bool VoidAction::cyan 			= true;
bool VoidAction::magen 			= true;
bool VoidAction::manualll 		= true;
bool VoidAction::jalanawalmagen = true;
bool VoidAction::majuLurus     	= false;
bool VoidAction::majuArahKiri  	= false;
bool VoidAction::tendangSKiri  	= false;
bool VoidAction::majuArahKanan	= false;
bool VoidAction::tendangSKanan 	= false;
bool VoidAction::searchMuter   	= true;
bool VoidAction::prepwalking 	= false;

/////////////////////////// Handle ////////////////////////////
void VoidAction::sighandler(int sig){
    exit(0);
}

void VoidAction::change_current_dir(){
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void VoidAction::cetak(const char *mystring){
    std::cout << mystring << std::endl;
    fprintf(stderr, "%s\n", mystring);
}

void VoidAction::updateBallPosition(int bataskiri, int bataskanan, int belakang){
    if ((mag <= (bataskiri - 50)) && (mag > belakang)) {
		majuArahKanan   = true;
		tendangSKanan   = true;
		majuArahKiri    = false;
		tendangSKiri    = false;
		majuLurus       = false;
	}
	else if ((mag >= (bataskanan + 50)) && (mag <= belakang)) {
		majuArahKiri    = true;
		tendangSKiri    = true;
		majuArahKanan   = false;
		tendangSKanan   = false;
		majuLurus       = false;
	}
	else if ((mag > (bataskiri - 50)) || (mag < (bataskanan + 50))) {
		majuLurus       = true;
		majuArahKanan   = false;
		tendangSKanan   = false;
		majuArahKiri    = false;
		tendangSKiri    = false;
	}
	else {
		majuArahKanan   = false;
		majuArahKiri    = false;
		tendangSKanan   = false;
		tendangSKiri    = false;
		majuLurus       = true;
	}
}

void VoidAction::handleKick(double pan2, double mid){
	if (tendangSKanan == true) {
		muterkepalaKanan();
		jalantempatawal();
		usleep(630000 * 2);
		Tendang();
		usleep(500000);
		Action::GetInstance()->Start(LS_KICK); // 108
		while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);
		searchMuter = true;
		prepwalking = true;
		firstkick_ += 1;
		return;
	}
	else if (tendangSKiri == true) {
		muterkepalaKiri();
		jalantempatawal();
		usleep(630000 * 2);
		Tendang();
		usleep(500000);
		Action::GetInstance()->Start(RS_KICK); // 108
		while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);
		searchMuter = true;
		prepwalking = true;
		firstkick_ += 1;
		return;
	}
	else {
		if (pan2 > mid) {
			jalantempatawal();
			usleep(200000 * 2);
			Tendang();
			usleep(500000);
			Action::GetInstance()->Start(L_KICK); // 108
			while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);
		}
		else {
			jalantempatawal();
			usleep(200000 * 2);
			Tendang();
			usleep(500000);
			Action::GetInstance()->Start(L_KICK); // 108
			while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);
		}
		firstkick_ += 1;
		prepwalking = true;
	}
}

/////////////////////////// Action ////////////////////////////
void VoidAction::Tendang(){
    //MotionManager::GetInstanceablm()->SetEnable(true);
    Action::GetInstance()->LoadFile(const_cast<char*>(MOTION_FILE_PATH));
    Action::GetInstance()->LoadFile(const_cast<char*>(MOTION_FILE_PATH));
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	//Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);	
}

void VoidAction::netral(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 13;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 0;
}

void VoidAction::arahkanan(){ 	// arah sudut kiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 13;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 14;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 0;
}

void VoidAction::arahkiri(){ 	// arah sudut kiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 13;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -15;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 0;
}

void VoidAction::luruskiri(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 30;
	Walking::GetInstance()->PERIOD_TIME 	 = 700;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 50;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void VoidAction::prep1luruskiri(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 15;
	Walking::GetInstance()->PERIOD_TIME 	 = 700;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 10;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void VoidAction::prep2luruskiri(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 20;
	Walking::GetInstance()->PERIOD_TIME 	 = 700;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void VoidAction::prep3luruskiri(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 25;
	Walking::GetInstance()->PERIOD_TIME 	 = 700;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void VoidAction::cobaluruskiri(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 25;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 25;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 35;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = -5;
}

void VoidAction::prepluruskiri(){ 	// lurus kekiri
	prep1luruskiri();
	usleep(100000);
	prep2luruskiri();
	usleep(200000);
	prep3luruskiri();
	usleep(200000);
}

void VoidAction::revprepluruskiri(){ 	// lurus kekiri
	prep3luruskiri();
	usleep(200000);
	prep2luruskiri();
	usleep(200000);
	prep1luruskiri();
	usleep(100000);
}

void VoidAction::kekiri2(){ 	// Putar kekiri
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

void VoidAction::cobaluruskanan(){ 	// lurus kekanan
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 25;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -3.5;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 12;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -35;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 5;
}

void VoidAction::prep1luruskanan(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 15;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 7;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -10;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 5;
}

void VoidAction::prep2luruskanan(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 20;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 7;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -20;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 5;
}

void VoidAction::prep3luruskanan(){ 	// lurus kekiri
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->R_OFFSET 		 = 25;
	Walking::GetInstance()->PERIOD_TIME 	 = 650;
	Walking::GetInstance()->BALANCE_ENABLE 	 = true;
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 9;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -30;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE = 5;
}

void VoidAction::prepluruskanan(){ 	// lurus kekiri
	prep1luruskanan();
	usleep(100000);
	prep2luruskanan();
	usleep(200000);
	prep3luruskanan();
	usleep(200000);
}

void VoidAction::revprepluruskanan(){ 	// lurus kekiri
	prep3luruskanan();
	usleep(200000);
	prep2luruskanan();
	usleep(200000);
	prep1luruskanan();
	usleep(100000);
}

void VoidAction::luruskanan(){ 	//lurus kekanan
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

void VoidAction::kekanan2(){ 	//Putar kekanan
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

void VoidAction::putarkepalaMagen(){
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

void VoidAction::jalantempat1(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	//Walking::GetInstance()->PERIOD_TIME = 6;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -1.5;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void VoidAction::jalantempatawal(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 685;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void VoidAction::jalantempat(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void VoidAction::jalantempat2(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->PERIOD_TIME = 650;
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 1;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
}

void VoidAction::jalantempat22(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 650;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 1;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
	
}

void VoidAction::maju(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31;
	Walking::GetInstance()->PERIOD_TIME = 700;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=5;
}

void VoidAction::majuu(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31;
	Walking::GetInstance()->PERIOD_TIME = 700;
    Walking::GetInstance()->BALANCE_ENABLE = true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE= 35;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.7;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 10;
}

void VoidAction::majuuu(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
    Walking::GetInstance()->PERIOD_TIME = 700;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=15;
}

void VoidAction::majuuuu(){
	 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31;
    Walking::GetInstance()->BALANCE_ENABLE =true;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE=35;
    Walking::GetInstance()->PERIOD_TIME = 650;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.4;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=15;
}

void VoidAction::ReadyS1(){
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

void VoidAction::ReadyS2(){
	//Tendang();
	//Action::GetInstance()->Start(14); //ganti walk
	//usleep(2000000);
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

void VoidAction::ReadyS3(){
	//Tendang();
	//Action::GetInstance()->Start(66); //ganti walk
	//usleep(2000000);
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

void VoidAction::jalantempatwhs(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -1;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void VoidAction::jalantempatwhs1(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 32.0;
	Walking::GetInstance()->PERIOD_TIME = 620;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void VoidAction::jalantempatwhs2(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 33.0;
	Walking::GetInstance()->PERIOD_TIME = 640;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=10;
}

void VoidAction::jalantempatwhs3(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 33.0;
	Walking::GetInstance()->PERIOD_TIME = 650;
	Walking::GetInstance()->BALANCE_ENABLE   = true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 21;
}

void VoidAction::jalantempatwhs4(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->HIP_PITCH_OFFSET = 33.0;
	Walking::GetInstance()->PERIOD_TIME = 650;
	Walking::GetInstance()->BALANCE_ENABLE   = true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE = 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
}

void VoidAction::JalanCyanAsli(){
	jalantempatwhs();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs1();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs2();
	usleep(1000000);
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
	usleep(2000000);
	StatusCheck::Check(cm);
	//usleep(2000000);
	//StatusCheck::Check(cm);
	jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	//jalantempatwhs4();
	//usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void VoidAction::JalanCyanBaru(){
	jalantempatwhs();
	usleep(500000);
	arahkiri();
	usleep(3000000);
	StatusCheck::Check(cm);
	jalantempatwhs1();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs2();
	usleep(1000000);
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
	usleep(2000000);
	StatusCheck::Check(cm);
	jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	arahkanan();
	usleep(5000000);
	//jalantempatwhs4();
	//usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void VoidAction::JalanCyanMirror(){
	jalantempatwhs();
	usleep(500000);
	arahkanan();
	usleep(3300000);
	StatusCheck::Check(cm);
	jalantempatwhs1();
	usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatwhs2();
	usleep(1000000);
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
	usleep(2000000);
	StatusCheck::Check(cm);
	jalantempat();
	usleep(800000);
	jalantempat();
	usleep(700000);
	arahkiri();
	usleep(4500000);
	//jalantempatwhs4();
	//usleep(1000000);
	StatusCheck::Check(cm);
	jalantempatawal();
	usleep(500000);
	StatusCheck::Check(cm);
}

void VoidAction::JalanAwal(){
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

void VoidAction::jalantempatbangun1(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}

void VoidAction::jalantempatbangun2(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->PERIOD_TIME = 650;
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
	
}

void VoidAction::jalantempatbangun3(){
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 685;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
}

void VoidAction::prepjalan(){
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

void VoidAction::jalantempatmuter()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	//Walking::GetInstance()->PERIOD_TIME = 6;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
	
}

void VoidAction::muterkepalaKanan()
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
void VoidAction::muterkepalaKiri()
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
