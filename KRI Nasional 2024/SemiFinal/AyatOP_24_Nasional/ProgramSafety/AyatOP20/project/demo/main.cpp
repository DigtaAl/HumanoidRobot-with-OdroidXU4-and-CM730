/*
   Main.cpp

    Created on: 2011. 1. 21.
        Author:
        Editor: Digta
*/

#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include "tracker.h"

//Tambahan Wifi//
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <darwin/framework/Comm.h>
#include <darwin/framework/MXDXL.h>
#include <darwin/linux/LinuxDARwIn.h>
#include <darwin/framework/Konstanta.h>
#include <darwin/framework/Interface.h>
#include <darwin/framework/VoidAction.h>
#include <darwin/framework/StatusCheck.h>
#include <darwin/framework/GameControl.h>
#include <darwin/framework/BallFollower.h>

using namespace Robot;
using namespace std;

LinuxCM730 linux_cm730(U2D_DEV_CM);
CM730 cm730(&linux_cm730);

//void *thread_function(void *data);
//void *wifi(void *data);

//////////////////////////////////Main///////////////////////////////////////////
int main(void){
  //Tambahan WIFI
  s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  // zero out the structure
  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  //bind socket to port
  bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) ;

  //==========================
  fprintf(stderr, "Start Main Program \n");
  signal(SIGABRT, &VoidAction::sighandler);
  signal(SIGTERM, &VoidAction::sighandler);
  signal(SIGQUIT, &VoidAction::sighandler);
  signal(SIGINT, &VoidAction::sighandler);

  VoidAction::change_current_dir();
  minIni* ini = new minIni(INI_FILE_PATH);
  BallFollower follower = BallFollower();
  //trackers tracker = trackers();

  //////////////////// Framework Initialize ////////////////////////////
  if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
    linux_cm730.SetPortName(U2D_DEV_NAME1);
    if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
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
  MotionManager::GetInstance()->LoadINISettings(ini);

  //////////////////// Baca Servo ////////////////////////////
  int firm_ver = 0;
  for (int i = 1; i <= 20; ++i) {
    if (cm730.ReadByte(i, MXDXL::P_VERSION, &firm_ver, 0) != CM730::SUCCESS) {
      fprintf(stderr, "Can't read firmware version from Servo ID %d!! \n\n", i);
    }
  }
  /////////////////////////////////////////////////////////////////////

  if (0 < firm_ver && firm_ver < 27) {
#ifdef MX28_1024
    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
    fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
    fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher atas\n\n");
    fprintf(stderr, "Check Version 28 fail \n");
    exit(0);
#endif
  }
  else if (27 <= firm_ver) {
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

  Action::GetInstance()->Start(WALK_READY); // Walkready
  while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);
  pthread_t thread_1, thread_2, connectThread;
  pthread_create(&thread_1, NULL, Interface::thread_function, NULL);
  pthread_create(&thread_2, NULL, GameControl::wifi, NULL);
  pthread_create(&connectThread, NULL, Comm::connectToServer, NULL);
  while (1) {
    //Walking::GetInstance()->Stop();
    //usleep(2000000);
    //VoidAction::JalanCyanluruskiri();
    StatusCheck::Check(cm730);
    follower.Process();
  }
  pthread_join(connectThread, NULL);

  return 0;
}