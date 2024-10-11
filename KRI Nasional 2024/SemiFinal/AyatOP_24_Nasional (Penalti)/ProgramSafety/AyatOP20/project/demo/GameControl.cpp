/*
   GameControl.cpp

    Created on: 2024. 6. 17.
        Author: Digta
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

LinuxCM730 linux_cm1(U2D_DEV_CM);
CM730 cm1(&linux_cm1);

//============= Game Controller ==================
//const char header1   = 85, header2 = 78, header3 = 89;//header1-header2-heaader3 = U-N-Y
const char GameHeader1 = 0x52, GameHeader2 = 0x47, GameHeader3 = 0x6D, GameHeader4 = 0x65;
const int  Version      = 7; //82
char buf[BUFLEN];
int s, recv_len;
struct sockaddr_in si_me, si_other;
socklen_t slen;

bool play     = false;
bool set1     = false;
bool ready      = false;
bool manual     = false;
bool initial    = false;

void GameControl::waitData() {
  ////keep listening for data
  //printf("Waiting for data...\n");
  fflush(stdout);
  //printf("Try Receive.....\n");
  //try to receive some data, this is a blocking call
  recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) ;
}

void *GameControl::wifi(void *n) {
  while (1) {
    StatusCheck::Check(cm1);
    waitData();
    if (StatusCheck::m_cur_mode == CYAN || StatusCheck::m_cur_mode == MAGEN || StatusCheck::m_cur_mode == MANUAL || StatusCheck::m_cur_mode == CYAN2 || StatusCheck::m_cur_mode == CYAN3) {
      if (buf[0] == GameHeader1 && buf[1] == GameHeader2 && buf[2] == GameHeader3 && buf[3] == GameHeader4)
      {
        //fprintf(stderr, "Siap masuk\n");
        if ((int)buf[4] == Version)
        {
          //fprintf(stderr, "Versi\n");
          if (buf[9] == Initial)
          {
            initial = true;
            ready = false;
            set1  = false;
            play  = false;
            //fprintf(stderr, "Inisial\n");

          }
          else if (buf[9] == Ready)
          {
            initial = false;
            ready = true;
            set1  = false;
            play  = false;
            //fprintf(stderr, "Ready\n");
          }
          else if (buf[9] == Set)
          {
            initial = false;
            ready = false;
            set1  = true;
            play  = false;
            //fprintf(stderr, "Set\n");
          }
          else if (buf[9] == Play)
          {
            initial = false;
            ready = false;
            set1  = false;
            play  = true;
            //fprintf(stderr, "Play\n");
          }
        }
      }
    }
  }
}