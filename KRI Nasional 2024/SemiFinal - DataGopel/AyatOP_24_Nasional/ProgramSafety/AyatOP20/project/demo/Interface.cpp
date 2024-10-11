/*
   Interface.cpp

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
#include <darwin/framework/Interface.h>
#include <darwin/framework/MotionStatus.h>
#include <darwin/framework/MotionManager.h>

using namespace Robot;
using namespace std;

#define BAUD_RATE B115200

//============= Imu Data ==================
int mag;
int baca;
int mode;
int arahValue;
int buttonValue;
char buf2[32];

void Interface::setupSerial(int &fd) {
  fd = open(U2D_DEV_Interface, O_RDWR | O_NOCTTY);
  if (fd == -1) {
    perror("open_port: Unable to open Interface port - ");
    exit(1);
  }

  struct termios toptions;
  tcgetattr(fd, &toptions);

  cfsetispeed(&toptions, BAUD_RATE);
  cfsetospeed(&toptions, BAUD_RATE);

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

  tcsetattr(fd, TCSANOW, &toptions);
}

void Interface::controlLed(int &fd, int mode) {
  char command = '0' + mode;  // Convert mode (1, 2, 3) to corresponding char ('1', '2', '3')
  write(fd, &command, 1);
}

void Interface::IMUData() {
  setupSerial(baca);
}

void *Interface::thread_function(void *n) {
  IMUData();

  while (true) {
    int bytesRead = read(baca, &buf2, sizeof(buf2));
    if (bytesRead > 0) {
      buf2[bytesRead] = '\0';

      if (strncmp(buf2, "B:", 2) == 0) {
        // Data tombol
        buttonValue = atoi(buf2 + 2);
        // Cycle through modes 1, 2, 3 when button is pressed
        if (buttonValue == 1) {
          mode = (mode % 5) + 1;
        }
        //std::cout << "Received Button: " << buttonValue << " | Mode: " << mode << std::endl;
      } else {
        // Data arah
        arahValue = atoi(buf2);
        mag = arahValue;
        std::cout << "Received IMU: " << mag << std::endl;
      }

      // Send command to control LED based on mode
      controlLed(baca, mode);
    }
  }
}