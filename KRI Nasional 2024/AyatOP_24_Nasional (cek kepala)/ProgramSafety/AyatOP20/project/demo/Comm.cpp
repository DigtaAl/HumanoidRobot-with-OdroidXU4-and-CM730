/*
 * Comm.cpp
 *
 *  Created on: 2024. 6. 17.
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

#include <darwin/framework/Comm.h>
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

const int PORT_COM			= 6666;

int receivedNumber;
int numberToSend 	= 0;
bool shouldSend 	= true;
bool cekSend1 		= false;
bool cekSend2 		= false;

void* sendInt(void* arg) {
    int clientSocket = *((int*)arg);
    while (true) {
        if (shouldSend == true) {
            send(clientSocket, &numberToSend, sizeof(numberToSend), 0);
            std::cout << "Sent: " << numberToSend << std::endl;
            shouldSend = false;
        }
        usleep(1000000);
    }

    pthread_exit(NULL);
}

void* receiveInt(void* arg) {
    int clientSocket = *((int*)arg);
    while (true) {
        int receivedInt;
        int bytesRead = recv(clientSocket, &receivedInt, sizeof(receivedInt), 0);
        if (bytesRead <= 0) {
            std::cerr << "Server disconnected.\n";
            break;
        }
		receivedNumber = receivedInt;
        std::cout << "Received : " << receivedInt << std::endl;
    }
    pthread_exit(NULL);
}

void* connectToServer(void* arg) {
    int clientSocket;
    while (true) {
        clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket == -1) {
            std::cerr << "Error membuat socket\n";
            return NULL;
        }
        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(PORT_COM);
        inet_pton(AF_INET, IP_COM, &serverAddr.sin_addr);
        if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
            //std::cerr << "Error menghubungkan ke server: " << strerror(errno) << "\n";
            close(clientSocket);
            usleep(5000000); // Menunggu 5 detik sebelum mencoba menghubungkan lagi
        } else {
            std::cout << "Terhubung ke server\n";
            break; // Keluar dari loop jika terhubung
        }
    }
    pthread_t sendThread, receiveThread;

    pthread_create(&sendThread, NULL, sendInt, &clientSocket);
    pthread_create(&receiveThread, NULL, receiveInt, &clientSocket);

    pthread_join(sendThread, NULL);
    pthread_join(receiveThread, NULL);

    // Tutup socket setelah thread selesai
    close(clientSocket);
    return NULL;
}