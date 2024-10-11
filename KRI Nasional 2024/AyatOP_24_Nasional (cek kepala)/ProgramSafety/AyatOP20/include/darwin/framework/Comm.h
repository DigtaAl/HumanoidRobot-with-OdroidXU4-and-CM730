/*
 * Comm.h

 *
 *  Created on: 2024. 6. 17.
 *      Author: Digta
 */

#ifndef COMM_H
#define COMM_H

#define IP_COM   			"192.168.0.119"
#define IP_LOCAL   			"127.0.0.1"
extern const int PORT_COM;

extern int receivedNumber;
extern int numberToSend;
extern bool shouldSend;
extern bool cekSend1;
extern bool cekSend2;

void* sendInt(void* arg);
void* receiveInt(void* arg);
void* connectToServer(void* arg);

#endif // COMM_H
