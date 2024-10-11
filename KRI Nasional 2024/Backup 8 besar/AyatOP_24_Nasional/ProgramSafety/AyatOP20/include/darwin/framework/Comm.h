/*
 * Comm.h

 *
 *  Created on: 2024. 6. 17.
 *      Author: Digta
 */

#ifndef COMM_H
#define COMM_H

#define IP_COM   			"192.168.104.184"
#define IP_LOCAL   			"127.0.0.1"

extern int receivedNumber;
extern int numberToSend;
extern bool shouldSend;
extern bool cekSend1;
extern bool cekSend2;
extern int PORT_COM;

namespace Robot {

class Comm {
public:
    static void* sendInt(void* arg);
    static void* receiveInt(void* arg);
    static void* connectToServer(void* arg);

private:
    
};

} // namespace Robot

#endif // COMM_H
