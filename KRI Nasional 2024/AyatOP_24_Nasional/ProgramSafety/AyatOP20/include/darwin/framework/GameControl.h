/*
 * GameControl.cpp
 *
 *  Created on: 2024. 6. 17.
 *      Author: Digta
 */

#ifndef GAMECONTROL_H
#define GAMECONTROL_H

/////////////////////////// Konstanta ////////////////////////////////
#define BUFLEN 	30		// Max length of buffer 1024
#define PORT 	3838	// The port on which to listen for incoming data

//const char header1 = 85, header2 = 78, header3 = 89;//header1-header2-heaader3 = U-N-Y
extern const char GameHeader1, GameHeader2, GameHeader3, GameHeader4; 
extern const int  Version;
extern char buf[BUFLEN];
extern int s,recv_len;
extern struct sockaddr_in si_me, si_other;	
extern socklen_t slen;

extern bool manual;
extern bool initial;
extern bool ready;
extern bool play;
extern bool set1;

namespace Robot {

class GameControl {
public:
    static void waitData();
    static void *wifi(void *n);
};

} // namespace Robot

#endif // GAMECONTROL_H
