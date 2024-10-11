/*
 * Interface.h
 *
 *  Created on: 2024. 6. 17.
 *      Author: Digta
 */

#ifndef INTERFACE_H
#define INTERFACE_H

extern char buf2[32];
extern int baca;
extern int mag;
extern int mode;
extern int arahValue;
extern int buttonValue;

namespace Robot {

class Interface {
public:
    static void setupSerial(int &fd);
    static void controlLed(int &fd, int mode);
    static void IMUData();
    static void *thread_function(void *n);
};

} // namespace Robot

#endif // INTERFACE_H
