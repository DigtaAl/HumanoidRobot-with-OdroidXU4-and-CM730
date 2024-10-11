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

void IMUData();
void *thread_function(void *n);

#endif // INTERFACE_H
