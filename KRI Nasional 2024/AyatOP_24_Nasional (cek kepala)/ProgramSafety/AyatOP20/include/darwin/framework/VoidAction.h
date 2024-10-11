/*
 * VoidAction.h
 *
 *  Created on: 2024. 4. 20.
 *      Author: Digta
 */

#ifndef VOIDACTION_H
#define VOIDACTION_H

#define WALK_READY 	55
#define R_KICK 	    63 // Right Kick Motion
#define L_KICK 	    63 // Left Kick Motion
#define RS_KICK     59 // Right Side Kick Motion
#define LS_KICK 	60 // Left Side Kick Motion
#define F_UP    	57 // Front Get up Motion
#define B_UP 	    58 // Back Get up Motion
#define RS_UP 	    61 // Right Side Get up  Motion
#define LS_UP    	62 // Left Side Get Up Motion

// Syarat Awal //
extern bool cyan;
extern bool magen;
extern bool jalanawalmagen;
extern bool manualll;
extern int i;

void sighandler(int sig);
void change_current_dir();
void cetak(const char *mystring);

void Tendang();
void netral1();
void luruskiri();
void kekiri2();
void luruskanan();
void kekanan2();
void putarkepalaMagen();
void jalantempat();
void jalantempat1();
void jalantempat2();
void jalantempat22();
void jalantempatawal();
void maju();
void majuu();
void majuuu();
void majuuuu();
void ReadyS1();
void ReadyS2();
void ReadyS3();
void jalantempatwhs();
void jalantempatwhs1();
void jalantempatwhs2();
void jalantempatwhs3();
void JalanCyanAsli();
void JalanAwal();
void jalantempatbangun1();
void jalantempatbangun2();
void jalantempatbangun3();
void prepjalan();
void muterkepalaKanan();
void muterkepalaKiri();

#endif // VOIDACTION_H
