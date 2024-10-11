/*
 * VoidAction.h
 *
 *  Created on: 2024. 4. 20.
 *      Author: Digta
 */

#ifndef VOIDACTION_H
#define VOIDACTION_H

#define WALK_READY 	55
#define R_KICK 	    65 // Right Kick Motion
#define L_KICK 	    66 // Left Kick Motion
#define RS_KICK     59 // Right Side Kick Motion
#define LS_KICK 	60 // Left Side Kick Motion
#define F_UP    	57 // Front Get up Motion
#define B_UP 	    58 // Back Get up Motion
#define RS_UP 	    61 // Right Side Get up  Motion
#define LS_UP    	62 // Left Side Get Up Motion

namespace Robot {

class VoidAction {
public:
    // Syarat Awal //
    static int i, firstkick_;
    static bool cyan;
    static bool magen;
    static bool jalanawalmagen;
    static bool manualll;
    static bool majuLurus;
    static bool majuArahKiri;
    static bool tendangSKiri;
    static bool majuArahKanan;
    static bool tendangSKanan;
    static bool searchMuter;
    static bool prepwalking;

    // Function prototypes
    static void sighandler(int sig);
    static void change_current_dir();
    static void cetak(const char *mystring);
    static void updateBallPosition(int bataskiri, int bataskanan, int belakang);
    static void handleKick(double pan2, double mid);

    static void Tendang();
    static void netral();
    static void arahkanan();
    static void arahkiri();
    static void prepluruskiri();
    static void prep1luruskiri();
    static void prep2luruskiri();
    static void prep3luruskiri();
    static void cobaluruskiri();
    static void revprepluruskiri();
    static void luruskiri();
    static void kekiri2();
    static void cobaluruskanan();
    static void prepluruskanan();
    static void prep1luruskanan();
    static void prep2luruskanan();
    static void prep3luruskanan();
    static void revprepluruskanan();
    static void luruskanan();
    static void kekanan2();
    static void putarkepalaMagen();
    static void jalantempat();
    static void jalantempat1();
    static void jalantempat2();
    static void jalantempat22();
    static void jalantempatawal();
    static void jalantempatmuter();
    static void maju();
    static void majuu();
    static void majuuu();
    static void majuuuu();
    static void ReadyS1();
    static void ReadyS2();
    static void ReadyS3();
    static void jalantempatwhs();
    static void jalantempatwhs1();
    static void jalantempatwhs2();
    static void jalantempatwhs3();
    static void jalantempatwhs4();
    static void JalanCyanAsli();
    static void JalanCyanBaru();
    static void JalanCyanMirror();
    static void JalanCyanluruskiri();
    static void JalanAwal();
    static void jalantempatbangun1();
    static void jalantempatbangun2();
    static void jalantempatbangun3();
    static void prepjalan();
    static void muterkepalaKanan();
    static void muterkepalaKiri();
};

} // namespace Robot

#endif // VOIDACTION_H
