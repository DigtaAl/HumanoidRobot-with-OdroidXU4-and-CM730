/*
 * StatusCheck.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#ifndef STATUSCHECK_H_
#define STATUSCHECK_H_

#include <darwin/framework/ArbotixPro.h>

namespace Robot
{
    enum {
        INITIAL,
        READY,
        SOCCER,
        MOTION,
        VISION,
        MAX_MODE
    };

    enum {
        BTN_MODE = 1,
        BTN_START = 2
    };

    class StatusCheck {
    private:
        

    public:
        static int m_cur_mode;
        static int m_is_started;
		static int m_old_btn;
		static int cek_kondisi;
		static int waktu;
        static void Check(ArbotixPro &arbotixpro);
    };
}

#endif /* STATUSCHECK_H_ */
