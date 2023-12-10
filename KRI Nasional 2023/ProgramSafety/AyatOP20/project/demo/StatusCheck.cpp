/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>

#include <darwin/framework/StatusCheck.h>
#include <darwin/framework/Head.h>
#include <darwin/framework/Action.h>
#include <darwin/framework/Walking.h>
#include <darwin/framework/MotionStatus.h>
#include <darwin/framework/MotionManager.h>
#include <darwin/linux/LinuxActionScript.h>

int urut=0;

using namespace Robot;

int StatusCheck::m_cur_mode     = INITIAL;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;
int StatusCheck::cek_kondisi	= 0;
int StatusCheck::waktu	= 0;

void StatusCheck::Check(CM730 &arbotixpro)
{
    /*if(MotionStatus::FALLEN != STANDUP && m_cur_mode == SOCCER && m_is_started == 1)
    {
        Walking::GetInstance()->Stop();
        while(Walking::GetInstance()->IsRunning() == 1) usleep(8000);

        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

        if(MotionStatus::FALLEN == FORWARD)
            Action::GetInstance()->Start(104);   // FORWARD GETUP
        else if(MotionStatus::FALLEN == BACKWARD)
            Action::GetInstance()->Start(106);   // BACKWARD GETUP

        while(Action::GetInstance()->IsRunning() == 1) usleep(8000);

        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    }
    */
     
    //if (MotionStatus::FALLEN != STANDUP && (m_cur_mode == WALKING || m_cur_mode == WALK_READY) && m_is_started == 1)
	if(MotionStatus::FALLEN != STANDUP && m_cur_mode == SOCCER && m_is_started == 1)	
		{
			Walking::GetInstance()->Stop();
			while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);

			Action::GetInstance()->m_Joint.SetEnableBody(true, true);
/*
			if (MotionStatus::FALLEN == FORWARD)
				{
				//Action::GetInstance()->Start(51);   // FORWARD GETUP 10
				arbotixpro.WriteByte(CM730::ID_CM, CM730::P_DXL_POWER,0,0);
				printf( "Robot has fallen forward.\n");
			    }
			else if (MotionStatus::FALLEN == BACKWARD)
				{
				//Action::GetInstance()->Start(52);   // BACKWARD GETUP 11
				arbotixpro.WriteByte(CM730::ID_CM, CM730::P_DXL_POWER,0,0);
				printf( "Robot has fallen backward.\n");
			    }
			     */
			while (Action::GetInstance()->IsRunning() == 1) usleep(8000);
			// Go back to Walk Ready
			//mWalkReady(arbotixpro);
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
		}

    if(m_old_btn == MotionStatus::BUTTON)
        return;

    m_old_btn = MotionStatus::BUTTON;
    
    if (m_old_btn & BTN_START)
    {
		urut+=1;
		
	}

    if(urut==1)
    {
        fprintf(stderr, "Mode button pressed.. \n");
        StatusCheck::waktu	= 0;
        MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                //LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");

                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                Action::GetInstance()->Start(14);
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
        m_is_started=0;

        if(m_is_started == 1)
        {
            m_is_started    = 0;
            m_cur_mode      = READY;
            LinuxActionScript::m_stop = 1;

            Walking::GetInstance()->Stop();
            Action::GetInstance()->m_Joint.SetEnableBody(true, true);

            while(Action::GetInstance()->Start(15) == false) usleep(8000);
            while(Action::GetInstance()->IsRunning() == true) usleep(8000);
        }
        else
        {
            m_cur_mode++;
            if(m_cur_mode >= MAX_MODE) m_cur_mode = READY;
        }

        MotionManager::GetInstance()->SetEnable(false);
        usleep(10000);

        if(m_cur_mode == READY)
        {
          //  arbotixpro.WriteByte(CM730::P_LED_PANNEL, 0x01, NULL);
           // arbotixpro.WriteWord(CM730::P_LED_HEAD_L, 0x20, NULL);
			//arbotixpro.WriteWord(CM730::P_LED_EYE_L, 0x20, NULL);
			//LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
        }
        else if(m_cur_mode == SOCCER)
        {
           // arbotixpro.WriteByte(CM730::P_LED_PANNEL, 0xFF, NULL);
			//arbotixpro.WriteWord(CM730::P_LED_HEAD_L, 0x01, NULL);
			//arbotixpro.WriteWord(CM730::P_LED_EYE_L, 0x01, NULL);
            //LinuxActionScript::PlayMP3("../../../Data/mp3/Autonomous soccer mode.mp3");
        }
        /*else if(m_cur_mode == MOTION)
        {
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x02, NULL);
            LinuxActionScript::PlayMP3("../../../Data/mp3/Interactive motion mode.mp3");
        }
        else if(m_cur_mode == VISION)
        {
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x04, NULL);
            LinuxActionScript::PlayMP3("../../../Data/mp3/Vision processing mode.mp3");
        }*/
    }

    if(urut==2)
    {
        if(m_is_started == 0)
        {
            fprintf(stderr, "Start button pressed.. & started is false.. \n");
            urut=0;
            StatusCheck::waktu	= 1000000;
			m_cur_mode = SOCCER;
            if(m_cur_mode == SOCCER)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                m_is_started = 1;
                //LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");

                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                //Action::GetInstance()->Start(9);
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
				fprintf(stderr, "Motion (9) Done \n");
				cek_kondisi++;
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                MotionManager::GetInstance()->ResetGyroCalibration();
                while(1)
                {
                    if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
                    {
                        //LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
                        break;
                    }
                    else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
                    {
                        //LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
                        MotionManager::GetInstance()->ResetGyroCalibration();
                    }
                    usleep(8000);
                }
            }
            /*else if(m_cur_mode == MOTION)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                m_is_started = 1;
                LinuxActionScript::PlayMP3("../../../Data/mp3/Start motion demonstration.mp3");

                // Joint Enable..
                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                Action::GetInstance()->Start(1);
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
            }
            else if(m_cur_mode == VISION)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                m_is_started = 1;
                LinuxActionScript::PlayMP3("../../../Data/mp3/Start vision processing demonstration.mp3");

                // Joint Enable...
                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                Action::GetInstance()->Start(1);
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
            }*/
        }
        else
        {
            fprintf(stderr, "Start button pressed.. & started is true.. \n");
        }
    }
}

/*void StatusCheck::mWalkReady(CM730 &arbotixpro)
{
	if (LinuxActionScript::m_is_running == 0)
		{
			if (m_is_started == 0)
				{
					arbotixpro.DXLPowerOn(true);
				}
			if (Walking::GetInstance()->IsRunning() == true)
				{
					Walking::GetInstance()->Stop();
					while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);
				}
			int lastMode = m_cur_mode;
			m_cur_mode = WALK_READY;
			printf("Robot is in WALK_READY state.\n");
			MotionManager::GetInstance()->Reinitialize();
			MotionManager::GetInstance()->SetEnable(true);
			m_is_started = 1;
			bLJState = bRJState = false;
			Head::GetInstance()->m_Joint.SetEnableBody(false);
			Walking::GetInstance()->m_Joint.SetEnableBody(false);
			Action::GetInstance()->m_Joint.SetEnableBody(true);

			Action::GetInstance()->Start(9); //9 WALK READY STANCE
			while (Action::GetInstance()->IsRunning() == true) usleep(8000);
			usleep(500);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);
			Action::GetInstance()->m_Joint.SetEnableBody(false);
			usleep(100);
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true);
		}
}

void StatusCheck::mAction(const char* scriptfilepath)
{
	if (LinuxActionScript::m_is_running == 0)
		{
			m_cur_mode = ACTION;
			printf("Robot is in ACTION state.\n");
			LinuxActionScript::m_stop = 0;
			m_is_started = 1;
			Head::GetInstance()->m_Joint.SetEnableBody(false);
			Walking::GetInstance()->m_Joint.SetEnableBody(false);
			Action::GetInstance()->m_Joint.SetEnableBody(true);
			LinuxActionScript::ScriptStart(scriptfilepath);
			while (Action::GetInstance()->IsRunning() == true) usleep(8000);
		}
}

void StatusCheck::mPlay(int motion_page, int mode, int wait)
{
	Walking::GetInstance()->Stop();
	while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);
	m_cur_mode = mode;
	MotionManager::GetInstance()->Reinitialize();
	MotionManager::GetInstance()->SetEnable(true);
	m_is_started = 1;

	Action::GetInstance()->m_Joint.SetEnableBody(true, true);

	Action::GetInstance()->Start(motion_page);
	if (wait == WAIT)
		{
			while (Action::GetInstance()->IsRunning() == true) usleep(8000);
			// if (mode != SITTING && mode != STAIRS)
			// 	{
			// 		Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			// 		Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			// 	}
		}
	return;
}
*/ 
