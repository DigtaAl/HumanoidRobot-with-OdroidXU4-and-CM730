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
int startt = 0;

using namespace Robot;

int StatusCheck::m_cur_mode     = CYAN;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;
int StatusCheck::cek_kondisi	= 0;
int StatusCheck::mulai	= 0;
int StatusCheck::waktu	= 0;

void jalantempatbangun1()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 600;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=10;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=0;
}
void jalantempatbangun2()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	Walking::GetInstance()->PERIOD_TIME = 650;
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 20;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
	
}
void jalantempatbangun3()
{
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->Start();
	//Walking::GetInstance()->HIP_PITCH_OFFSET = 31.0;
	Walking::GetInstance()->PERIOD_TIME = 685;
	Walking::GetInstance()->BALANCE_ENABLE =true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE= 30;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 2;
	Walking::GetInstance()->X_MOVE_AMPLITUDE= 0;
}
void prepjalan()
{
		Head::GetInstance()->MoveToHome();
		MotionManager::GetInstance()->SetEnable(true);
		Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
		Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
		jalantempatbangun1();
		usleep(1000000);
		jalantempatbangun2();
		usleep(1000000);
		jalantempatbangun3();
		usleep(1000000);
}
void StatusCheck::Check(CM730 &cm730)
{
    if(MotionStatus::FALLEN != STANDUP && (m_cur_mode == CYAN || m_cur_mode == MAGEN || m_cur_mode == MANUAL) && (m_is_started == 1 || m_is_started == 0))
    {
		std::cout<<"Praaaa Bangunnnnnn"<<std::endl;
        Walking::GetInstance()->Stop();
        while(Walking::GetInstance()->IsRunning() == 1) usleep(8000);

        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

        if(MotionStatus::FALLEN == FORWARD){
			std::cout<<"Bangunnnnnn Depan"<<std::endl;
            Action::GetInstance()->Start(115);   // FORWARD GETUP
			while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
			prepjalan();
		}
        else if(MotionStatus::FALLEN == BACKWARD){
            std::cout<<"Bangunnnnnn Belakang"<<std::endl;
            Action::GetInstance()->Start(117);   // BACKWARD GETUP
			while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
			prepjalan();
        }
        else if(MotionStatus::FALLEN == RIGHT){
            std::cout<<"Bangunnnnnn Kanan"<<std::endl;
            Action::GetInstance()->Start(118);   // RIGHT GETUP
			while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
			Action::GetInstance()->Start(115);   // FORWARD GETUP
			while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
			prepjalan();
        }
        else if(MotionStatus::FALLEN == LEFT){
            std::cout<<"Bangunnnnnn Kiri"<<std::endl;
            Action::GetInstance()->Start(119);   // LEFT GETUP
			while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
			Action::GetInstance()->Start(115);   // FORWARD GETUP
			while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
			prepjalan();
        }

        while(Action::GetInstance()->IsRunning() == 1) usleep(8000);

        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    }


    if(m_old_btn == MotionStatus::BUTTON)
        return;

    m_old_btn = MotionStatus::BUTTON;
    
    if (m_old_btn & BTN_MODE)
    {
		//urut++;
	}
	if (m_old_btn & BTN_START)
    {
		startt = 2;
		mulai = 1;
		cek_kondisi = 1;
		std::cout<<"Cek Kondisi :"<<cek_kondisi<<std::endl;
	}

    if(m_old_btn & BTN_MODE)
    {
        fprintf(stderr, "Mode button pressed.. \n");
        //StatusCheck::waktu	= 0;
        MotionManager::GetInstance()->Reinitialize();
        MotionManager::GetInstance()->SetEnable(true);
        //LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");
		Action::GetInstance()->m_Joint.SetEnableBody(true, true);

        Action::GetInstance()->Start(14);
        while(Action::GetInstance()->IsRunning() == true) usleep(8000);
        //m_is_started=0;
        
        if(m_is_started == 1)
        {
            m_is_started    = 0;
            m_cur_mode      = CYAN;
            LinuxActionScript::m_stop = 1;

            Walking::GetInstance()->Stop();
            Action::GetInstance()->m_Joint.SetEnableBody(true, true);

            while(Action::GetInstance()->Start(14) == false) usleep(8000);
            while(Action::GetInstance()->IsRunning() == true) usleep(8000);
        }
        else
        {
            m_cur_mode++;
            if(m_cur_mode >= MAX_MODE) m_cur_mode = CYAN;
        }

        MotionManager::GetInstance()->SetEnable(false);
        usleep(10000);

        if(m_cur_mode == CYAN)
        {
			fprintf(stderr, "Mode CYAN.. \n");
			MotionManager::GetInstance()->Reinitialize();
			MotionManager::GetInstance()->SetEnable(true);
			Action::GetInstance()->m_Joint.SetEnableBody(true, true);
			Action::GetInstance()->Start(14);
			while(Action::GetInstance()->IsRunning() == true) usleep(8000);
            fprintf(stderr, "Mode CYAN selesai.. \n");

        }
        else if(m_cur_mode == MAGEN)
        {
			fprintf(stderr, "Mode MAGEN1.. \n");
			MotionManager::GetInstance()->Reinitialize();
			MotionManager::GetInstance()->SetEnable(true);
			Action::GetInstance()->m_Joint.SetEnableBody(true, true);
            Action::GetInstance()->Start(62);
			while(Action::GetInstance()->IsRunning() == true) usleep(8000);
            fprintf(stderr, "Mode MAGEN1 selesai.. \n");
            
        }
        else if(m_cur_mode == MANUAL)
        {
			fprintf(stderr, "Mode MANUAL.. \n");
			MotionManager::GetInstance()->Reinitialize();
			MotionManager::GetInstance()->SetEnable(true);
			Action::GetInstance()->m_Joint.SetEnableBody(true, true);
            Action::GetInstance()->Start(64);
            while(Action::GetInstance()->IsRunning() == true) usleep(8000);
            fprintf(stderr, "Mode MANUAL selesai.. \n");
            
        }
    }
	
	//if (urut == 2)
    if(startt == 2)
    {
        if(m_is_started == 0)
        {
            fprintf(stderr, "Start button pressed.. & started is false.. \n");
            startt=0;
            cek_kondisi = 0;
            StatusCheck::waktu	= 1000000;
			//m_cur_mode = CYAN;
            if(m_cur_mode == CYAN)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                m_is_started = 1;
                //LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");

                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                //Action::GetInstance()->Start(9);
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
				fprintf(stderr, "CYAN \n");
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
            else if(m_cur_mode == MAGEN)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                m_is_started = 1;
                //LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");

                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                //Action::GetInstance()->Start(9);
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
				fprintf(stderr, "MAGEN \n");
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
			else if(m_cur_mode == MANUAL)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                m_is_started = 1;
                //LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");

                Action::GetInstance()->m_Joint.SetEnableBody(true, true);

                //Action::GetInstance()->Start(9);
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
				fprintf(stderr, "MANUAL \n");
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
            
        }
        else
        {
            fprintf(stderr, "Start button pressed.. & started is true.. \n");
            Walking::GetInstance()->Stop();
            MotionManager::GetInstance()->Reinitialize();
			MotionManager::GetInstance()->SetEnable(true);
			Action::GetInstance()->m_Joint.SetEnableBody(true, true);
			Action::GetInstance()->Start(14);
			while(Action::GetInstance()->IsRunning() == true) usleep(8000);
        }
    }
}
