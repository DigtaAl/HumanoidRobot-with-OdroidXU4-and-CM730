/*
   StatusCheck.cpp

    Created on: 2011. 1. 21.
        Author:
        Editor: Digta
*/

#include <stdio.h>
#include <unistd.h>

#include <darwin/framework/Head.h>
#include <darwin/framework/Action.h>
#include <darwin/framework/Walking.h>
#include <darwin/framework/Konstanta.h>
#include <darwin/framework/Interface.h>
#include <darwin/framework/VoidAction.h>
#include <darwin/framework/StatusCheck.h>
#include <darwin/framework/BallFollower.h>
#include <darwin/framework/MotionStatus.h>
#include <darwin/framework/MotionManager.h>
#include <darwin/linux/LinuxActionScript.h>

using namespace Robot;

int StatusCheck::m_cur_mode     = INITIAL;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;
int StatusCheck::cek_kondisi    = 0;
int StatusCheck::waktu          = 0;
int StatusCheck::trigready      = 0;

int urut              = 0;
bool wasButtonPressed = false;

void StatusCheck::Check(CM730 &cm730)
{
  if (MotionStatus::FALLEN != STANDUP && (m_cur_mode == CYAN || m_cur_mode == MAGEN || m_cur_mode == MANUAL) && (m_is_started == 1 || m_is_started == 0))
  {
    std::cout << "Praaaa Bangunnnnnn" << std::endl;
    Walking::GetInstance()->Stop();
    while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);

    if (MotionStatus::FALLEN == FORWARD) {
      std::cout << "Bangunnnnnn Depan" << std::endl;
      Action::GetInstance()->Start(F_UP);   // FORWARD GETUP
      while (Action::GetInstance()->IsRunning() == 1) usleep(8000);
      VoidAction::prepjalan();
    }
    else if (MotionStatus::FALLEN == BACKWARD) {
      std::cout << "Bangunnnnnn Belakang" << std::endl;
      Action::GetInstance()->Start(B_UP);   // BACKWARD GETUP
      while (Action::GetInstance()->IsRunning() == 1) usleep(8000);
      VoidAction::prepjalan();
    }
    else if (MotionStatus::FALLEN == RIGHT) {
      std::cout << "Bangunnnnnn Kanan" << std::endl;
      Action::GetInstance()->Start(RS_UP);   // RIGHT GETUP
      while (Action::GetInstance()->IsRunning() == 1) usleep(8000);
      VoidAction::prepjalan();
    }
    else if (MotionStatus::FALLEN == LEFT) {
      std::cout << "Bangunnnnnn Kiri" << std::endl;
      Action::GetInstance()->Start(LS_UP);   // LEFT GETUP
      while (Action::GetInstance()->IsRunning() == 1) usleep(8000);
      VoidAction::prepjalan();
    }

    while (Action::GetInstance()->IsRunning() == 1) usleep(8000);

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
  }

  //std::cout<<"Dari status check button"<<buttonValue<<std::endl;
  //std::cout<<"urut : "<<urut<<std::endl;

  //if(m_old_btn == MotionStatus::BUTTON)
  //return;

  m_old_btn = buttonValue;

  if (m_old_btn & BTN_MODE) {
    m_is_started = 0;
  }

  if (m_old_btn & BTN_START)
  {
    trigready = 1;
    if (mode == 3) {
      m_cur_mode      = MANUAL;
    }
  }

  if (mode == 1) {
    m_cur_mode      = CYAN;
  }
  else if (mode == 2) {
    m_cur_mode      = MAGEN;
  }
  else if (mode == 4) {
    m_cur_mode      = CYAN2;
  }
  else if (mode == 5) {
    m_cur_mode      = CYAN3;
  }

  if (m_is_started == 0) {
    if (m_cur_mode == CYAN) {
      MotionManager::GetInstance()->Reinitialize();
      MotionManager::GetInstance()->SetEnable(true);
      m_is_started = 1;

      Action::GetInstance()->m_Joint.SetEnableBody(true, true);

      while (Action::GetInstance()->IsRunning() == true) usleep(8000);
      //fprintf(stderr, "CYAN \n");
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

      MotionManager::GetInstance()->ResetGyroCalibration();
      while (1)
      {
        if (MotionManager::GetInstance()->GetCalibrationStatus() == 1)
        {
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
          break;
        }
        else if (MotionManager::GetInstance()->GetCalibrationStatus() == -1)
        {
          //LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
          MotionManager::GetInstance()->ResetGyroCalibration();
        }
        usleep(8000);
      }
    }
    else if (m_cur_mode == MAGEN) {
      MotionManager::GetInstance()->Reinitialize();
      MotionManager::GetInstance()->SetEnable(true);
      m_is_started = 1;

      Action::GetInstance()->m_Joint.SetEnableBody(true, true);

      while (Action::GetInstance()->IsRunning() == true) usleep(8000);
      //fprintf(stderr, "MAGEN \n");
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

      MotionManager::GetInstance()->ResetGyroCalibration();
      while (1)
      {
        if (MotionManager::GetInstance()->GetCalibrationStatus() == 1)
        {
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
          break;
        }
        else if (MotionManager::GetInstance()->GetCalibrationStatus() == -1)
        {
          //LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
          MotionManager::GetInstance()->ResetGyroCalibration();
        }
        usleep(8000);
      }
    }
    else if (m_cur_mode == MANUAL) {
      MotionManager::GetInstance()->Reinitialize();
      MotionManager::GetInstance()->SetEnable(true);
      m_is_started = 1;

      Action::GetInstance()->m_Joint.SetEnableBody(true, true);

      while (Action::GetInstance()->IsRunning() == true) usleep(8000);
      //fprintf(stderr, "MANUAL \n");
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

      MotionManager::GetInstance()->ResetGyroCalibration();
      while (1)
      {
        if (MotionManager::GetInstance()->GetCalibrationStatus() == 1)
        {
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
          break;
        }
        else if (MotionManager::GetInstance()->GetCalibrationStatus() == -1)
        {
          //LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
          MotionManager::GetInstance()->ResetGyroCalibration();
        }
        usleep(8000);
      }
    }
    else if (m_cur_mode == CYAN2) {
      MotionManager::GetInstance()->Reinitialize();
      MotionManager::GetInstance()->SetEnable(true);
      m_is_started = 1;

      Action::GetInstance()->m_Joint.SetEnableBody(true, true);

      while (Action::GetInstance()->IsRunning() == true) usleep(8000);
      //fprintf(stderr, "MAGEN \n");
      Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
      Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

      MotionManager::GetInstance()->ResetGyroCalibration();
      while (1)
      {
        if (MotionManager::GetInstance()->GetCalibrationStatus() == 1)
        {
          //LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
          break;
        }
        else if (MotionManager::GetInstance()->GetCalibrationStatus() == -1)
        {
          //LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
          MotionManager::GetInstance()->ResetGyroCalibration();
        }
        usleep(8000);
      }
    }
  }
}
