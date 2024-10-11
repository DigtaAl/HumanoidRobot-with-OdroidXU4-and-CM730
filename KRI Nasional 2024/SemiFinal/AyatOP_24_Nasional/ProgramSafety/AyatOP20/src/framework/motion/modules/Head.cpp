/*
 *   Head.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <iostream>
#include <darwin/framework/Head.h>
#include <darwin/framework/MX28.h>
//#include <darwin/framework/RX28M.h>
#include <darwin/framework/Kinematics.h>
#include <darwin/framework/MotionStatus.h>

using namespace Robot;
using namespace std;

Head* Head::m_UniqueInstance = new Head();

Head::Head()
{
	m_Pan_p_gain 	= 0.1;	//0.1;
	m_Pan_d_gain 	= 0.12;

    m_Tilt_p_gain 	= 0.1;	//0.1;
	m_Tilt_d_gain 	= 0.12;	//0.22;

	m_LeftLimit 	= 90;
	m_RightLimit 	= -90;
	
	//m_TopLimit = 20;
	//m_BottomLimit = -60;
	m_TopLimit 		= Kinematics::EYE_TILT_OFFSET_ANGLE;
	m_BottomLimit 	= Kinematics::EYE_TILT_OFFSET_ANGLE - 65;
	
	m_Pan_Home 		= 0.0;
	m_Tilt_Home 	= Kinematics::EYE_TILT_OFFSET_ANGLE - 30.0;
	//m_Tilt_Home 	= (m_BottomLimit + m_TopLimit)/2;
	
	m_Joint.SetEnableHeadOnly(true);
}

Head::~Head()
{
}

void Head::CheckLimit()
{
	if(m_PanAngle > m_LeftLimit)
		m_PanAngle = m_LeftLimit;
	else if(m_PanAngle < m_RightLimit)
		m_PanAngle = m_RightLimit;

	if(m_TiltAngle > m_TopLimit)
		m_TiltAngle = m_TopLimit;
	else if(m_TiltAngle < m_BottomLimit)
		m_TiltAngle = m_BottomLimit;	
}

void Head::Initialize()
{
	m_PanAngle 	= MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	m_TiltAngle = -MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	CheckLimit();

	InitTracking(1);
	MoveToHome();
}

void Head::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, HEAD_SECTION);
}

void Head::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "pan_p_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_p_gain = value;
    if((value = ini->getd(section, "pan_d_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_d_gain = value;
    if((value = ini->getd(section, "tilt_p_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_p_gain = value;
    if((value = ini->getd(section, "tilt_d_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_d_gain = value;
    if((value = ini->getd(section, "left_limit", INVALID_VALUE)) != INVALID_VALUE)  m_LeftLimit = value;
    if((value = ini->getd(section, "right_limit", INVALID_VALUE)) != INVALID_VALUE) m_RightLimit = value;
    if((value = ini->getd(section, "top_limit", INVALID_VALUE)) != INVALID_VALUE)   m_TopLimit = value;
    if((value = ini->getd(section, "bottom_limit", INVALID_VALUE)) != INVALID_VALUE)m_BottomLimit = value;
    if((value = ini->getd(section, "pan_home", INVALID_VALUE)) != INVALID_VALUE)    m_Pan_Home = value;
    if((value = ini->getd(section, "tilt_home", INVALID_VALUE)) != INVALID_VALUE)   m_Tilt_Home = value;
}

void Head::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, HEAD_SECTION);
}

void Head::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "pan_p_gain",   m_Pan_p_gain);
    ini->put(section,   "pan_d_gain",   m_Pan_d_gain);
    ini->put(section,   "tilt_p_gain",  m_Tilt_p_gain);
    ini->put(section,   "tilt_d_gain",  m_Tilt_d_gain);
    ini->put(section,   "left_limit",   m_LeftLimit);
    ini->put(section,   "right_limit",  m_RightLimit);
    ini->put(section,   "top_limit",    m_TopLimit);
    ini->put(section,   "bottom_limit", m_BottomLimit);
    ini->put(section,   "pan_home",     m_Pan_Home);
    ini->put(section,   "tilt_home",    m_Tilt_Home);
}

void Head::MoveToHome()
{
	MoveByAngle(m_Pan_Home, m_Tilt_Home);
}

void Head::MoveByAngle(double pan, double tilt)
{
	m_PanAngle 	= pan;
	m_TiltAngle = tilt;

	CheckLimit();
}

void Head::MoveByAngleOffset(double pan, double tilt)
{	
	MoveByAngle(m_PanAngle + pan, m_TiltAngle + tilt);
}

double time_move	= 0.5;  // 1 setting waktu putar kepala
bool Start			= true;
int move			= 1;
int move_value		= 3;
int pan_move		= 0;
int tilt_move		= 0;
int count			= 0;
int search_count	= 1;
int end_search		= 0;
int Searchingcount  = 0;

void Head::InitTracking(int start)
{
    // Settingan op master
	
	/*m_Pan_err = 0;
	m_Pan_err_diff = 0;
	m_Tilt_err = 0;
    m_Tilt_err_diff = 0;
    */ 
	
	// Settingan darwin lomba
	double pan 	= MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	
	if(Start == true){
		move			= start;
		end_search		= move-1;
		if(end_search < 1) end_search=6;
		Start			= false;
		search_count	= 0;
		putar			= 0;
	}
	
	if(search_count >= 2){
		if(start == 1 || start == 5) putar = 1; //kiri
		else putar = 2; //kanan
	}
	else if(move == end_search) {
		search_count++;
		end_search	= move-1;
	if(end_search<1) end_search=6;
	}
	
	if(move < 1) move = 1;
	if(move == 1){ // Left top
		if(pan > (m_LeftLimit - 3)) pan_move = 0; else pan_move = move_value;
		if(tilt > (m_TopLimit - 3)) tilt_move = 0; else tilt_move = move_value;
		if(pan > (m_LeftLimit - 3) && tilt > (m_TopLimit - 3)) move = 2;
	}
	else if(move == 2){ // Right top
		if(pan < (m_RightLimit + 3)) pan_move = 0; else pan_move =- move_value;
		if(tilt > (m_TopLimit - 3))tilt_move = 0; else tilt_move = move_value;
		if(pan < (m_RightLimit + 3) && tilt > (m_TopLimit - 3)) move = 3;
	}
	 else if(move == 3){ // Right center
		pan_move = 0;
		if(tilt > (m_Tilt_Home - 3)) tilt_move =- move_value;
		if(tilt < (m_Tilt_Home + 3)) tilt_move = move_value;
		if(tilt > (m_Tilt_Home - 3) && tilt < (m_Tilt_Home + 3)) move=4;
	}
	else if(move == 4){ // Left center
		tilt_move = 0;
		if(pan > (m_LeftLimit - 3)) move = 5; else pan_move = move_value;
	}
	else if(move==5){ // Left bottom
		if(pan > (m_LeftLimit - 3)) pan_move = 0; else pan_move = move_value;
		if(tilt < (m_BottomLimit + 3)) tilt_move = 0; else tilt_move =- move_value;
		if(pan > (m_LeftLimit - 3) && tilt < (m_BottomLimit + 3)) move = 6;
	
	}
	else if(move == 6){ // Right bottom
		if(pan < (m_RightLimit + 3)) pan_move = 0; else pan_move =- move_value;
		if(tilt < (m_BottomLimit + 3)) tilt_move = 0; else tilt_move =- move_value;
		if(move == 6 && pan < (m_RightLimit + 3) && tilt < (m_BottomLimit + 3)){
			move = 1;
			Searchingcount += 1;
		} 
		
	}
	
	if(count > time_move)
	{
		MoveByAngleOffset(pan_move,tilt_move);
		count = 0;
	}
	count++;
	//fprintf(stderr,"Move : %d - pan : %f - tilt : %f \n", move, pan, tilt); 
}

void Head::MoveTracking(double offsetX, double offsetY)
{	
	m_Pan_err_diff 	= offsetX - m_Pan_err;
	m_Pan_err 		= offsetX;

	m_Tilt_err_diff = offsetY - m_Tilt_err;
	m_Tilt_err 		= offsetY;
	
	MoveTracking();
}

void Head::MoveTracking()
{
	double pOffset, dOffset;

	pOffset = m_Pan_err * m_Pan_p_gain;
	pOffset *= pOffset;
	if(m_Pan_err < 0)
		pOffset = -pOffset;
	dOffset = m_Pan_err_diff * m_Pan_d_gain;
	dOffset *= dOffset;
	if(m_Pan_err_diff < 0)
		dOffset = -dOffset;
	m_PanAngle += (pOffset + dOffset);

	pOffset = m_Tilt_err * m_Tilt_p_gain;
	pOffset *= pOffset;
	if(m_Tilt_err < 0)
		pOffset = -pOffset;
	dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
	dOffset *= dOffset;
	if(m_Tilt_err_diff < 0)
		dOffset = -dOffset;
	m_TiltAngle += (pOffset + dOffset);

	CheckLimit();
}

int sample=0;
void Head::Process()
{
	
	if(m_Joint.GetEnable(JointData::ID_HEAD_PAN) == true)
		m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_PanAngle);

	if(m_Joint.GetEnable(JointData::ID_HEAD_TILT) == true)
	m_Joint.SetAngle(JointData::ID_HEAD_TILT, m_TiltAngle);
	
	//printf("pan: %f   tilt: %f \n",m_PanAngle, m_TiltAngle);	
	

}
