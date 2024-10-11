/*
 *   BallFollower.h
 *
 *   Author: 
 *	 Editor: Digta
 *
 */

#ifndef _BALL_FOLLOWER_H_
#define _BALL_FOLLOWER_H_

#include <darwin/framework/Point.h>
#include <darwin/framework/BallTracker.h>

namespace Robot
{
	class BallFollower
	{
	private:
		int m_NoBallMaxCount;
		int m_NoBallCount;
		int m_KickBallMaxCount;
		int m_KickBallCount;
		int move;
		int areabola;

		bool jalanCyan 			= true;
		bool jalanMagen 		= true;
		bool jalanManual 		= true;
		bool puterkepalaMagen 	= true;
		
		double 	posX;
		double 	posY;
		double	arahX;
		double	arahY;
		double 	centerX;
		double 	centerY;
		double 	offsetX;
		double	offsetY;

		double m_MaxFBStep;
		double m_MaxRLStep;
		double m_MaxDirAngle;

		double m_KickTopAngle;
		double m_KickRightAngle;
		double m_KickLeftAngle;

		double m_FollowMaxFBStep;
        double m_FollowMinFBStep;
		double m_FollowMaxRLTurn;
        double m_FitFBStep;
		double m_FitMaxRLTurn;
		double m_UnitFBStep;
		double m_UnitRLTurn;
		
		double m_GoalFBStep;
		double m_GoalRLTurn;
		double m_FBStep;
		double m_RLTurn;
		
		int yaw_tolerance;

	protected:

	public:
		bool DEBUG_PRINT;
		int KickBall;		// 0: No ball 1:Left -1:Right
		bool magen;
		bool cyan;
		bool C1;

		BallFollower();
		~BallFollower();
		int NoBallCount;
		static const int NoBallMaxCount = 15;
		
		bool m_trackingBall;
		//Point2D	ball_position;
		void Process();
		void Tracker();	
	};
}

#endif
