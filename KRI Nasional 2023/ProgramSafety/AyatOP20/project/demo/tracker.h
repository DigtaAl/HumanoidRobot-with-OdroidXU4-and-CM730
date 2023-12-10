#ifndef _TRACKER_H_
#define _TRACKER_H_

namespace Robot
{
	class trackers
	{
	private:
		int m_NoBallMaxCount;
		int m_NoBallCount;
		int m_KickBallMaxCount;
		int m_KickBallCount;
		int move;

		int NoBallCount;
		static const int NoBallMaxCount = 15;

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
		trackers();
		~trackers();
		void Process();
		
	};
}

#endif
