#include "WPILib.h"
#include "DashboardDataFormat.h"

/**
 * This is a demo program showing the use of the Dashboard data packing class.
 */ 
class DashboardDataExample : public SimpleRobot
{
	DashboardDataFormat dashboardDataFormat;

public:
	DashboardDataExample(void)
	{
		GetWatchdog().SetExpiration(0.1);
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void RobotMain(void)
	{
		GetWatchdog().SetEnabled(false);
		
		while (true)
		{
			GetWatchdog().Feed();
			dashboardDataFormat.SendIOPortData();
			dashboardDataFormat.SendVisionData();
			Wait(1.0);
		}
	}
};

START_ROBOT_CLASS(DashboardDataExample);

