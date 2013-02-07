
#include "WPILib.h"

/**
 * This example shows how you can write text to the LCD on the driver station.
 */ 
class DriverStationLCDTextExample : public SimpleRobot
{

public:
	DriverStationLCDTextExample(void)
	{
	}

	void RobotMain()
	{
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();

		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Hello World");
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 11, "Time: %4.1f", GetClock());
		dsLCD->UpdateLCD();

		Wait(0.1);
	}
};

START_ROBOT_CLASS(DriverStationLCDTextExample);

