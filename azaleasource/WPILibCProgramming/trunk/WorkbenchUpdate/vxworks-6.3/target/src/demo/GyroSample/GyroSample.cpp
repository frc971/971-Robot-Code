#include "WPILib.h"

/**
 * This is a demo program showing the use of the Gyro for driving in a straight
 * line. The Gyro object in the library returns the heading with 0.0 representing
 * straight and positive or negative angles in degrees for left and right turns.
 * The zero point is established when the gyro object is created so it is important
 * to have the robot still after turning it on.
 * 
 * If you are experiencing drift in the gyro, it can always be reset back to the
 * zero heading by calling Gyro.Reset().
 * 
 * Notice that there is no OperatorControl method. Since this program doesn't supply
 * one the default OperatorControl method will be called from the library.
 */
class GyroSample : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Gyro gyro;

public:
	GyroSample(void):
		myRobot(1, 2),		// initialize the sensors in initialization list
		gyro(1)
	{
		myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive in a straight line using the gyro.
	 * This short program uses simple proportional to correct for errors in heading.
	 * Proportional control simply means that the amount of correction to the heading is
	 * proportional to the error. Since we are trying to drive straight, then the
	 * desired gyro value is 0.0. The gyro heading, which varies from 0.0 as the robot
	 * turns is supplied as the curve parameter to the Drive method. This method takes
	 * values from -1 to 1 to represent turns left and right. 0 is no turn.
	 * 
	 * While the gyro heading is 0.0 (the desired heading) there will be no turn. As the
	 * heading increases in either direction, it results in a proportionally larger
	 * turn to counteract the error. The angle is divided by 30 to scale the value so
	 * the robot doesn't turn too fast or too slowly while correcting.
	 */
	void Autonomous(void)
	{
		gyro.Reset();
		while (IsAutonomous())
		{
			float angle = gyro.GetAngle();			// current heading (0 = target)
			myRobot.Drive(-1.0, -angle / 30.0);		// proportionally drive in a straight line
			Wait(0.004);
		}
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}
	
	void OperatorControl(void)
	{
		Joystick stick(1);
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick);
			Wait(0.005);
		}
	}
};

START_ROBOT_CLASS(GyroSample);
