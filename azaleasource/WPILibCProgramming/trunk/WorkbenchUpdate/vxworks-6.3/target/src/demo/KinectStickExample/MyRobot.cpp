#include "WPILib.h"

/**
 * This code demonstrates the use of the KinectStick
 * class to drive your robot during the autonomous mode
 * of the match, making it a hybrid machine. The gestures
 * used to control the KinectStick class are described in the
 * "Getting Started with the Microsoft Kinect for FRC" document
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	KinectStick leftArm;	//The Left arm should be constructed as stick 1
	KinectStick rightArm; 	//The Right arm should be constructed as stick 2
	Joystick stick;			//Joystick for teleop control

public:
	RobotDemo(void):
		myRobot(1, 2),	// these must be initialized in the same order
		leftArm(1),		// as they are declared above.
		rightArm(2),
		stick(1)
	{
		myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		bool exampleButton;
	
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while(IsAutonomous()){
            /**
             * KinectStick axis values are accessed identically to those of a joystick
             * In this example the axis values have been scaled by ~1/3 for safer
             * operation when learning to use the Kinect.
             */
            myRobot.TankDrive(leftArm.GetY()*.33, rightArm.GetY()*.33);

            /* An alternative illustrating that the KinectStick can be used just like a Joystick */
            //myRobot.TankDrive(leftArm, rightArm);

            /*Example illustrating that accessing buttons is identical to a Joystick */
            exampleButton = leftArm.GetRawButton(1);

            Wait(.01); /* Delay 10ms to reduce processing load */
		}
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

