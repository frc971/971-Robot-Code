#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class DefaultRobot : public SimpleRobot
{
	RobotDrive *myRobot;			// robot drive system
	DigitalInput *armUpperLimit;	// arm upper limit switch
	DigitalInput *armLowerLimit;	// arm lower limit switch
	Joystick *rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick *leftStick;			// joystick 2 (tank left stick)
	Joystick *armStick;				// joystick 3 to control arm
	DriverStation *ds;				// driver station object

	enum							// Driver Station jumpers to control program operation
	{ ARCADE_MODE = 1,				// Tank/Arcade jumper is on DS Input 1 (Jumper present is arcade)
	  ENABLE_AUTONOMOUS = 2,		// Autonomous/Teleop jumper is on DS Input 2 (Jumper present is autonomous)
	} jumpers;	                            

public:
	/**
	 * 
	 * 
	 * Constructor for this robot subclass.
	 * Create an instance of a RobotDrive with left and right motors plugged into PWM
	 * ports 1 and 2 on the first digital module.
	 */
	DefaultRobot(void)
	{
		ds = DriverStation::GetInstance();
		myRobot = new RobotDrive(1, 3, 2, 4);	// create robot drive base
		rightStick = new Joystick(1);			// create the joysticks
		leftStick = new Joystick(2);
		armStick = new Joystick(3);
		armUpperLimit = new DigitalInput(1);	// create the limit switch inputs
		armLowerLimit = new DigitalInput(2);
		//Update the motors at least every 100ms.
		myRobot->SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds, enabled by a jumper (jumper
	 * must be in for autonomous to operate).
	 */
	void Autonomous(void)
	{
		myRobot->SetSafetyEnabled(false);
		if (ds->GetDigitalIn(ENABLE_AUTONOMOUS) == 1)	// only run the autonomous program if jumper is in place
		{
			myRobot->Drive(0.5, 0.0);			// drive forwards half speed
			Wait(2.0);							//    for 2 seconds
			myRobot->Drive(0.0, 0.0);			// stop robot
		}
		myRobot->SetSafetyEnabled(true);
	}

	/**
	 * Runs the motors under driver control with either tank or arcade steering selected
	 * by a jumper in DS Digin 0. Also an arm will operate based on a joystick Y-axis. 
	 */
	void OperatorControl(void)
	{
		Victor armMotor(5);						// create arm motor instance
		while (IsOperatorControl())
		{
			// determine if tank or arcade mode; default with no jumper is for tank drive
			if (ds->GetDigitalIn(ARCADE_MODE) == 0) {	
				myRobot->TankDrive(leftStick, rightStick);	 // drive with tank style
			} else{
				myRobot->ArcadeDrive(rightStick);	         // drive with arcade style (use right stick)
			}

			// Control the movement of the arm using the joystick
			// Use the "Y" value of the arm joystick to control the movement of the arm
			float armStickDirection = armStick->GetY();

			// if at a limit and telling the arm to move past the limit, don't drive the motor
			if ((armUpperLimit->Get() == 0) && (armStickDirection > 0.0)) {
				armStickDirection = 0;
			} else if ((armLowerLimit->Get() == 0) && (armStickDirection < 0.0)) {
				armStickDirection = 0;
			}

			// Set the motor value 
			armMotor.Set(armStickDirection);
			Wait(0.005);
		}
	}
};

START_ROBOT_CLASS(DefaultRobot);


