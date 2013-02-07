#include "WPILib.h"

/**
 * Sample line tracking class for FIRST 2011 Competition
 * Jumpers on driver station digital I/O pins select the operating mode:
 * The Driver Station digital input 1 select whether the code tracks the straight
 * line or the forked line. Driver station digital input 2 selects whether the
 * code takes the left or right fork. You can set these inputs using jumpers on
 * the USB I/O module or in the driver station I/O Configuration pane (if there
 * is no Digital I/O module installed.
 *
 * Since there is the fork to contend with, the code tracks the edge of the line
 * using a technique similar to that used with a single-sensor Lego robot.
 *
 * This code worked on a simple bot built from the 2011 kit chassis and weighted
 * to behave similarly to complete robot complete with scoring mechanisms. Your
 * own robot will likely be geared differently, the CG will be different, the
 * wheels may be different - so expect to tune the program for your own robot
 * configuration. The two places to do tuning are:
 *
 * defaultSteeringGain - this is the amount of turning correction applied
 * forkProfile & straightProfile - these are power profiles applied at various
 *	times (one power setting / second of travel) as the robot moves towards
 *	the wall.
 *
 * In addition: this program uses dead reckoning - that is it drives at various
 * power settings so it will slow down and stop at the end of the line. This is
 * highly dependent on robot weight, wheel choice, and battery voltage. It will
 * behave differently on a fully charged vs. partially charged battery.
 *
 * To overcome these limitations, you should investigate the use of feedback to
 * have power/wheel independent control of the program. Examples of feedback
 * include using wheel encoders to measure the distance traveled or some kind of
 * rangefinder to determine the distance to the wall. With these sensors installed
 * your program can know precisely how far from the wall it is, and set the speeds
 * accordingly.
 *
 */
class RobotDemo : public SimpleRobot
{
	RobotDrive *drive;			// robot drive base object
	DigitalInput *left;			// digital inputs for line tracking sensors
	DigitalInput *middle;
	DigitalInput *right;
	DriverStation *ds;			// driver station object for getting selections

public:
	/*
	 * RobotDemo constructor
	 * This code creates instances of the objects and sets up the driving directions
	 * for the RobotDrive object. Our robot was geared such that we had to invert
	 * each of them motor outputs. You may not need to do that
	 */
	RobotDemo() {
		drive = new RobotDrive(1, 2);
		drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		drive->SetExpiration(15);
		left = new DigitalInput(1);
		middle = new DigitalInput(2);
		right = new DigitalInput(3);
		ds = DriverStation::GetInstance();	// driver station instance for digital I/O
	}
	/**
	 * This function is called once each time the robot enters autonomous mode.
	 */
	void Autonomous() {
		double defaultSteeringGain = 0.65;	// default value for steering gain

		int binaryValue;
		int previousValue = 0;
		double steeringGain;

		// the power profiles for the straight and forked robot path. They are
		// different to let the robot drive more slowly as the robot approaches
		// the fork on the forked line case.
		double forkProfile[] = {0.70, 0.70, 0.55, 0.60, 0.60, 0.50, 0.40, 0.0};
		double straightProfile[] = {0.70, 0.70, 0.60, 0.60, 0.35, 0.35, 0.35, 0.0};

		double *powerProfile;  // the selected power profile
		
		// set the straight vs forked path variables as read from the DS digital
		// inputs or the I/O Setup panel on the driver station.
		bool straightLine = ds->GetDigitalIn(1);
		powerProfile = (straightLine) ? straightProfile : forkProfile;
		double stopTime = (straightLine) ? 2.0 : 4.0;
		bool goLeft = !ds->GetDigitalIn(2) && !straightLine;
		printf("StraightLine: %d\n", straightLine);
		printf("GoingLeft: %d\n", goLeft);

		bool atCross = false;	// true when robot has reached end

		// set up timer for 8 second max driving time and use the timer to
		// pick values from the power profile arrays
		Timer *timer = new Timer();
		timer->Start();
		timer->Reset();
		
		int oldTimeInSeconds = -1;
		double time;
		double speed, turn;

		// loop until either we hit the "T" at the end or 8 seconds has
		// elapsed. The time to the end should be less than 7 seconds
		// for either path.
		while ((time = timer->Get()) < 8.0 && !atCross) {
			int timeInSeconds = (int) time;
			int leftValue = left->Get() ? 1 : 0;	// read the line tracking sensors
			int middleValue = middle->Get() ? 1 : 0;
			int rightValue = right->Get() ? 1 : 0;

		    // compute the single value from the 3 sensors. Notice that the bits
		    // for the outside sensors are flipped depending on left or right
		    // fork. Also the sign of the steering direction is different for left/right.
			if (goLeft) {
				binaryValue = leftValue * 4 + middleValue * 2 + rightValue;
				steeringGain = -defaultSteeringGain;
			} else {
				binaryValue = rightValue * 4 + middleValue * 2 + leftValue;
				steeringGain = defaultSteeringGain;
			}

			speed = powerProfile[timeInSeconds];	// speed value for this time
			turn = 0;								// default to no turn

			switch (binaryValue) {
				case 1:					// just the outside sensor - drive straight
					turn = 0;
					break;
				case 7:					// all sensors - maybe at the "T"
				if (time> stopTime) {
					atCross = true;
					speed = 0;
				}
				break;
				case 0:					// no sensors - apply previous correction
				if (previousValue == 0 || previousValue == 1) {
					turn = steeringGain;
				}
				else {
					turn = -steeringGain;
				}
				break;
				default:				// anything else, steer back to the line
				turn = -steeringGain;
			}
			// useful debugging output for tuning your power profile and steering gain
			if(binaryValue != previousValue)
				printf("Time: %2.2f sensor: %d speed: %1.2f turn: %1.2f atCross: %d\n", time, binaryValue, speed, turn, atCross);
			// move the robot forward
			drive->ArcadeDrive(speed, turn);
			if (binaryValue != 0) previousValue = binaryValue;
			oldTimeInSeconds = timeInSeconds;
			Wait(0.01);
		}
		// stop driving when finished
		drive->ArcadeDrive(0.0, 0.0);
	}

	/**
	 * This function is called once each time the robot enters operator control.
	 */
	void OperatorControl() {
		// supply your own teleop code here
	}
};

START_ROBOT_CLASS(RobotDemo)
;

