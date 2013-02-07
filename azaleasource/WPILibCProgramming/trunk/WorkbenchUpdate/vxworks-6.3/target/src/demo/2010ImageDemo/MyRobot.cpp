#include "WPILib.h"

#include "Target.h"
#include "DashboardDataSender.h"

#define MINIMUM_SCORE 0.01

double outputValue;

class SamplePIDOutput : public PIDOutput {
public:
	SamplePIDOutput(RobotDrive *base) {
		m_base = base;
	}

	void PIDWrite(float output) {
		m_base->ArcadeDrive(0.0, output);
		outputValue = -output;
	}
private:
	RobotDrive *m_base;
};

/**
 * Demo that demonstrates tracking with the robot.
 * In Teleop mode the robot will drive using a single joystick plugged into port 1.
 * When the trigger is pressed, the robot will turn to face the circular target.
 * This demo uses the AxisCamera class to grab images. It finds the target and computes
 * the angle to the target from the center of view in the camera image. It then computes
 * a goal correction angle and uses the gyro to turn to that heading. The gyro correction
 * is done using the PID class to drive the robot to the desired heading.
 * The coefficients for the PID loop will need to be tuned to match the CG and weight of
 * your robot as well as the wheel and driving surface types.
 * 
 * As an added bonus, the program also sends robot status information to the LabVIEW sample
 * dashboard as well as camera annotation to show the targets that the robot is currently
 * seeing.
 */
class RobotDemo : public SimpleRobot {
	RobotDrive *base;
	Joystick *stick; // only joystick
	Gyro *gyro;
	PIDOutput *pidOutput;
	DashboardDataSender *dds;

public:
	RobotDemo(void) {
		// initialize all variables
		base = new RobotDrive(1, 2);
		base->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		base->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		stick = new Joystick(1);
		gyro = new Gyro(1);
		pidOutput = new SamplePIDOutput(base);
		dds = new DashboardDataSender();
		base->SetExpiration(0.5);
	}

	/**
	 * Main test program in operator control period.
	 * Aquires images from the camera and determines if they match FRC targets.
	 */
	void OperatorControl(void) {
		/**
		 * Set up the PID controller with some parameters that should be pretty
		 * close for most kitbot robots.
		 */
		printf("Initializing PIDController\n");
		PIDController turnController( 0.1, // P
				0.00, // I
				0.5, // D
				gyro, // source
				pidOutput, // output
				0.005); // period
		turnController.SetInputRange(-360.0, 360.0);
		turnController.SetOutputRange(-0.6, 0.6);
		turnController.SetTolerance(1.0 / 90.0 * 100);
		turnController.Disable();

		// Create and set up a camera instance. first wait for the camera to start
		// if the robot was just powered on. This gives the camera time to boot.
		printf("Getting camera instance\n");
		// Pass in the IP address of your camera here.
		// If you connect your camera to the switch on your robot, the
		// camera's IP address is typically 10.te.am.11
		AxisCamera &camera = AxisCamera::GetInstance("192.168.0.90");
		printf("Setting camera parameters\n");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(0);

		// set sensitivity for the 2010 kit gyro
		gyro->SetSensitivity(0.007);

		// set MotorSafety expiration
		base->SetExpiration(1.0);

		// keep track of the previous joystick trigger value
		bool lastTrigger = false;

		// loop getting images from the camera and finding targets
		printf("Starting operator control loop\n");
		Timer timer;
		timer.Start();
		while (IsOperatorControl())
		{
			bool trigger;
			// if trigger is pulled, the robot will run with standard arcade drive
			// otherwise the robot will home towards the target.
			if (trigger = stick->GetTrigger()) {
				if (trigger != lastTrigger) 			// check if trigger changed
					turnController.Enable();
				// if there's a fresh and we're at the previous target heading then
				// get a camera image and process it
				if (camera.IsFreshImage())
				{
					timer.Reset();
					// get the gyro heading that goes with this image
					double gyroAngle = gyro->PIDGet();

					// get the camera image
					HSLImage *image = camera.GetImage();

					// find FRC targets in the image
					vector<Target> targets = Target::FindCircularTargets(image);
					delete image;
					if (targets.size() == 0 || targets[0].m_score < MINIMUM_SCORE)
					{
						// no targets found. Make sure the first one in the list is 0,0
						// since the dashboard program annotates the first target in green
						// and the others in magenta. With no qualified targets, they'll all
						// be magenta.
						Target nullTarget;
						nullTarget.m_majorRadius = 0.0;
						nullTarget.m_minorRadius = 0.0;
						nullTarget.m_score = 0.0;
						if (targets.size() == 0)
							targets.push_back(nullTarget);
						else
							targets.insert(targets.begin(), nullTarget);
						dds->sendVisionData(0.0, gyro->GetAngle(), 0.0, 0.0, targets);
						if (targets.size() == 0)
							printf("No target found\n\n");
						else
							printf("No valid targets found, best score: %f ", targets[0].m_score);
					}
					else {
						// We have some targets.
						// set the new PID heading setpoint to the first target in the list
						double horizontalAngle = targets[0].GetHorizontalAngle();
						double setPoint = gyroAngle + horizontalAngle;

						turnController.SetSetpoint(setPoint);
						
						// send dashbaord data for target tracking
						dds->sendVisionData(0.0, gyro->GetAngle(), 0.0, targets[0].m_xPos / targets[0].m_xMax, targets);
						printf("Target found %f ", targets[0].m_score);
//						targets[0].Print();
					}
					printf("Time: %f\n", 1.0 / timer.Get());
				}
			} else {
				// if the trigger is not pressed, then do Arcade driving with joystick 1
				if (trigger != lastTrigger)
					turnController.Disable();
				base->ArcadeDrive(stick);
			}
			lastTrigger = trigger;
			
			// send the dashbaord data associated with the I/O ports
			dds->sendIOPortData();
		}
	}
};

START_ROBOT_CLASS(RobotDemo)
;

