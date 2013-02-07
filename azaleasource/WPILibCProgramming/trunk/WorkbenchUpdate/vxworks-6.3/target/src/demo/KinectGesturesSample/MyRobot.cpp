#include "WPILib.h"
#include "Math.h"

#define PI 3.14159

//Constants which define the valid arm positions
#define ARM_MAX_ANGLE 105
#define ARM_MIN_ANGLE -90
#define Z_PLANE_TOLERANCE 0.3	/* In meters */

//Constants which define the "trigger" angles for the various buttons
#define LEG_FORWARD -110
#define LEG_BACKWARD -80
#define LEG_OUT -75
#define HEAD_LEFT 98
#define HEAD_RIGHT 82

/**
 * This is a program that demonstrates gesture processing on the robot using C++
 * The autonomous section of this code implements the same gestures as the FRC Kinect Server
 * These gestures are described in the Getting Started with the Microsoft Kinect for FRC document
 * The arm gestures are mapped to tank drive with the axis scaled to 1/3 power
 * The buttons are not mapped to any action, they are stored in individual boolean variables
 * named based on the gesture.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	Kinect *kinect;
	//SmartDashboard *dash;   //Optional SmartDashboard logging

public:
	RobotDemo(void):
		myRobot(1, 2),	// these must be initialized in the same order
		stick(1)		// as they are declared above.
	{
		myRobot.SetExpiration(0.1);
		kinect = Kinect::GetInstance();
		//dash = SmartDashboard::GetInstance();		//Optional SmartDashboard logging
	}
	
	/**
	 * Function to process a pair of joints into an angle in the XY plane. The joints are projected into
	 * the XY plane of the Kinect, then an angle is calculated using one joint as the origin and the
	 * X-axis as the reference. The X-axis can optionally be mirrored in order to avoid the discontinuity
	 * of the atan2 function.
	 * @param origin The joint to use as the origin for the calculation
	 * @param measured The second point to use as for the angle calculation
	 * @param mirrored Whether to mirror the X-axis or not
	 * @return Angle in degrees referenced from the X-axis
	 */
	double AngleXY(Skeleton::Joint origin, Skeleton::Joint measured, UINT8 mirrored){
        return (atan2((measured.y- origin.y), (mirrored) ? (origin.x - measured.x) : (measured.x - origin.x))*180/PI);
    }

	/**
	 * Funtion to process a pair of joints into an angle in the YZ plane. The joint are projected into
	 * the YZ plane of the Kinect, than an angle is calculated using one joint as the origin and the
	 * Z-axis as the reference.The Z-axis can optionally be mirrored in order to avoid the discontinuity
	 * of the atan2 function.
	 * @param origin The joint to use as the origin for the angle calculation
	 * @param measured The second point to use as for the angle calculation
	 * @param mirrored Whether to mirror the Z-axis or not
	 * @return Angle in degrees referenced from the Z-axis
	 */
    double AngleYZ(Skeleton::Joint origin, Skeleton::Joint measured, UINT8 mirrored){
        return (atan2((measured.y- origin.y), (mirrored) ? (origin.z - measured.z) : (measured.z - origin.z))*180/PI);

    }

    /**
     * Function to determine whether or not two joints are in approximately the same XY plane
     * IE. If they have approximately the same z coordinates
     * @param origin The first joint to be used in the comparison
     * @param measured The second joint to be used in the comparison
     * @return Whether or not the joints are in approximately the same XY plane
     */
    bool InSameZPlane(Skeleton::Joint origin, Skeleton::Joint measured, double tolerance)
        {
            return fabs(measured.z - origin.z) < tolerance;
        }

    /**
     * Converts an input value in the given input range into an output value along the given
     * output range.
     * If the result would be outside of the given output range, it is constrained to the 
     * output range.
     * @param input An input value within the given input range.
     * @param inputMin The minimum expected input value.
     * @param inputMax The maximum expected input value.
     * @param outputMin The minimum expected output value.
     * @param outputMax The maximum expected output value.
     * @return An output value within the given output range proportional to the input.
     */
    double CoerceToRange(double input, double inputMin, double inputMax, double outputMin, double outputMax)
        {
            // Determine the center of the input range
            double inputCenter = fabs(inputMax - inputMin) / 2 + inputMin;
            double outputCenter = fabs(outputMax - outputMin) / 2 + outputMin;

            // Scale the input range to the output range
            double scale = (outputMax - outputMin) / (inputMax - inputMin);

            // Apply the transformation
            double result = (input + -inputCenter) * scale + outputCenter;

            // Constrain to the result range
            return max(min(result, outputMax), outputMin);
        }

	/**
	 * Tank drive using the Kinect.
	 */
	void Autonomous(void)
	{
		double leftAxis, rightAxis;
		double leftAngle, rightAngle, headAngle, rightLegAngle, leftLegAngle, rightLegYZ, leftLegYZ=0;
		bool dataWithinExpectedRange;
		bool buttons[8];

		
		while (IsAutonomous())
		{
			/* Only process data if skeleton is tracked */
			if (kinect->GetTrackingState() == Kinect::kTracked) {
			
							/* Determine angle of each arm and map to range -1,1 */
			                leftAngle = AngleXY(kinect->GetSkeleton().GetShoulderLeft(), kinect->GetSkeleton().GetWristLeft(), true);
			                rightAngle = AngleXY(kinect->GetSkeleton().GetShoulderRight(), kinect->GetSkeleton().GetWristRight(), false);
			                leftAxis = CoerceToRange(leftAngle, -70, 70, -1, 1);
			                rightAxis = CoerceToRange(rightAngle, -70, 70, -1, 1);

							/* Check if arms are within valid range and at approximately the same z-value */
			                dataWithinExpectedRange = (leftAngle < ARM_MAX_ANGLE) && (leftAngle > ARM_MIN_ANGLE)
			                                     && (rightAngle < ARM_MAX_ANGLE) && (rightAngle > ARM_MIN_ANGLE);
			                dataWithinExpectedRange = dataWithinExpectedRange &&
			                                      InSameZPlane(kinect->GetSkeleton().GetShoulderLeft(),
			                                                   kinect->GetSkeleton().GetWristLeft(),
			                                                   Z_PLANE_TOLERANCE) &&
			                                      InSameZPlane(kinect->GetSkeleton().GetShoulderRight(),
			                                                   kinect->GetSkeleton().GetWristRight(),
			                                                   Z_PLANE_TOLERANCE);
							
							/* Determine the head angle and use it to set the Head buttons */
			                headAngle = AngleXY(kinect->GetSkeleton().GetShoulderCenter(), kinect->GetSkeleton().GetHead(), false);
			                buttons[0] = headAngle > HEAD_LEFT;
			                buttons[1] = headAngle < HEAD_RIGHT;

							/* Calculate the leg angles in the XY plane and use them to set the Leg Out buttons */
			                leftLegAngle = AngleXY(kinect->GetSkeleton().GetHipLeft(), kinect->GetSkeleton().GetAnkleLeft(), true);
			                rightLegAngle = AngleXY(kinect->GetSkeleton().GetHipRight(), kinect->GetSkeleton().GetAnkleRight(), false);
			                buttons[2] = leftLegAngle > LEG_OUT;
			                buttons[3] = rightLegAngle > LEG_OUT;

							/* Calculate the leg angle in the YZ plane and use them to set the Leg Forward and Leg Back buttons */
			                leftLegYZ = AngleYZ(kinect->GetSkeleton().GetHipLeft(), kinect->GetSkeleton().GetAnkleLeft(), false);
			                rightLegYZ = AngleYZ(kinect->GetSkeleton().GetHipRight(), kinect->GetSkeleton().GetAnkleRight(), false);
			                buttons[4] = rightLegYZ < LEG_FORWARD;
			                buttons[5] = rightLegYZ > LEG_BACKWARD;
			                buttons[6] = rightLegYZ < LEG_FORWARD;
			                buttons[7] = rightLegYZ > LEG_BACKWARD;

			                if (dataWithinExpectedRange){
								/**
								 * Drives using the Kinect axes scaled to 1/3 power
								 * Axes are inverted so arms up == joystick pushed away from you
								 */
			                    myRobot.TankDrive(-leftAxis*.33, -rightAxis*.33);
								
								/**
								 * Do something with boolean "buttons" here
								 */

								/* Optional SmartDashboard display of Kinect values */
								//dash->PutDouble("Left Arm", -leftAxis);
								//dash->PutDouble("Right Arm", -rightAxis);
								//dash->PutBoolean("Head Left", buttons[0]);
								//dash->PutBoolean("Head Right", buttons[1]);
								//...etc...
			                }
			                else{
			                    
								/* Arms are outside valid range */

								myRobot.TankDrive(0.0, 0.0);

								/**
								 * Do default behavior with boolean "buttons" here
								 */

								/* Optional SmartDashboard display of Kinect values */
								//dash->PutDouble("Left Arm", 0);
								//dash->PutDouble("Right Arm", 0);
								//dash->PutBoolean("Head Left", false);
								//dash->PutBoolean("Head Right", false);
								//...etc...
			                }
			            }
			            else{
			                
							/* Skeleton not tracked */

							myRobot.TankDrive(0.0, 0.0);

							/**
							* Do default behavior with boolean "buttons" here
							*/

							/* Optional SmartDashboard display of Kinect values */
							//dash->PutDouble("Left Arm", 0);
							//dash->PutDouble("Right Arm", 0);
							//dash->PutBoolean("Head Left", false);
							//dash->PutBoolean("Head Right", false);
							//...etc...
			            }
			Wait(0.01);
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

