package com.ctre.phoenix;

public class Util
{
	public static double cap(double value, double peak)
	{
		if(value < -peak) value = -peak;
		if(value > peak) value = peak;
		return value;
	}
	
	public static int scaleRotationsToNativeUnits(double scalar, double fullRotations) {
		/* first assume we don't have config info, prep the default return */
		int retval = (int) fullRotations;
		/* apply scalar if its available */
		if (scalar > 0) {
		  retval = (int) (fullRotations * scalar);
		}
		return retval;
	}
	public static int scaleVelocityToNativeUnits(double scalar, double rpm) {
		/* first assume we don't have config info, prep the default return */
		int retval = (int) rpm;
		/* apply scalar if its available */
		if (scalar > 0) {
		  retval = (int) (rpm * scalar);
		}
		return retval;
	}
	public static double scaleNativeUnitsToRotations(double scalar, long nativePos) {
		/* first assume we don't have config info, prep the default return */
		double retval = (double) nativePos;
		/* retrieve scaling info */
		if (scalar > 0) {
		  retval = ((double) nativePos) / scalar;
		}
		return retval;
	}
	public static double scaleNativeUnitsToRpm(double scalar, long nativeVel) {
		/* first assume we don't have config info, prep the default return */
		double retval = (double) nativeVel;
		/* apply scalar if its available */
		if (scalar > 0) {
		  retval = (double) (nativeVel) / (scalar);
		}
		return retval;
	}
}