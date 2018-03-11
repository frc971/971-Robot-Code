package com.ctre.phoenix.motorcontrol.can;

import com.ctre.phoenix.motorcontrol.IMotorController;

import edu.wpi.first.wpilibj.hal.HAL;
/**
 * VEX Victor SPX Motor Controller when used on CAN Bus.
 */
public class VictorSPX extends com.ctre.phoenix.motorcontrol.can.BaseMotorController
    implements IMotorController {
	/**
	 * Constructor
	 * 
	 * @param deviceNumber
	 *            [0,62]
	 */
	public VictorSPX(int deviceNumber) {
		super(deviceNumber | 0x01040000);
		HAL.report(65, deviceNumber + 1);
	}
}
