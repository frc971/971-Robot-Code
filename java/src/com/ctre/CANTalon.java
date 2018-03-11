package com.ctre;

/**
*  @deprecated Use TalonSRX instead. This deprecated class will be removed in 2019.
*  @see <a href="https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Migration%20Guide.md">Phoenix 2018 Migration Guide</a>
*/
@Deprecated
public class CANTalon extends com.ctre.phoenix.motorcontrol.can.TalonSRX {
	public CANTalon(int deviceNumber) {
		super(deviceNumber);
	}
}
