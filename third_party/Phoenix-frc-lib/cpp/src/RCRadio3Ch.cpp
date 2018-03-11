#ifndef CTR_EXCLUDE_WPILIB_CLASSES

#include "ctre/phoenix/RCRadio3Ch.h"
#include <vector>

namespace ctre {
namespace phoenix {

RCRadio3Ch::RCRadio3Ch(ctre::phoenix::CANifier *canifier) {
	_canifier = canifier;
}

float RCRadio3Ch::GetDutyCycleUs(Channel channel) {
	return _dutyCycleAndPeriods[(int) channel][0];
}

float RCRadio3Ch::GetDutyCyclePerc(Channel channel) {
	float retval = RCRadio3Ch::GetDutyCycleUs(channel);

	std::vector<double> xData = { 1000, 2000 };
	std::vector<double> yData = { -1, 1 };

	retval = RCRadio3Ch::Interpolate(xData, yData, retval, true);

	if (retval < -1) {
		retval = -1;
	} else if (retval > +1) {
		retval = +1;
	}

	return retval;
}

bool RCRadio3Ch::GetSwitchValue(Channel channel) {
	float retval = RCRadio3Ch::GetDutyCyclePerc(channel);

	return retval > 0.5f;
}

float RCRadio3Ch::GetPeriodUs(Channel channel) {
	return _dutyCycleAndPeriods[(int) channel][1];
}

void RCRadio3Ch::Process() {
	//Does some error code stuff, which we don't have...
	_errorCodes[0] = _canifier->GetPWMInput(
			ctre::phoenix::CANifier::PWMChannel::PWMChannel0, _dutyCycleAndPeriods[0]);
	_errorCodes[1] = _canifier->GetPWMInput(
			ctre::phoenix::CANifier::PWMChannel::PWMChannel1, _dutyCycleAndPeriods[1]);
	_errorCodes[2] = _canifier->GetPWMInput(
			ctre::phoenix::CANifier::PWMChannel::PWMChannel2, _dutyCycleAndPeriods[2]);
	_errorCodes[3] = _canifier->GetPWMInput(
			ctre::phoenix::CANifier::PWMChannel::PWMChannel3, _dutyCycleAndPeriods[3]);

	Status health = Status::Okay;
	if (health == Status::Okay) {
		if (_errorCodes[0] < 0) {
			health = Status::LossOfCAN;
		}
		if (_errorCodes[1] < 0) {
			health = Status::LossOfCAN;
		}
		if (_errorCodes[2] < 0) {
			health = Status::LossOfCAN;
		}
		if (_errorCodes[3] < 0) {
			health = Status::LossOfCAN;
		}
	}

	if (health == Status::Okay) {
		if (RCRadio3Ch::GetPeriodUs(RCRadio3Ch::Channel1) == 0) {
			health = Status::LossOfPwm;
		}
		if (RCRadio3Ch::GetPeriodUs(RCRadio3Ch::Channel2) == 0) {
			health = Status::LossOfPwm;
		}
		if (RCRadio3Ch::GetPeriodUs(RCRadio3Ch::Channel3) == 0) {
			health = Status::LossOfPwm;
		}
	}
	CurrentStatus = health;	//Will have to change this to a getter and a setter
}

double RCRadio3Ch::Interpolate(std::vector<double> &xData,
		std::vector<double> &yData, double x, bool extrapolate) {
	int size = xData.size();

	int i = 0;                    // find left end of interval for interpolation
	if (x >= xData[size - 2])                  // special case: beyond right end
			{
		i = size - 2;
	} else {
		while (x > xData[i + 1])
			i++;
	}
	double xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i + 1]; // points on either side (unless beyond ends)
	if (!extrapolate)           // if beyond ends of array and not extrapolating
	{
		if (x < xL)
			yR = yL;
		if (x > xR)
			yL = yR;
	}

	double dydx = (yR - yL) / (xR - xL);                             // gradient

	return yL + dydx * (x - xL);
}

}
}
#endif
