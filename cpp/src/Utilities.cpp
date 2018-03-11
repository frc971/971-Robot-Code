#include "ctre/phoenix/Utilities.h"

namespace ctre {
namespace phoenix {

float Utilities::abs(float f) {
	if (f >= 0)
		return f;
	return -f;
}

float Utilities::bound(float value, float capValue) {
	if (value > capValue)
		return capValue;
	if (value < -capValue)
		return -capValue;
	return value;
}

float Utilities::cap(float value, float peak) {
	if (value < -peak)
		return -peak;
	if (value > +peak)
		return +peak;
	return value;
}

bool Utilities::Contains(char array[], char item) {
	//Not sure how to implement in c++ yet, made private
	(void)array;
	(void)item;
	return false;
}

void Utilities::Deadband(float &value, float deadband) {
	if (value < -deadband) {
		/* outside of deadband */
	} else if (value > +deadband) {
		/* outside of deadband */
	} else {
		/* within 10% so zero it */
		value = 0;
	}
}

bool Utilities::IsWithin(float value, float compareTo, float allowDelta) {
	float f = value - compareTo;
	if (f < 0)
		f *= -1;
	return (f < allowDelta);
}

int Utilities::SmallerOf(int value_1, int value_2) {
	if (value_1 > value_2)
		return value_2;
	else
		return value_1;
}

void Utilities::Split_1(float forward, float turn, float *left, float *right) {
	*left = forward + turn;
	*right = forward - turn;
}
void Utilities::Split_2(float left, float right, float *forward, float *turn) {
	*forward = (left + right) * 0.5f;
	*turn = (left - right) * 0.5f;
}
}
}
