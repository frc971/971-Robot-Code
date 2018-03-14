#pragma once

namespace ctre {
namespace phoenix {
	
class Utilities {
public:
	static float abs(float f);
	static float bound(float value, float capValue = 1);
	static float cap(float value, float peak);
	static void Deadband(float &value, float deadband = -.10);
	static bool IsWithin(float value, float compareTo, float allowDelta);
	static int SmallerOf(int value_1, int value_2);
	static void Split_1(float forward, float turn, float *left, float *right);
	static void Split_2(float left, float right, float *forward, float *turn);
private:
	static bool Contains(char array[], char item);
};

}}
