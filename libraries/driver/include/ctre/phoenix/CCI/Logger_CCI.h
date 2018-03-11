#pragma once

#include "ctre/phoenix/ErrorCode.h"
#include <string>

extern "C" {
	void c_Logger_Close();
	void c_Logger_Open(int language, bool logDriverStation);
	ctre::phoenix::ErrorCode c_Logger_Log(ctre::phoenix::ErrorCode code, const char* origin, int hierarchy, const char *stacktrace);
	void c_Logger_Description(ctre::phoenix::ErrorCode code, std::string & shortDescripToFill, std::string & longDescripToFill);
}
