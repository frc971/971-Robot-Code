#pragma once
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/paramEnum.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <map>
#include <sys/stat.h>
#include <vector>
#include <dirent.h>
#include <mutex>
#include <thread>

/* forward prototype */
namespace ctre {
namespace phoenix {
namespace logger {
class TimestampMsgMap;
}
}
}

class LoggerDriver {
public:
	void Looping();
	void Open(int language, bool logDriverStation);
	void Close();

	static const int kHierarchyInternal = 0;
	static const int kHierarchyCCI = 1;
	static const int kHierarchyJava = 2;
	static const int kHierarchyAPI = 3;


	ctre::phoenix::ErrorCode Log(ctre::phoenix::ErrorCode code, const std::string & origin, int hierarchy, const char *stacktrace);
	void GetDescription(ctre::phoenix::ErrorCode code, std::string & shrtError, std::string & longError);

	static LoggerDriver & GetInstance();
private:
	LoggerDriver();
	~LoggerDriver();
	static unsigned long GetDirSize(DIR *t);
	static std::vector<std::string> OrderedFiles(DIR *directory);
	
	ctre::phoenix::logger::TimestampMsgMap * _msgMap;

//	unsigned long startOfSeconds = 0;
//	//std::map<std::string, unsigned long> timeStamps;
//	bool writing = false;
//	bool errorF = false;
//	bool threadStarted = false;
//	bool closeThread = false;
//	std::string buf = "";
//	std::mutex thisMutex;
//	std::thread t;
//	FILE *file = NULL;

	static LoggerDriver *_instance;
	
};
