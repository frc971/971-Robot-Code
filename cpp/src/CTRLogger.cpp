#include "ctre/phoenix/CTRLogger.h"
#include "ctre/phoenix/CCI/Logger_CCI.h" // c_Logger_*
#include <execinfo.h>

namespace ctre {
namespace phoenix {

void CTRLogger::Open(int language) {
	c_Logger_Open(language, true);
}
ErrorCode CTRLogger::Log(ErrorCode code, std::string origin) {
	void *buf[100];
	char **strings;
	int size = backtrace(buf, 100);
	strings = backtrace_symbols(buf, size);
	std::string stackTrace;
	for (int i = 1; i < size; i++) {
		stackTrace += strings[i];
		stackTrace += "\n";
	}
	return c_Logger_Log(code, origin.c_str(), 3, stackTrace.c_str());
}
void CTRLogger::Close() {
	c_Logger_Close();
}
//void CTRLogger::Description(ErrorCode code, const char *&shrt, const char *&lng) {
//	c_Logger_Description(code, shrt, lng);
//}

}
}
