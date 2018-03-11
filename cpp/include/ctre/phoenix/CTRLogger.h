#include "ctre/phoenix/ErrorCode.h" // ErrorCode
#include <string>

namespace ctre {
namespace phoenix {

class CTRLogger {
public:
	static void Close();
	static ErrorCode Log(ErrorCode code, std::string origin);
	static void Open(int language);
	//static void Description(ErrorCode code, const char *&shrt, const char *&lng);
};

}
}
