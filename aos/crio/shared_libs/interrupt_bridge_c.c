#include <vxWorksCommon.h> // or else it blows up...
// This header ends up trying to #include a header which declares a struct with
// a member named "delete". This means that it can not be used from code
// compiled with the C++ compiler.
#include <usrConfig.h>

int aos_getExcExtendedVectors(void) {
	return excExtendedVectors;
}
