
#ifndef __AICalibration_h__
#define __AICalibration_h__

#include <vxWorks.h>

#ifdef __cplusplus
extern "C"
{
#endif

	UINT32 FRC_NetworkCommunication_nAICalibration_getLSBWeight(const UINT32 aiSystemIndex, const UINT32 channel, INT32 *status);
	INT32 FRC_NetworkCommunication_nAICalibration_getOffset(const UINT32 aiSystemIndex, const UINT32 channel, INT32 *status);

#ifdef __cplusplus
}
#endif

#endif // __AICalibration_h__
