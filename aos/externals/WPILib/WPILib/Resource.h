/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef RESOURCE_H_
#define RESOURCE_H_

#include "ErrorBase.h"
#include "Synchronized.h"
#include <vxWorks.h>

/**
 * The Resource class is a convenient way to track allocated resources.
 * It tracks them as indicies in the range [0 .. elements - 1].
 * E.g. the library uses this to track hardware channel allocation.
 *
 * The Resource class does not allocate the hardware channels or other
 * resources; it just tracks which indices were marked in use by
 * Allocate and not yet freed by Free.
 */
class Resource : public ErrorBase
{
public:
	virtual ~Resource();
	static void CreateResourceObject(Resource **r, UINT32 elements);
	UINT32 Allocate(const char *resourceDesc);
	UINT32 Allocate(UINT32 index, const char *resourceDesc);
	void Free(UINT32 index);

private:
	explicit Resource(UINT32 size);

	bool *m_isAllocated;
	ReentrantSemaphore m_allocateLock;
	UINT32 m_size;

	static ReentrantSemaphore m_createLock;

	DISALLOW_COPY_AND_ASSIGN(Resource);
};

#endif
