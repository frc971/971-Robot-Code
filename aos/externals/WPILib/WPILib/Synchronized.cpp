/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Synchronized.h"

/**
 * Semaphore class deals with low-level vxworks semaphores.
 * The Semaphore class is very useful for static variables because it takes care of creating and
 * deleting the raw vxworks system semaphore at load and unload time.
 * This constructor will create the semaphore with semBCreate(SEM_Q_PRIORITY, SEM_FULL).
 */
Semaphore::Semaphore()
{
	m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
}
/**
 * Semaphore destructor.
 * This destructor deletes the semaphore ensuring that the code does not leak vxworks system resources.
 */
Semaphore::~Semaphore()
{
	semDelete(m_semaphore);
}
/**
 * Retrieve the raw vxworks semaphore.
 */
SEM_ID Semaphore::get()
{
	return m_semaphore;
}

/**
 * Take the semaphore.
 *
 * Taking the semaphore is also called locking. If the semaphore is not
 * currently taken, it will block until it is given.
 *
 * @return 0 for success and -1 for error. If -1, the error will be in errno.
 */
int Semaphore::Take()
{
	return semTake(m_semaphore, WAIT_FOREVER);
}
/**
 * Give the semaphore.
 *
 * Giving the semaphore is also called unlocking. If another task is currently
 * waiting to take the semaphore, it will succeed.
 *
 * @return 0 for success and -1 for error. If -1, the error will be in errno.
 */
int Semaphore::Give()
{
	return semGive(m_semaphore);
}

/**
 * Synchronized class deals with critical regions.
 * Declare a Synchronized object at the beginning of a block. That will take the semaphore.
 * When the code exits from the block it will call the destructor which will give the semaphore.
 * This ensures that no matter how the block is exited, the semaphore will always be released.
 * Use the CRITICAL_REGION(SEM_ID) and END_REGION macros to make the code look cleaner (see header file)
 * @param semaphore The semaphore controlling this critical region.
 */
Synchronized::Synchronized(SEM_ID semaphore)
{
	m_semaphore = semaphore;
	semTake(m_semaphore, WAIT_FOREVER);
}
Synchronized::Synchronized(Semaphore &semaphore)
{
	m_semaphore = semaphore.get();
	semTake(m_semaphore, WAIT_FOREVER);
}

/**
 * Synchronized destructor.
 * This destructor frees the semaphore ensuring that the resource is freed for the block
 * containing the Synchronized object.
 */
Synchronized::~Synchronized()
{
	semGive(m_semaphore);
}
