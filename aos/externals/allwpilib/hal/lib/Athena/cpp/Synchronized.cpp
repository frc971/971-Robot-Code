/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "HAL/cpp/Synchronized.hpp"
#include "HAL/Semaphore.hpp"

/**
 * Synchronized class deals with critical regions.
 * Declare a Synchronized object at the beginning of a block. That will take the semaphore.
 * When the code exits from the block it will call the destructor which will give the semaphore.
 * This ensures that no matter how the block is exited, the semaphore will always be released.
 * @param semaphore The semaphore controlling this critical region.
 */
Synchronized::Synchronized(SEMAPHORE_ID semaphore)
    : m_semaphore(semaphore) {
  takeSemaphore(m_semaphore);
}

/**
 * This destructor unlocks the semaphore.
 */
Synchronized::~Synchronized()
{
	giveSemaphore(m_semaphore);
}
