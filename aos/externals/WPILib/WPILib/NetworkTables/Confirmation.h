/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __CONFIRMATION_H__
#define __CONFIRMATION_H__

#include "NetworkTables/Data.h"

namespace NetworkTables
{

class Buffer;

class Confirmation : public Data
{
public:
	Confirmation(int count);
	virtual void Encode(Buffer *buffer);
private:
	static Confirmation *Combine(Confirmation *a, Confirmation *b);

	int m_count;
};

} // namespace

#endif
