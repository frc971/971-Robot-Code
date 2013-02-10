/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __DENIAL_H__
#define __DENIAL_H__

#include "NetworkTables/Data.h"

namespace NetworkTables
{

class Buffer;

class Denial : public Data
{
public:
	Denial(int count);
	virtual void Encode(Buffer *buffer);
private:
	static Denial *Combine(Denial *a, Denial *b);

	int m_count;
};

} // namespace

#endif
