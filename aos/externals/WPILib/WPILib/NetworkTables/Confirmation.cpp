/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/Confirmation.h"
#include "NetworkTables/Buffer.h"
#include "NetworkTables/InterfaceConstants.h"

namespace NetworkTables
{

Confirmation::Confirmation(int count) :
	m_count(count)
{
}

void Confirmation::Encode(Buffer *buffer)
{
	for (int i = m_count; i > 0; i -= kNetworkTables_CONFIRMATION - 1)
	{
		buffer->WriteByte(kNetworkTables_CONFIRMATION | std::min(kNetworkTables_CONFIRMATION - 1, i));
	}
}

// Currently unused
Confirmation *Confirmation::Combine(Confirmation *a, Confirmation *b)
{
	a->m_count = a->m_count + b->m_count;
	delete b;
	return a;
}

}
