/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Buttons/Button.h"

#include "Buttons/HeldButtonScheduler.h"
#include "Buttons/PressedButtonScheduler.h"
#include "Buttons/ReleasedButtonScheduler.h"
#include "NetworkTables/NetworkTable.h"

Button::Button() {
	m_table = NULL;
}

bool Button::Grab()
{
	if (Get())
		return true;
	else if (m_table != NULL)
	{
		if (m_table->IsConnected())
			return m_table->GetBoolean("pressed");
		else
			return false;
	}
	else
		return false;
}

void Button::WhenPressed(Command *command)
{
	PressedButtonScheduler *pbs = new PressedButtonScheduler(Grab(), this, command);
	pbs->Start();
}

void Button::WhileHeld(Command *command)
{
	HeldButtonScheduler *hbs = new HeldButtonScheduler(Grab(), this, command);
	hbs->Start();
}

void Button::WhenReleased(Command *command)
{
	ReleasedButtonScheduler *rbs = new ReleasedButtonScheduler(Grab(), this, command);
	rbs->Start();
}

NetworkTable* Button::GetTable()
{
	if (m_table == NULL)
	{
		m_table = new NetworkTable();
		m_table->PutBoolean("pressed", Get());
	}
	return m_table;
}
