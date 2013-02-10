/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "SmartDashboard/SmartDashboardData.h"

class Command;

class Button : public SmartDashboardData
{
public:
	Button();
	virtual ~Button() {}
	bool Grab();
	virtual bool Get() = 0;
	void WhenPressed(Command *command);
	void WhileHeld(Command *command);
	void WhenReleased(Command *command);

	virtual std::string GetType() {return "Button";}
	virtual NetworkTable *GetTable();

protected:
	NetworkTable* m_table;
};

#endif
