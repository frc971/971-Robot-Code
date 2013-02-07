#ifndef __DashboardDataFormat_h__
#define __DashboardDataFormat_h__

#include "WPILib.h"

/**
 * This class is just an example of one way you could organize the data that you want
 * to send to the dashboard.  The PackAndSend method does all the work.  You could
 * put the packing code directly in your code, but this model protects you from
 * packing data in the wrong order throughout your code.
 * 
 * The data and the format of this structure are just an example.  It is written to
 * match the initial data format expected by the LabVIEW Dashboard project.  Feel free
 * to add data elements or remove them.  Just remember to make any changes consistently
 * between the LabVIEW "Dashboard Datatype" and the data that gets packed by this class.
 */
class DashboardDataFormat : public SensorBase
{
public:
	DashboardDataFormat(void);
	virtual ~DashboardDataFormat();
	void SendIOPortData(void);
	void SendVisionData(void);
private:
	DISALLOW_COPY_AND_ASSIGN(DashboardDataFormat);
	AnalogModule *analogModule1;
	AnalogModule *analogModule2;
	DigitalModule *digitalModule1;
	DigitalModule *digitalModule2;
};

#endif // __DashboardDataFormat_h__
