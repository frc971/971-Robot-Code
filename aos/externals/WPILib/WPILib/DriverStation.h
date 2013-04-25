/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __DRIVER_STATION_H__
#define __DRIVER_STATION_H__

#include "Dashboard.h"
#include "DriverStationEnhancedIO.h"
#include "SensorBase.h"
#include "Task.h"
#include "Synchronized.h"
#include "RWLock.h"

struct FRCCommonControlData;
class AnalogChannel;

/**
 * Provide access to the network communication data to / from the Driver Station.
 */
class DriverStation : public SensorBase
{
public:
	enum Alliance {kRed, kBlue, kInvalid};

  // Represents all of the states that FMS thinks of a robot as being in.
  // NOTE: All of the ones except kDisabled mean that the robot is enabled too.
  enum FMSState {
    kDisabled,
    kAutonomous,
    kTeleop,
    kTestMode,
  };

	static DriverStation *GetInstance();

  RWLock::Locker GetDataReadLock();

	static const UINT32 kBatteryModuleNumber = 1;
	static const UINT32 kBatteryChannel = 8;
	static const UINT32 kJoystickPorts = 4;
	static const UINT32 kJoystickAxes = 6;

	float GetStickAxis(UINT32 stick, UINT32 axis);
	short GetStickButtons(UINT32 stick);

	float GetAnalogIn(UINT32 channel);
	bool GetDigitalIn(UINT32 channel);
	void SetDigitalOut(UINT32 channel, bool value);
	bool GetDigitalOut(UINT32 channel);

	bool IsEnabled();
	bool IsDisabled();
  bool IsAutonomous();
	bool IsOperatorControl();
  bool IsTest();
  FMSState GetCurrentState();
	bool IsNewControlData();
	bool IsFMSAttached();

	UINT32 GetPacketNumber();
	Alliance GetAlliance();
	UINT32 GetLocation();
	void WaitForData();
	double GetMatchTime();
	float GetBatteryVoltage();
	UINT16 GetTeamNumber();

	// Get the default dashboard packers. These instances stay around even after
	// a call to SetHigh|LowPriorityDashboardPackerToUse() changes which packer
	// is in use. You can restore the default high priority packer by calling
	// SetHighPriorityDashboardPackerToUse(&GetHighPriorityDashboardPacker()).
	Dashboard& GetHighPriorityDashboardPacker() { return m_dashboardHigh; }
	Dashboard& GetLowPriorityDashboardPacker() { return m_dashboardLow; }

	// Get/set the dashboard packers to use. This can sideline or restore the
	// default packers. Initializing SmartDashboard changes the high priority
	// packer in use so beware that the default packer will then be idle. These
	// methods support any kind of DashboardBase, e.g. a Dashboard or a
	// SmartDashboard.
	DashboardBase* GetHighPriorityDashboardPackerInUse() { return m_dashboardInUseHigh; }
	DashboardBase* GetLowPriorityDashboardPackerInUse() { return m_dashboardInUseLow; }
	void SetHighPriorityDashboardPackerToUse(DashboardBase* db) { m_dashboardInUseHigh = db; }
	void SetLowPriorityDashboardPackerToUse(DashboardBase* db) { m_dashboardInUseLow = db; }

	DriverStationEnhancedIO& GetEnhancedIO() { return m_enhancedIO; }

	void IncrementUpdateNumber() { m_updateNumber++; }
	SEM_ID GetUserStatusDataSem() { return m_statusDataSemaphore; }

	/** Only to be used to tell the Driver Station what code you claim to be executing
	 *   for diagnostic purposes only
	 * @param entering If true, starting disabled code; if false, leaving disabled code */
	void InDisabled(bool entering) {m_userInDisabled=entering;}
	/** Only to be used to tell the Driver Station what code you claim to be executing
	 *   for diagnostic purposes only
	 * @param entering If true, starting autonomous code; if false, leaving autonomous code */
	void InAutonomous(bool entering) {m_userInAutonomous=entering;}
    /** Only to be used to tell the Driver Station what code you claim to be executing
     *   for diagnostic purposes only
     * @param entering If true, starting teleop code; if false, leaving teleop code */
    void InOperatorControl(bool entering) {m_userInTeleop=entering;}
    /** Only to be used to tell the Driver Station what code you claim to be executing
     *   for diagnostic purposes only
     * @param entering If true, starting test code; if false, leaving test code */
    void InTest(bool entering) {m_userInTest=entering;}
  /**
   * Get a pointer to the lock used for the data set by the In* methods.
   * Creating write locks on this is useful if you want to atomically modify the
   * information about what code you claim to be executing.
   * @return A pointer to the lock. Be aware that the code that looks at this
   * state (using a read lock) runs after the code that reads new packets and
   * must finish before a new one can be read.
   * @see #InDisabled(bool)
   * @see #InAutonomous(bool)
   * @see #InOperatorControl(bool)
   * @see #InTest(bool)
   */
  RWLock *GetUserStateLock() { return &m_userStateLock; }

protected:
	DriverStation();
	virtual ~DriverStation();

	void GetData();
	void SetData();

private:
	static void InitTask(DriverStation *ds);
	static DriverStation *m_instance;
  static ReentrantSemaphore m_instanceSemaphore;
	static UINT8 m_updateNumber;
	///< TODO: Get rid of this and use the semaphore signaling
	static const float kUpdatePeriod = 0.02;

	void Run();

  // Volatile because it gets modified by GetData() in a separate task. Be
  // careful using values out of here (2-byte and 4-byte accesses are safe as
  // long as they're aligned, which all of the ones in here should be). If you
  // need consistent data, use m_dataLock.
  // Const because it should never be modifed except by getCommonControlData,
  // and that call has to const_cast away the volatile anyways.
  const volatile struct FRCCommonControlData *m_controlData;
  // A lock for *m_controlData.
  // Read (not write) RWLock::Lockers for this get given out to users so that
  // they can prevent updates to the data while they are doing stuff with it.
  RWLock m_dataLock;

	UINT8 m_digitalOut;
	AnalogChannel *m_batteryChannel;
	SEM_ID m_statusDataSemaphore;
	Task m_task;
	Dashboard m_dashboardHigh;  // the default dashboard packers
	Dashboard m_dashboardLow;
	DashboardBase* m_dashboardInUseHigh;  // the current dashboard packers in use
	DashboardBase* m_dashboardInUseLow;
  // Used to indicate when there is new control data available for
  // IsNewControlData(). A semaphore instead of just a bool to avoid race
  // conditions resulting in missed packets.
	SEM_ID m_newControlData;
	SEM_ID m_packetDataAvailableSem;
	DriverStationEnhancedIO m_enhancedIO;
  // Always empty. Gets semFlushed when there is new data available so that
  // multiple tasks waiting for it can be woken at the same time.
	SEM_ID m_waitForDataSem;
	double m_approxMatchTimeOffset;

  RWLock m_userStateLock;
	bool m_userInDisabled;
	bool m_userInAutonomous;
    bool m_userInTeleop;
    bool m_userInTest;
};

#endif

