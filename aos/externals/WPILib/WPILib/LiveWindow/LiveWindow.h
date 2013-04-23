#ifndef _LIVE_WINDOW_H
#define _LIVE_WINDOW_H

#include "LiveWindow/LiveWindowSendable.h"
#include "tables/ITable.h"
#include "Commands/Scheduler.h"
#include "Synchronized.h"
#include <vector>
#include <map>

struct LiveWindowComponent
{
#if 0
	std::string subsystem;
	std::string name;
	bool isSensor;
#endif

	LiveWindowComponent()
	{}//WTF?
	LiveWindowComponent(std::string subsystem, std::string name, bool isSensor)
	{
#if 0
		this->subsystem = subsystem;
		this->name = name;
		this->isSensor = isSensor;
#else
    (void)subsystem;
    (void)name;
    (void)isSensor;
#endif
	}
};

/**
 * The LiveWindow class is the public interface for putting sensors and actuators
 * on the LiveWindow.
 *
 * @author Brad Miller
 */
class LiveWindow {
public:
	static LiveWindow * GetInstance();
	void Run();
	void AddSensor(char *subsystem, char *name, LiveWindowSendable *component);
	void AddActuator(char *subsystem, char *name, LiveWindowSendable *component);
	void AddSensor(std::string type, int module, int channel, LiveWindowSendable *component);
	void AddActuator(std::string type, int module, int channel, LiveWindowSendable *component);
	
	bool IsEnabled() { return false; }
	void SetEnabled(bool enabled);

protected:
	LiveWindow();
	virtual ~LiveWindow();

private:
	void UpdateValues();
	void Initialize();
	void InitializeLiveWindowComponents();
	
#if 0
	std::vector<LiveWindowSendable *> m_sensors;
	std::map<LiveWindowSendable *, LiveWindowComponent> m_components;
	
#endif
	static LiveWindow *m_instance;
  static ReentrantSemaphore m_instanceLock;
#if 0
	ITable *m_liveWindowTable;
	ITable *m_statusTable;
	
	Scheduler *m_scheduler;
	
	bool m_enabled;
	bool m_firstTime;
#endif
};

#endif

