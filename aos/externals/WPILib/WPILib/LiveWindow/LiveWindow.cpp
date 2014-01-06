#include "LiveWindow/LiveWindow.h"
#include "networktables/NetworkTable.h"
#include <algorithm>
#include <sstream>

LiveWindow* LiveWindow::m_instance = NULL;
ReentrantSemaphore LiveWindow::m_instanceLock;

/**
 * Get an instance of the LiveWindow main class
 * This is a singleton to guarantee that there is only a single instance regardless of
 * how many times GetInstance is called.
 */
LiveWindow * LiveWindow::GetInstance()
{
  Synchronized sync(m_instanceLock);
	if (m_instance == NULL)
	{
		m_instance = new LiveWindow();
	}
	return m_instance;
}

/**
 * LiveWindow constructor.
 * Allocate the necessary tables.
 */
LiveWindow::LiveWindow()
{
#if 0
	m_enabled = false;
	m_liveWindowTable = NetworkTable::GetTable("LiveWindow");
	m_statusTable = m_liveWindowTable->GetSubTable("~STATUS~");
	m_scheduler = Scheduler::GetInstance();
#endif
}

/**
 * Change the enabled status of LiveWindow
 * If it changes to enabled, start livewindow running otherwise stop it
 */
void LiveWindow::SetEnabled(bool enabled)
{
#if 0
	if (m_enabled == enabled)
		return;
	if (enabled)
	{
		printf("Starting live window mode\n");
		if (m_firstTime)
		{
			InitializeLiveWindowComponents();
			m_firstTime = false;
		}
		m_scheduler->SetEnabled(false);
		m_scheduler->RemoveAll();
		for (std::map<LiveWindowSendable *, LiveWindowComponent>::iterator it =
				m_components.begin(); it != m_components.end(); ++it)
		{
			it->first->StartLiveWindowMode();
		}
	}
	else
	{
		printf("Ending LiveWindow mode\n");
		for (std::map<LiveWindowSendable *, LiveWindowComponent>::iterator it =
				m_components.begin(); it != m_components.end(); ++it)
		{
			it->first->StopLiveWindowMode();
		}
		m_scheduler->SetEnabled(true);
	}
	m_enabled = enabled;
	m_statusTable->PutBoolean("LW Enabled", m_enabled);
#endif
}

LiveWindow::~LiveWindow()
{
}

/**
 * Add a Sensor associated with the subsystem and with call it by the given name.
 * @param subsystem The subsystem this component is part of.
 * @param name The name of this component.
 * @param component A LiveWindowSendable component that represents a sensor.
 */
void LiveWindow::AddSensor(const char *subsystem, const char *name,
		LiveWindowSendable *component)
{
#if 0
	m_components[component].subsystem = subsystem;
	m_components[component].name = name;
	m_components[component].isSensor = true;
#endif
}

/**
 * Add an Actuator associated with the subsystem and with call it by the given name.
 * @param subsystem The subsystem this component is part of.
 * @param name The name of this component.
 * @param component A LiveWindowSendable component that represents a actuator.
 */
void LiveWindow::AddActuator(const char *subsystem, const char *name,
		LiveWindowSendable *component)
{
#if 0
	m_components[component].subsystem = subsystem;
	m_components[component].name = name;
	m_components[component].isSensor = false;
#endif
}

/**
 * INTERNAL
 */
void LiveWindow::AddSensor(std::string type, int module, int channel, LiveWindowSendable *component)
{
#if 0
	std::ostringstream oss;
	oss << type << "[" << module << "," << channel << "]";
	std::string types(oss.str());
	char* cc = new char[types.size() + 1];
	types.copy(cc, types.size());
	cc[types.size()]='\0';
	AddSensor("Ungrouped", cc, component);
	if (std::find(m_sensors.begin(), m_sensors.end(), component) == m_sensors.end())
		m_sensors.push_back(component);
#endif
}

/**
 * INTERNAL
 */
void LiveWindow::AddActuator(std::string type, int module, int channel, LiveWindowSendable *component)
{
#if 0
	std::ostringstream oss;
	oss << type << "[" << module << "," << channel << "]";
	std::string types(oss.str());
	char* cc = new char[types.size() + 1];
	types.copy(cc, types.size());
	cc[types.size()]='\0';
	AddActuator("Ungrouped", cc, component);
#endif
}

/**
 * Tell all the sensors to update (send) their values
 * Actuators are handled through callbacks on their value changing from the
 * SmartDashboard widgets.
 */
void LiveWindow::UpdateValues()
{
#if 0
	for (unsigned int i = 0; i < m_sensors.size(); i++)
	{
		m_sensors[i]->UpdateTable();
	}
#endif
}

/**
 * This method is called periodically to cause the sensors to send new values
 * to the SmartDashboard.
 */
void LiveWindow::Run()
{
#if 0
	if (m_enabled)
	{
		UpdateValues();
	}
#endif
}

/**
 * Initialize all the LiveWindow elements the first time we enter LiveWindow mode.
 * By holding off creating the NetworkTable entries, it allows them to be redefined
 * before the first time in LiveWindow mode. This allows default sensor and actuator
 * values to be created that are replaced with the custom names from users calling
 * addActuator and addSensor.
 */
void LiveWindow::InitializeLiveWindowComponents()
{
#if 0
	for (std::map<LiveWindowSendable *, LiveWindowComponent>::iterator it =
			m_components.begin(); it != m_components.end(); ++it)
	{
		LiveWindowSendable *component = it->first;
		LiveWindowComponent c = it->second;
		std::string subsystem = c.subsystem;
		std::string name = c.name;
		m_liveWindowTable->GetSubTable(subsystem)->PutString("~TYPE~",
				"LW Subsystem");
		ITable *table = m_liveWindowTable->GetSubTable(subsystem)->GetSubTable(
				name);
		table->PutString("~TYPE~", component->GetSmartDashboardType());
		table->PutString("Name", name);
		table->PutString("Subsystem", subsystem);
		component->InitTable(table);
		if (c.isSensor)
		{
			m_sensors.push_back(component);
		}
	}
#endif
}

