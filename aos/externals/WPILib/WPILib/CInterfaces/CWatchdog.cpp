#include "CInterfaces/CWatchdog.h"
#include "Watchdog.h"

WatchdogObject CreateWatchdog()
{
	return (WatchdogObject) new Watchdog();
}

void DeleteWatchdog(WatchdogObject o)
{
	delete (Watchdog *) o;
}

bool FeedWatchdog(WatchdogObject o)
{
	return ((Watchdog *)o)->Feed();
}

void KillWatchdog(WatchdogObject o)
{
	((Watchdog *)o)->Kill();
}

double GetWatchdogTimer(WatchdogObject o)
{
	return ((Watchdog *)o)->GetTimer();
}

double GetWatchdogExpiration(WatchdogObject o)
{
	return ((Watchdog *)o)->GetExpiration();
}

void SetWatchdogExpiration(WatchdogObject o, double expiration)
{
	((Watchdog *)o)->SetExpiration(expiration);
}

bool GetWatchdogEnabled(WatchdogObject o)
{
	return ((Watchdog *)o)->GetEnabled();
}

void SetWatchdogEnabled(WatchdogObject o, bool enabled)
{
	((Watchdog *)o)->SetEnabled(enabled);
}

bool IsWatchdogAlive(WatchdogObject o)
{
	return ((Watchdog *)o)->IsAlive();
}

bool IsWatchdogSystemActive(WatchdogObject o)
{
	return ((Watchdog *)o)->IsSystemActive();
}

void LoadWatchdog()
{
}
