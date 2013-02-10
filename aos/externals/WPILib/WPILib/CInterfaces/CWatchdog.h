/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_WATCHDOG_H
#define C_WATCHDOG_H

typedef void *WatchdogObject;

WatchdogObject CreateWatchdog();
void DeleteWatchdog(WatchdogObject o);
bool FeedWatchdog(WatchdogObject o);
void KillWatchdog(WatchdogObject o);
double GetWatchdogTimer(WatchdogObject o);
double GetWatchdogExpiration(WatchdogObject o);
void SetWatchdogExpiration(WatchdogObject o, double expiration);
bool GetWatchdogEnabled(WatchdogObject o);
void SetWatchdogEnabled(WatchdogObject o, bool enabled);
bool IsWatchdogAlive(WatchdogObject o);
bool IsWatchdogSystemActive(WatchdogObject o);

void LoadWatchdog();

#endif

