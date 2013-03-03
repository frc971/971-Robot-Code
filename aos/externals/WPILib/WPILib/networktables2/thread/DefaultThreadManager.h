/*
 * DefaultThreadManager.h
 *
 *  Created on: Sep 21, 2012
 *      Author: Mitchell Wills
 */

#ifndef DEFAULTTHREADMANAGER_H_
#define DEFAULTTHREADMANAGER_H_

class DefaultThreadManager;
class PeriodicNTThread;

#include "networktables2/thread/PeriodicRunnable.h"
#include "networktables2/thread/NTThreadManager.h"
#include "networktables2/thread/NTThread.h"
#include "Task.h"




class DefaultThreadManager : public NTThreadManager{
	virtual NTThread* newBlockingPeriodicThread(PeriodicRunnable* r, const char* name);
};

class PeriodicNTThread : public NTThread {
private:
	const char* name;
	Task* thread;
	PeriodicRunnable* r;
	bool run;
	int _taskMain();
	static int taskMain(PeriodicNTThread* o);
public:
	PeriodicNTThread(PeriodicRunnable* r, const char* name);
	virtual ~PeriodicNTThread();
	virtual void stop();
	virtual bool isRunning();
};


#endif /* DEFAULTTHREADMANAGER_H_ */
