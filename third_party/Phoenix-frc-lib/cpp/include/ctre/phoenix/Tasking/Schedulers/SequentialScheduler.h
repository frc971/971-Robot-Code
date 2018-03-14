#pragma once

#include <vector>
#include "ctre/phoenix/Tasking/ILoopable.h"
#include "ctre/phoenix/Tasking/IProcessable.h"

namespace ctre { namespace phoenix { namespace tasking { namespace schedulers {

class SequentialScheduler: public ILoopable, public IProcessable{
public:
	bool _running = false;
	std::vector<ILoopable*> _loops;
	unsigned int _idx = 0;
	bool _iterated = false;

	SequentialScheduler();
	virtual ~SequentialScheduler();

	void Add(ILoopable *aLoop);
	ILoopable * GetCurrent();
	void RemoveAll();
	void Start();
	void Stop();

	//IProcessable
	void Process();

	//ILoopable
	void OnStart();
	void OnLoop();
	void OnStop();
	bool IsDone();
};
}}}}
