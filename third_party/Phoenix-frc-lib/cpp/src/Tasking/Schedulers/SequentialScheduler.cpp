#include "ctre/phoenix/Tasking/Schedulers/SequentialScheduler.h"

namespace ctre {
namespace phoenix {
namespace tasking {
namespace schedulers {

SequentialScheduler::SequentialScheduler() {
}
SequentialScheduler::~SequentialScheduler() {
}
void SequentialScheduler::Add(ILoopable *aLoop) {
	_loops.push_back(aLoop);
}
ILoopable * SequentialScheduler::GetCurrent() {
	ILoopable* retval = nullptr;
	if (_idx < _loops.size()) {
		retval = _loops[_idx];
	}
	return retval;
}

void SequentialScheduler::RemoveAll() {
	_loops.clear();
}
void SequentialScheduler::Start() {
	/* reset iterator regardless of loopable container */
	_idx = 0;
	/* do we have any to loop? */
	if (_idx >= _loops.size()) {
		/* there are no loopables */
		_running = false;
	} else {
		/* start the first one */
		_loops[_idx]->OnStart();
		_running = true;
	}

}
void SequentialScheduler::Stop() {
	for (unsigned int i = 0; i < _loops.size(); i++) {
		_loops[i]->OnStop();
	}
	_running = false;
}
void SequentialScheduler::Process() {
	if (_idx < _loops.size()) {
		if (_running) {
			ILoopable* loop = _loops[_idx];
			loop->OnLoop();
			if (loop->IsDone()) {
				/* iterate to next loopable */
				++_idx;
				if (_idx < _loops.size()) {
					/* callback to start it */
					_loops[_idx]->OnStart();
				}
			}
		}
	} else {
		_running = false;
	}
}
/* ILoopable */
void SequentialScheduler::OnStart() {
	SequentialScheduler::Start();
}
void SequentialScheduler::OnLoop() {
	SequentialScheduler::Process();
}
void SequentialScheduler::OnStop() {
	SequentialScheduler::Stop();
}
bool SequentialScheduler::IsDone() {
	/* Have to return something to know if we are done */
	if (_running == false)
		return true;
	else
		return false;
}

} // namespace schedulers
} // namespace tasking
} // namespace phoenix
} // namespace ctre

