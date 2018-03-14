#pragma once

namespace ctre { namespace phoenix { namespace tasking {
	
class ILoopable{
public:
	virtual ~ILoopable(){}
	virtual void OnStart() = 0;
	virtual void OnLoop() = 0;
	virtual bool IsDone() = 0;
	virtual void OnStop() = 0;
};
}}}
