#pragma once
namespace ctre { namespace phoenix { namespace tasking{
	
class IProcessable {
public:
	virtual ~IProcessable(){}
	virtual void Process() = 0;
};
}}}
