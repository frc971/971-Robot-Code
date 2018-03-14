#pragma once

namespace ctre {
namespace phoenix {
namespace signals {

class IOutputSignal {
public:
	virtual ~IOutputSignal(){}
	virtual void Set(double value) = 0;
};

} // namespace  Signals
} // namespace phoenix
} // namespace ctre
