#pragma once

namespace ctre {
namespace phoenix {
namespace signals {

class IInvertable {
public:
	virtual ~IInvertable(){}
	virtual void SetInverted(bool invert) = 0;
	virtual bool GetInverted() const = 0;
};

} // namespace  Signals
} // namespace phoenix
} // namespace ctre
