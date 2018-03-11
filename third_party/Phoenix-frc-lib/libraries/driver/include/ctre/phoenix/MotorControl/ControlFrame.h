#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

//{
enum ControlFrame {
	Control_3_General = 0x040080,
	Control_4_Advanced = 0x0400C0,
	Control_6_MotProfAddTrajPoint = 0x040140,
};

enum ControlFrameEnhanced {
	Control_3_General_ = 0x040080,
	Control_4_Advanced_ = 0x0400c0,
	Control_5_FeedbackOutputOverride_ = 0x040100,
	Control_6_MotProfAddTrajPoint_ = 0x040140,
};
class ControlFrameRoutines {
	static ControlFrameEnhanced Promote(ControlFrame controlFrame) {
		return (ControlFrameEnhanced) controlFrame;
	}
};

}
}
}

