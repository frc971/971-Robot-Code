#pragma once

namespace ctre {
namespace phoenix {
namespace sensors {

/** Enumerated type for status frame types. */
enum PigeonIMU_StatusFrame {
	PigeonIMU_CondStatus_1_General = 0x042000,
	PigeonIMU_CondStatus_9_SixDeg_YPR = 0x042200,
	PigeonIMU_CondStatus_6_SensorFusion = 0x042140,
	PigeonIMU_CondStatus_11_GyroAccum = 0x042280,
	PigeonIMU_CondStatus_2_GeneralCompass = 0x042040,
	PigeonIMU_CondStatus_3_GeneralAccel = 0x042080,
	PigeonIMU_CondStatus_10_SixDeg_Quat = 0x042240,
	PigeonIMU_RawStatus_4_Mag = 0x041CC0,
	PigeonIMU_BiasedStatus_2_Gyro = 0x041C40,
	PigeonIMU_BiasedStatus_4_Mag = 0x041CC0,
	PigeonIMU_BiasedStatus_6_Accel = 0x41D40,
};

} // namespace sensors
} // namespace phoenix
} // namespace ctre
