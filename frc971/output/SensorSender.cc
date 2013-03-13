#include "aos/crio/motor_server/SensorSender.h"
#include "aos/aos_core.h"

#include "frc971/queues/sensor_values.h"

AOS_RUN_FORK(aos::SensorSender<frc971::sensor_values>, "971SS", 100)
