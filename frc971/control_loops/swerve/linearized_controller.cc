#include "frc971/control_loops/swerve/linearized_controller.h"

ABSL_FLAG(bool, use_slicot, false,
          "Whether to use the SLICOT DARE solver for the linearized controller "
          "class.");
ABSL_FLAG(int, dare_iterations, 3,
          "Number of iterations to use for a finite-horizon LQR.");
