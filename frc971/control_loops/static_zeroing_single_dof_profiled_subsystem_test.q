package frc971.control_loops;

import "frc971/control_loops/profiled_subsystem.q";

message StaticZeroingSingleDOFProfiledSubsystemTestGoal {
  double unsafe_goal;
  .frc971.ProfileParameters profile_params;
};

message StaticZeroingSingleDOFProfiledSubsystemOutput {
  double output;
};

queue_group StaticZeroingSingleDOFProfiledSubsystemPotAndAbsoluteEncoderTestQueueGroup {
  implements aos.control_loops.ControlLoop;

  message Status {
    PotAndAbsoluteEncoderProfiledJointStatus status;
  };

  message Position {
    PotAndAbsolutePosition position;
  };

  queue StaticZeroingSingleDOFProfiledSubsystemTestGoal goal;
  queue StaticZeroingSingleDOFProfiledSubsystemOutput output;
  queue Status status;
  queue Position position;
};

queue_group StaticZeroingSingleDOFProfiledSubsystemAbsoluteEncoderTestQueueGroup {
  implements aos.control_loops.ControlLoop;

  message Status {
    AbsoluteEncoderProfiledJointStatus status;
  };

  message Position {
    AbsolutePosition position;
  };

  queue StaticZeroingSingleDOFProfiledSubsystemTestGoal goal;
  queue StaticZeroingSingleDOFProfiledSubsystemOutput output;
  queue Status status;
  queue Position position;
};
