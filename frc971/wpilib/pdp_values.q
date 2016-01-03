package frc971;

// Values retrieved from the PDP.
message PDPValues {
  double voltage;
  double temperature;
  double power;
  double[16] currents;
};

queue PDPValues pdp_values;
