package frc971;

// Values retrieved from the PDP.
// Published on ".frc971.pdp_values"
message PDPValues {
  double voltage;
  double temperature;
  double power;
  double[16] currents;
};
