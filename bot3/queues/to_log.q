package frc971.logging_structs;

struct CapeReading {
  Time time;
  uint16_t struct_size;

  uint16_t left_low;
  uint16_t left_high;
  uint16_t right_low;
  uint16_t right_high;
};
