package frc971.logging_structs;

struct CapeReading {
  uint32_t sec;
  uint32_t nsec;
  uint64_t struct_size;
  double sonar;

  uint16_t left_low;
  uint16_t left_high;
  uint16_t right_low;
  uint16_t right_high;
};
