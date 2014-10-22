package aos.common.testing;

struct Structure {
  bool struct_bool;
  uint16_t struct_int;
  float struct_float;
};

message MessageWithStructure {
  bool other_member;
  Structure struct1;
  Structure struct2;
};

message TestingMessage {
  bool test_bool;
  int32_t test_int;
};

message OtherTestingMessage {
  bool test_bool;
  int32_t test_int;
  double test_double;
};

message MessageWithArrays {
  uint16_t[3] test_int;
  Structure[2] test_struct;
};

queue TestingMessage test_queue;

queue_group TwoQueues {
  queue TestingMessage first;
  queue OtherTestingMessage second;
};

queue_group TwoQueues test_queuegroup;
