package aos.common.testing;

message TestingMessage {
  bool test_bool;
  int32_t test_int;
};

message OtherTestingMessage {
  bool test_bool;
  int32_t test_int;
  double test_double;
};

queue TestingMessage test_queue;

queue_group TwoQueues {
  queue TestingMessage first;
  queue OtherTestingMessage second;
};

queue_group TwoQueues test_queuegroup;
