#include "build_tests/queue.q.h"

int main() {
  ::build_tests::TestStruct s;
  s.field = 971;
  return (s.field == 971) ? 0 : 1;
}
