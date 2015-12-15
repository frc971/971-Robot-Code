#include <stdio.h>

#include "build_tests/queue.q.h"

int main() {
  ::build_tests::TestMessage s;
  s.field = 971;
  char buffer[1024];
  s.Print(buffer, sizeof(buffer));
  buffer[sizeof(buffer) - 1] = '\0';
  printf("s = {%s}\n", buffer);
}
