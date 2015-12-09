#include <stdlib.h>

int main() {
  void *big_chunk = malloc(500000);
  free(big_chunk);
}
