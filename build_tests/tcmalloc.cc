#include <stdlib.h>

int main() { void *big_chunk __attribute__((unused)) = malloc(500000); }
