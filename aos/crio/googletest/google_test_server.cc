#include <stdio.h>

#include "gtest/gtest.h"

extern "C" int run_gtest(char *arg1, char *arg2, char *arg3, char *arg4,
                         char *arg5, char *arg6, char *arg7, char *arg8,
                         char *arg9, char *arg10, char *arg11) {
  static bool run = false;
  if (!run) {
    run = true;
  } else {
    printf("error: gtest only supports being run once\n");
    return -1;
  }

  char *argv[1 + 11 + 1];
  // In /tmp in case it wants to write anything relative to "itself".
  argv[0] = const_cast<char *>("/tmp/aos-crio-googletest-runner");
  argv[12] = NULL; // the argv passed to main is always NULL-terminated
  argv[1] = arg1;
  argv[2] = arg2;
  argv[3] = arg3;
  argv[4] = arg4;
  argv[5] = arg5;
  argv[6] = arg6;
  argv[7] = arg7;
  argv[8] = arg8;
  argv[9] = arg9;
  argv[10] = arg10;
  argv[11] = arg11;
  int argc = 0;
  while (argc[argv] != NULL) ++argc;

  testing::GTEST_FLAG(color) = "yes";
  testing::InitGoogleTest(&argc, argv);

  if (argc > 1) {
    printf("warning: flags not recognized by gtest passed\n");
    for (int i = 1; i < argc; ++i) {
      printf("\t%s\n", argv[i]);
    }
  }

  return RUN_ALL_TESTS();
}
