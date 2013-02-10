#ifndef __AOS_STARTER_H_
#define __AOS_STARTER_H_

#include <map>
#include <sys/types.h>
#include <signal.h>
#include <stdint.h>
#include <string>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>
#include <set>

using namespace std;

map<pid_t, const char *> children;
map<pid_t, const char *> files; // the names of the actual files
map<int, pid_t> watches;
set<pid_t> restarts;
map<int, time_t> mtimes;

int sigfd = 0;
int notifyfd = 0;

const size_t CMDLEN = 5000;

#endif

