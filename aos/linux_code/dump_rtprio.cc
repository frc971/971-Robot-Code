// This is a utility program that prints out realtime priorities for processes
// on a system. It is useful both because the standard tools don't format that
// information very well and the roboRIO's busybox ones don't seem to do it at
// all.
//
// The output format is the following comma-separated columns:
// exe,name,cpumask,policy,nice,priority,tid,pid,ppid,sid,cpu

#include <sched.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

#include <string>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/implementations.h"
#include "aos/common/time.h"

namespace {

const char *policy_string(uint32_t policy) {
  switch (policy) {
    case SCHED_OTHER:
      return "OTHER";
    case SCHED_BATCH:
      return "BATCH";
    case SCHED_IDLE:
      return "IDLE";
    case SCHED_FIFO:
      return "FIFO";
    case SCHED_RR:
      return "RR";
#ifdef SCHED_DEADLINE
    case SCHED_DEADLINE:
      return "DEADLINE";
#endif
    default:
      return "???";
  }
}

::std::string strip_string_prefix(size_t length, ::std::string str) {
  str = str.substr(length);
  while (str[0] == ' ' || str[0] == '\t') {
    str = str.substr(1);
  }
  return str.substr(0, str.size() - 1);
}

int find_pid_max() {
  int r;
  FILE *pid_max_file = fopen("/proc/sys/kernel/pid_max", "r");
  if (pid_max_file == nullptr) {
    PLOG(FATAL, "fopen(\"/proc/sys/kernel/pid_max\")");
  }
  CHECK_EQ(1, fscanf(pid_max_file, "%d", &r));
  PCHECK(fclose(pid_max_file));
  return r;
}

cpu_set_t find_all_cpus() {
  long nproc = sysconf(_SC_NPROCESSORS_CONF);
  if (nproc == -1) {
    PLOG(FATAL, "sysconf(_SC_NPROCESSORS_CONF)");
  }
  cpu_set_t r;
  CPU_ZERO(&r);
  for (long i = 0; i < nproc; ++i) {
    CPU_SET(i, &r);
  }
  return r;
}

cpu_set_t find_cpu_mask(int process, bool *not_there) {
  cpu_set_t r;
  const int result = sched_getaffinity(process, sizeof(r), &r);
  if (result == -1 && errno == ESRCH) {
    *not_there = true;
    return cpu_set_t();
  }
  if (result != 0) {
    PLOG(FATAL, "sched_getaffinity(%d, %zu, %p)", process, sizeof(r), &r);
  }
  return r;
}

sched_param find_sched_param(int process, bool *not_there) {
  sched_param r;
  const int result = sched_getparam(process, &r);
  if (result == -1 && errno == ESRCH) {
    *not_there = true;
    return sched_param();
  }
  if (result != 0) {
    PLOG(FATAL, "sched_getparam(%d)", process);
  }
  return r;
}

int find_scheduler(int process, bool *not_there) {
  int scheduler = sched_getscheduler(process);
  if (scheduler == -1 && errno == ESRCH) {
    *not_there = true;
    return 0;
  }
  if (scheduler == -1) {
    PLOG(FATAL, "sched_getscheduler(%d)", process);
  }
  return scheduler;
}

::std::string find_exe(int process, bool *not_there) {
  ::std::string exe_filename = "/proc/" + ::std::to_string(process) + "/exe";
  char exe_buffer[1024];
  ssize_t exe_size =
      readlink(exe_filename.c_str(), exe_buffer, sizeof(exe_buffer));
  if (exe_size == -1 && errno == ENOENT) {
    return "ENOENT";
  } else {
    if (exe_size == -1 && errno == ESRCH) {
      *not_there = true;
      return "";
    }
    if (exe_size == -1) {
      PLOG(FATAL, "readlink(%s, %p, %zu)", exe_filename.c_str(), exe_buffer,
           sizeof(exe_buffer));
    }
    return ::std::string(exe_buffer, exe_size);
  }
}

int find_nice_value(int process, bool *not_there) {
  errno = 0;
  int nice_value = getpriority(PRIO_PROCESS, process);
  if (errno == ESRCH) {
    *not_there = true;
    return 0;
  }
  if (errno != 0) {
    PLOG(FATAL, "getpriority(PRIO_PROCESS, %d)", process);
  }
  return nice_value;
}

void read_stat(int process, int *ppid, int *sid, bool *not_there) {
  ::std::string stat_filename = "/proc/" + ::std::to_string(process) + "/stat";
  FILE *stat = fopen(stat_filename.c_str(), "r");
  if (stat == nullptr && errno == ENOENT) {
    *not_there = true;
    return;
  }
  if (stat == nullptr) {
    PLOG(FATAL, "fopen(%s, \"r\")", stat_filename.c_str());
  }

  char buffer[2048];
  if (fgets(buffer, sizeof(buffer), stat) == nullptr) {
    if (ferror(stat)) {
      if (errno == ESRCH) {
        *not_there = true;
        return;
      }
      PLOG(FATAL, "fgets(%p, %zu, %p)", buffer, sizeof(buffer), stat);
    }
  }

  int pid = 0;

  int field = 0;
  size_t field_start = 0;
  int parens = 0;
  for (size_t i = 0; i < sizeof(buffer); ++i) {
    if (buffer[i] == '\0') break;
    if (buffer[i] == '(') ++parens;
    if (parens > 0) {
      if (buffer[i] == ')') --parens;
    } else if (buffer[i] == ' ') {
      ::std::string field_string(buffer, field_start, i - field_start);
      switch (field) {
        case 0:
          pid = ::std::stoi(field_string);
          break;
        case 3:
          *ppid = ::std::stoi(field_string);
          break;
        case 4:
          *sid = ::std::stoi(field_string);
          break;
        default:
          break;
      }
      ++field;
      field_start = i + 1;
    }
  }
  PCHECK(fclose(stat));

  if (field < 4) {
    LOG(FATAL, "couldn't get fields from /proc/%d/stat\n", process);
  }
  CHECK_EQ(pid, process);
}

void read_status(int process, int ppid, int *pgrp, ::std::string *name,
                 bool *not_there) {
  ::std::string status_filename =
      "/proc/" + ::std::to_string(process) + "/status";
  FILE *status = fopen(status_filename.c_str(), "r");
  if (status == nullptr && errno == ENOENT) {
    *not_there = true;
    return;
  }
  if (status == nullptr) {
    PLOG(FATAL, "fopen(%s, \"r\")", status_filename.c_str());
  }

  int pid = 0, status_ppid = 0;
  while (true) {
    char buffer[1024];
    if (fgets(buffer, sizeof(buffer), status) == nullptr) {
      if (ferror(status)) {
        PLOG(FATAL, "fgets(%p, %zu, %p)", buffer, sizeof(buffer), status);
      } else {
        break;
      }
    }
    ::std::string line(buffer);
    if (line.substr(0, 5) == "Name:") {
      *name = strip_string_prefix(5, line);
    } else if (line.substr(0, 4) == "Pid:") {
      pid = ::std::stoi(strip_string_prefix(4, line));
    } else if (line.substr(0, 5) == "PPid:") {
      status_ppid = ::std::stoi(strip_string_prefix(5, line));
    } else if (line.substr(0, 5) == "Tgid:") {
      *pgrp = ::std::stoi(strip_string_prefix(5, line));
    }
  }
  PCHECK(fclose(status));
  CHECK_EQ(pid, process);
  CHECK_EQ(status_ppid, ppid);
}

}  // namespace

int main() {
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  const int pid_max = find_pid_max();
  const cpu_set_t all_cpus = find_all_cpus();

  for (int i = 0; i < pid_max; ++i) {
    bool not_there = false;

    const cpu_set_t cpu_mask = find_cpu_mask(i, &not_there);
    const sched_param param = find_sched_param(i, &not_there);
    const int scheduler = find_scheduler(i, &not_there);
    const ::std::string exe = find_exe(i, &not_there);
    const int nice_value = find_nice_value(i, &not_there);

    int ppid = 0, sid = 0;
    read_stat(i, &ppid, &sid, &not_there);

    int pgrp = 0;
    ::std::string name;
    read_status(i, ppid, &pgrp, &name, &not_there);

    if (not_there) continue;

    const char *cpu_mask_string =
        CPU_EQUAL(&cpu_mask, &all_cpus) ? "all" : "???";

    printf("%s,%s,%s,%s,%d,%d,%d,%d,%d,%d\n", exe.c_str(), name.c_str(),
           cpu_mask_string, policy_string(scheduler), nice_value,
           param.sched_priority, i, pgrp, ppid, sid);
  }
}
