// This is a utility program that prints out realtime priorities for processes
// on a system. It is useful both because the standard tools don't format that
// information very well and the roboRIO's busybox ones don't seem to do it at
// all.
//
// The output format is the following comma-separated columns:
// exe,name,cpumask,policy,nice,priority,tid,pid,ppid,sid
// The threads in the output are sorted by realtime priority.

#include <sched.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>

#include <bitset>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>

#include "glog/logging.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "aos/util/top.h"

DEFINE_string(config, "aos_config.json", "File path of aos configuration");

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

cpu_set_t FindAllCpus() {
  long nproc = sysconf(_SC_NPROCESSORS_CONF);
  PCHECK(nproc != -1);
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
  PCHECK(result == 0) << ": sched_getaffinity of " << process;
  return r;
}

sched_param find_sched_param(int process, bool *not_there) {
  sched_param r;
  const int result = sched_getparam(process, &r);
  if (result == -1 && errno == ESRCH) {
    *not_there = true;
    return sched_param();
  }
  PCHECK(result == 0) << ": sched_getparam of " << process;
  return r;
}

int find_scheduler(int process, bool *not_there) {
  int scheduler = sched_getscheduler(process);
  if (scheduler == -1 && errno == ESRCH) {
    *not_there = true;
    return 0;
  }
  PCHECK(scheduler != -1) << ": sched_getscheduler of " << process;
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
    PCHECK(exe_size != -1) << ": readlink " << exe_filename
                           << " into buffer of size " << sizeof(exe_buffer);
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
  PCHECK(errno == 0) << "getpriority of " << process;
  return nice_value;
}

void read_stat(int process, int *ppid, int *sid, bool *not_there) {
  ::std::string stat_filename = "/proc/" + ::std::to_string(process) + "/stat";
  FILE *stat = fopen(stat_filename.c_str(), "r");
  if (stat == nullptr && errno == ENOENT) {
    *not_there = true;
    return;
  }
  PCHECK(stat != nullptr) << ": Failed to open " << stat_filename;

  char buffer[2048];
  if (fgets(buffer, sizeof(buffer), stat) == nullptr) {
    if (ferror(stat)) {
      if (errno == ESRCH) {
        *not_there = true;
        return;
      }
      PLOG(FATAL) << "reading from " << stat_filename << " into buffer of size "
                  << sizeof(buffer);
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
  PCHECK(fclose(stat) == 0);

  if (field < 4) {
    LOG(FATAL) << "couldn't get fields from /proc/" << process << "/stat";
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
  PCHECK(status != nullptr) << ": Failed to open " << status_filename;

  int pid = 0, status_ppid = 0;
  while (true) {
    char buffer[1024];
    if (fgets(buffer, sizeof(buffer), status) == nullptr) {
      if (ferror(status)) {
        PLOG(FATAL) << "reading from " << status_filename
                    << " into buffer of size " << sizeof(buffer);
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
  PCHECK(fclose(status) == 0);
  CHECK_EQ(pid, process);
  CHECK_EQ(status_ppid, ppid);
}

struct Thread {
  uint32_t policy;
  std::string exe, name, cpu_mask;
  int nice_value, sched_priority, tid, pid, ppid, sid;

  void Print() const {
    printf("%s,%s,%s,%s,%d,%d,%d,%d,%d,%d\n", exe.c_str(), name.c_str(),
           cpu_mask.c_str(), policy_string(policy), nice_value, sched_priority,
           tid, pid, ppid, sid);
  }

  // Make threads with SCHED_FIFO or SCHED_RR show up before SCHED_OTHER, and
  // make higher priority threads show up first. Higher priority threads are
  // less than lower priority so that they appear first.
  bool operator<(const Thread &t) const {
    return (((((policy == SCHED_FIFO) || (policy == SCHED_RR))) &&
             (t.policy == SCHED_OTHER)) ||
            (sched_priority > t.sched_priority));
  }
};

}  // namespace

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());
  event_loop.SkipTimingReport();
  event_loop.SkipAosLog();
  aos::util::Top top(&event_loop, true);
  top.set_track_top_processes(true);

  const cpu_set_t all_cpus = FindAllCpus();

  top.set_on_reading_update([&]() {
    std::multiset<Thread> threads;

    for (const std::pair<const pid_t, aos::util::Top::ProcessReadings>
             &reading : top.readings()) {
      pid_t tid = reading.first;
      bool not_there = false;

      const cpu_set_t cpu_mask = find_cpu_mask(tid, &not_there);
      const sched_param param = find_sched_param(tid, &not_there);
      const int scheduler = find_scheduler(tid, &not_there);
      const ::std::string exe = find_exe(tid, &not_there);
      const int nice_value = find_nice_value(tid, &not_there);

      int ppid = 0, sid = 0;
      read_stat(tid, &ppid, &sid, &not_there);

      int pgrp = 0;
      ::std::string name;
      read_status(tid, ppid, &pgrp, &name, &not_there);

      if (not_there) continue;

      std::string_view cpu_mask_string;
      if (CPU_EQUAL(&cpu_mask, &all_cpus)) {
        cpu_mask_string = "all";
      } else {
        // Convert the CPU mask to a bitset
        std::bitset<CPU_SETSIZE> bitset;
        for (size_t i = 0; i < CPU_SETSIZE; i++) {
          bitset[i] = CPU_ISSET(i, &cpu_mask);
        }
        std::stringstream ss;
        ss << std::hex << bitset.to_ulong();
        cpu_mask_string = ss.str();
      }

      threads.emplace(Thread{.policy = static_cast<uint32_t>(scheduler),
                             .exe = exe,
                             .name = name,
                             .cpu_mask = cpu_mask_string.data(),
                             .nice_value = nice_value,
                             .sched_priority = param.sched_priority,
                             .tid = tid,
                             .pid = pgrp,
                             .ppid = ppid,
                             .sid = sid});
    }

    printf("exe,name,cpumask,policy,nice,priority,tid,pid,ppid,sid\n");
    for (const auto &t : threads) {
      t.Print();
    }
    event_loop.Exit();
  });

  event_loop.Run();
}
