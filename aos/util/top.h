#ifndef AOS_UTIL_TOP_H_
#define AOS_UTIL_TOP_H_

#include <map>
#include <string>

#include "aos/containers/ring_buffer.h"
#include "aos/events/event_loop.h"
#include "aos/util/process_info_generated.h"

namespace aos::util {

// ProcStat is a struct to hold all the fields available in /proc/[pid]/stat.
// Currently we only use a small subset of the feilds. See man 5 proc for
// details on what the fields are--these are in the same order as they appear in
// the stat file.
//
// Things are signed or unsigned based on whether they are listed
// as signed/unsigned in man 5 proc. We just make everything 64 bits wide
// because otherwise we have to write out way too many casts everywhere.
struct ProcStat {
  int pid;
  std::string name;
  char state;
  int64_t parent_pid;
  int64_t group_id;
  int64_t session_id;
  int64_t tty;
  int64_t tpgid;
  uint64_t kernel_flags;
  uint64_t minor_faults;
  uint64_t children_minor_faults;
  uint64_t major_faults;
  uint64_t children_major_faults;
  uint64_t user_mode_ticks;
  uint64_t kernel_mode_ticks;
  int64_t children_user_mode_ticks;
  int64_t children_kernel_mode_ticks;
  int64_t priority;
  int64_t nice;
  int64_t num_threads;
  int64_t itrealvalue;  // always zero.
  uint64_t start_time_ticks;
  uint64_t virtual_memory_size;
  // Number of pages in real memory.
  int64_t resident_set_size;
  uint64_t rss_soft_limit;
  uint64_t start_code_address;
  uint64_t end_code_address;
  uint64_t start_stack_address;
  uint64_t stack_pointer;
  uint64_t instruction_pointer;
  uint64_t signal_bitmask;
  uint64_t blocked_signals;
  uint64_t ignored_signals;
  uint64_t caught_signals;
  uint64_t wchan;
  // swap_pages fields are not maintained.
  uint64_t swap_pages;
  uint64_t children_swap_pages;
  int64_t exit_signal;
  // CPU number last exitted on.
  int64_t processor;
  // Zero for non-realtime processes.
  uint64_t rt_priority;
  uint64_t scheduling_policy;
  // Aggregated block I/O delay.
  uint64_t block_io_delay_ticks;
  uint64_t guest_ticks;
  uint64_t children_guest_ticks;
  uint64_t start_data_address;
  uint64_t end_data_address;
  uint64_t start_brk_address;
  uint64_t start_arg_address;
  uint64_t end_arg_address;
  uint64_t start_env_address;
  uint64_t end_env_address;
  int64_t exit_code;
};

// Retrieves the stats for a particular process (note that there also exists a
// /proc/[pid]/task/[tid]/stat with the same format for per-thread information;
// we currently do not read that).
// Returns nullopt if unable to read/parse the file.
std::optional<ProcStat> ReadProcStat(int pid);

// This class provides a basic utility for retrieving general performance
// information on running processes (named after the top utility). It can either
// be used to directly get information on individual processes (via
// set_track_pids()) or used to track a list of the top N processes with the
// highest CPU usage.
// Note that this currently relies on sampling processes in /proc every second
// and using the differences between the two readings to calculate CPU usage.
// For crash-looping processees or other situations with highly variable or
// extremely short-lived loads, this may do a poor job of capturing information.
class Top {
 public:
  // A snapshot of the resource usage of a process.
  struct Reading {
    aos::monotonic_clock::time_point reading_time;
    std::chrono::nanoseconds total_run_time;
    // Memory usage in bytes.
    uint64_t memory_usage;
  };

  // All the information we have about a process.
  struct ProcessReadings {
    std::string name;
    aos::monotonic_clock::time_point start_time;
    // CPU usage is based on the past two readings.
    double cpu_percent;
    // True if this is a kernel thread, false if this is a userspace thread.
    bool kthread;
    // Last 2 readings
    aos::RingBuffer<Reading, 2> readings;
  };

  Top(aos::EventLoop *event_loop, bool track_threads = false);

  // Set whether to track all the top processes (this will result in us having
  // to track every single process on the system, so that we can sort them).
  void set_track_top_processes(bool track_all) { track_all_ = track_all; }

  void set_on_reading_update(std::function<void()> fn) {
    on_reading_update_ = std::move(fn);
  }

  // Specify a set of individual processes to track statistics for.
  // This can be changed at run-time, although it may take up to kSamplePeriod
  // to have full statistics on all the relevant processes, since we need at
  // least two samples to estimate CPU usage.
  void set_track_pids(const std::set<pid_t> &pids) { pids_to_track_ = pids; }

  // Retrieve statistics for the specified process. Will return the null offset
  // of no such pid is being tracked.
  flatbuffers::Offset<ProcessInfo> InfoForProcess(
      flatbuffers::FlatBufferBuilder *fbb, pid_t pid);

  // Returns information on up to n processes, sorted by CPU usage.
  flatbuffers::Offset<TopProcessesFbs> TopProcesses(
      flatbuffers::FlatBufferBuilder *fbb, int n);

  const std::map<pid_t, ProcessReadings> &readings() const { return readings_; }

 private:
  // Rate at which to sample /proc/[pid]/stat.
  static constexpr std::chrono::seconds kSamplePeriod{1};

  std::chrono::nanoseconds TotalProcessTime(const ProcStat &proc_stat);
  aos::monotonic_clock::time_point ProcessStartTime(const ProcStat &proc_stat);
  uint64_t RealMemoryUsage(const ProcStat &proc_stat);
  void UpdateReadings();
  // Adds thread ids for the given pid to the pids set,
  // if we are tracking threads.
  void MaybeAddThreadIds(pid_t pid, std::set<pid_t> *pids);

  aos::EventLoop *event_loop_;

  // Length of a clock tick (used to convert from raw numbers in /proc to actual
  // times).
  const std::chrono::nanoseconds clock_tick_;
  // Page size, in bytes, on the current system.
  const long page_size_;

  std::set<pid_t> pids_to_track_;
  bool track_all_ = false;
  bool track_threads_;

  std::map<pid_t, ProcessReadings> readings_;

  std::function<void()> on_reading_update_;
};

}  // namespace aos::util
#endif  // AOS_UTIL_TOP_H_
