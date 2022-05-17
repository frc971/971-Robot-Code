#include "aos/util/top.h"

#include <dirent.h>
#include <unistd.h>

#include <queue>
#include <string>

#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"

namespace aos::util {
namespace {
std::optional<std::string> ReadShortFile(std::string_view file_name) {
  // Open as input and seek to end immediately.
  std::ifstream file(std::string(file_name), std::ios_base::in);
  if (!file.good()) {
    VLOG(1) << "Can't read " << file_name;
    return std::nullopt;
  }
  const size_t kMaxLineLength = 4096;
  char buffer[kMaxLineLength];
  file.read(buffer, kMaxLineLength);
  if (!file.eof()) {
    return std::nullopt;
  }
  return std::string(buffer, file.gcount());
}
}  // namespace

std::optional<ProcStat> ReadProcStat(pid_t pid) {
  std::optional<std::string> contents =
      ReadShortFile(absl::StrFormat("/proc/%d/stat", pid));
  if (!contents.has_value()) {
    return std::nullopt;
  }
  const size_t start_name = contents->find_first_of('(');
  const size_t end_name = contents->find_last_of(')');
  if (start_name == std::string::npos || end_name == std::string::npos ||
      end_name < start_name) {
    VLOG(1) << "No name found in stat line " << contents.value();
    return std::nullopt;
  }
  std::string_view name(contents->c_str() + start_name + 1,
                        end_name - start_name - 1);

  std::vector<std::string_view> fields =
      absl::StrSplit(std::string_view(contents->c_str() + end_name + 1,
                                      contents->size() - end_name - 1),
                     ' ', absl::SkipWhitespace());
  constexpr int kNumFieldsAfterName = 50;
  if (fields.size() != kNumFieldsAfterName) {
    VLOG(1) << "Incorrect number of fields " << fields.size();
    return std::nullopt;
  }
  // The first field is a character for the current process state; every single
  // field after that should be an integer.
  if (fields[0].size() != 1) {
    VLOG(1) << "State field is too long: " << fields[0];
    return std::nullopt;
  }
  std::array<absl::int128, kNumFieldsAfterName - 1> numbers;
  for (int ii = 1; ii < kNumFieldsAfterName; ++ii) {
    if (!absl::SimpleAtoi(fields[ii], &numbers[ii - 1])) {
      VLOG(1) << "Failed to parse field " << ii << " as number: " << fields[ii];
      return std::nullopt;
    }
  }
  return ProcStat{
      .pid = pid,
      .name = std::string(name),
      .state = fields.at(0).at(0),
      .parent_pid = static_cast<int64_t>(numbers.at(0)),
      .group_id = static_cast<int64_t>(numbers.at(1)),
      .session_id = static_cast<int64_t>(numbers.at(2)),
      .tty = static_cast<int64_t>(numbers.at(3)),
      .tpgid = static_cast<int64_t>(numbers.at(4)),
      .kernel_flags = static_cast<uint64_t>(numbers.at(5)),
      .minor_faults = static_cast<uint64_t>(numbers.at(6)),
      .children_minor_faults = static_cast<uint64_t>(numbers.at(7)),
      .major_faults = static_cast<uint64_t>(numbers.at(8)),
      .children_major_faults = static_cast<uint64_t>(numbers.at(9)),
      .user_mode_ticks = static_cast<uint64_t>(numbers.at(10)),
      .kernel_mode_ticks = static_cast<uint64_t>(numbers.at(11)),
      .children_user_mode_ticks = static_cast<int64_t>(numbers.at(12)),
      .children_kernel_mode_ticks = static_cast<int64_t>(numbers.at(13)),
      .priority = static_cast<int64_t>(numbers.at(14)),
      .nice = static_cast<int64_t>(numbers.at(15)),
      .num_threads = static_cast<int64_t>(numbers.at(16)),
      .itrealvalue = static_cast<int64_t>(numbers.at(17)),
      .start_time_ticks = static_cast<uint64_t>(numbers.at(18)),
      .virtual_memory_size = static_cast<uint64_t>(numbers.at(19)),
      .resident_set_size = static_cast<int64_t>(numbers.at(20)),
      .rss_soft_limit = static_cast<uint64_t>(numbers.at(21)),
      .start_code_address = static_cast<uint64_t>(numbers.at(22)),
      .end_code_address = static_cast<uint64_t>(numbers.at(23)),
      .start_stack_address = static_cast<uint64_t>(numbers.at(24)),
      .stack_pointer = static_cast<uint64_t>(numbers.at(25)),
      .instruction_pointer = static_cast<uint64_t>(numbers.at(26)),
      .signal_bitmask = static_cast<uint64_t>(numbers.at(27)),
      .blocked_signals = static_cast<uint64_t>(numbers.at(28)),
      .ignored_signals = static_cast<uint64_t>(numbers.at(29)),
      .caught_signals = static_cast<uint64_t>(numbers.at(30)),
      .wchan = static_cast<uint64_t>(numbers.at(31)),
      .swap_pages = static_cast<uint64_t>(numbers.at(32)),
      .children_swap_pages = static_cast<uint64_t>(numbers.at(33)),
      .exit_signal = static_cast<int64_t>(numbers.at(34)),
      .processor = static_cast<int64_t>(numbers.at(35)),
      .rt_priority = static_cast<uint64_t>(numbers.at(36)),
      .scheduling_policy = static_cast<uint64_t>(numbers.at(37)),
      .block_io_delay_ticks = static_cast<uint64_t>(numbers.at(38)),
      .guest_ticks = static_cast<uint64_t>(numbers.at(39)),
      .children_guest_ticks = static_cast<uint64_t>(numbers.at(40)),
      .start_data_address = static_cast<uint64_t>(numbers.at(41)),
      .end_data_address = static_cast<uint64_t>(numbers.at(42)),
      .start_brk_address = static_cast<uint64_t>(numbers.at(43)),
      .start_arg_address = static_cast<uint64_t>(numbers.at(44)),
      .end_arg_address = static_cast<uint64_t>(numbers.at(45)),
      .start_env_address = static_cast<uint64_t>(numbers.at(46)),
      .end_env_address = static_cast<uint64_t>(numbers.at(47)),
      .exit_code = static_cast<int64_t>(numbers.at(48))};
}

Top::Top(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      clock_tick_(std::chrono::nanoseconds(1000000000 / sysconf(_SC_CLK_TCK))),
      page_size_(sysconf(_SC_PAGESIZE)) {
  TimerHandler *timer = event_loop_->AddTimer([this]() { UpdateReadings(); });
  event_loop_->OnRun([timer, this]() {
    timer->Setup(event_loop_->monotonic_now(), kSamplePeriod);
  });
}

std::chrono::nanoseconds Top::TotalProcessTime(const ProcStat &proc_stat) {
  return (proc_stat.user_mode_ticks + proc_stat.kernel_mode_ticks) *
         clock_tick_;
}

aos::monotonic_clock::time_point Top::ProcessStartTime(
    const ProcStat &proc_stat) {
  return aos::monotonic_clock::time_point(proc_stat.start_time_ticks *
                                          clock_tick_);
}

uint64_t Top::RealMemoryUsage(const ProcStat &proc_stat) {
  return proc_stat.resident_set_size * page_size_;
}

void Top::UpdateReadings() {
  aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
  // Get all the processes that we *might* care about.
  std::set<pid_t> pids = pids_to_track_;
  // Ensure that we check on the status of every process that we are already
  // tracking.
  for (const auto &reading : readings_) {
    pids.insert(reading.first);
  }
  if (track_all_) {
    DIR *const dir = opendir("/proc");
    if (dir == nullptr) {
      PLOG(FATAL) << "Failed to open /proc";
    }
    while (true) {
      struct dirent *const dir_entry = readdir(dir);
      if (dir_entry == nullptr) {
        break;
      }
      pid_t pid;
      if (dir_entry->d_type == DT_DIR &&
          absl::SimpleAtoi(dir_entry->d_name, &pid)) {
        pids.insert(pid);
      }
    }
  }

  for (const pid_t pid : pids) {
    std::optional<ProcStat> proc_stat = ReadProcStat(pid);
    // Stop tracking processes that have died.
    if (!proc_stat.has_value()) {
      readings_.erase(pid);
      continue;
    }
    const aos::monotonic_clock::time_point start_time =
        ProcessStartTime(*proc_stat);
    auto reading_iter = readings_.find(pid);
    if (reading_iter == readings_.end()) {
      reading_iter = readings_
                         .insert(std::make_pair(
                             pid, ProcessReadings{.name = proc_stat->name,
                                                  .start_time = start_time,
                                                  .cpu_percent = 0.0,
                                                  .readings = {}}))
                         .first;
    }
    ProcessReadings &process = reading_iter->second;
    // The process associated with the PID has changed; reset the state.
    if (process.start_time != start_time) {
      process.name = proc_stat->name;
      process.start_time = start_time;
      process.readings.Reset();
    }
    // If the process name has changed (e.g., if our first reading for a process
    // name occurred before execvp was called), then update it.
    if (process.name != proc_stat->name) {
      process.name = proc_stat->name;
    }

    process.readings.Push(Reading{now, TotalProcessTime(*proc_stat),
                                  RealMemoryUsage(*proc_stat)});
    if (process.readings.size() == 2) {
      process.cpu_percent =
          aos::time::DurationInSeconds(process.readings[1].total_run_time -
                                       process.readings[0].total_run_time) /
          aos::time::DurationInSeconds(process.readings[1].reading_time -
                                       process.readings[0].reading_time);
    } else {
      process.cpu_percent = 0.0;
    }
  }
}

flatbuffers::Offset<ProcessInfo> Top::InfoForProcess(
    flatbuffers::FlatBufferBuilder *fbb, pid_t pid) {
  auto reading_iter = readings_.find(pid);
  if (reading_iter == readings_.end()) {
    return {};
  }
  const ProcessReadings &reading = reading_iter->second;
  const flatbuffers::Offset<flatbuffers::String> name =
      fbb->CreateString(reading.name);
  ProcessInfo::Builder builder(*fbb);
  builder.add_pid(pid);
  builder.add_name(name);
  builder.add_cpu_usage(reading.cpu_percent);
  builder.add_physical_memory(
      reading.readings[reading.readings.size() - 1].memory_usage);
  return builder.Finish();
}

flatbuffers::Offset<TopProcessesFbs> Top::TopProcesses(
    flatbuffers::FlatBufferBuilder *fbb, int n) {
  // Pair is {cpu_usage, pid}.
  std::priority_queue<std::pair<double, pid_t>> cpu_usages;
  for (const auto &pair : readings_) {
    // Deliberately include 0.0 percent CPU things in the usage list so that if
    // the user asks for an arbitrarily large number of processes they'll get
    // everything.
    cpu_usages.push(std::make_pair(pair.second.cpu_percent, pair.first));
  }
  std::vector<flatbuffers::Offset<ProcessInfo>> offsets;
  for (int ii = 0; ii < n && !cpu_usages.empty(); ++ii) {
    offsets.push_back(InfoForProcess(fbb, cpu_usages.top().second));
    cpu_usages.pop();
  }
  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ProcessInfo>>>
      vector_offset = fbb->CreateVector(offsets);
  TopProcessesFbs::Builder builder(*fbb);
  builder.add_processes(vector_offset);
  return builder.Finish();
}

}  // namespace aos::util
