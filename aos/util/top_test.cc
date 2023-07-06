#include "aos/util/top.h"

#include <unistd.h>

#include <array>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "aos/events/shm_event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"

namespace aos::util::testing {

class TopTest : public ::testing::Test {
 protected:
  TopTest()
      : shm_dir_(aos::testing::TestTmpDir() + "/aos"),
        cpu_consumer_([this]() {
          while (!stop_flag_.load()) {
          }
        }),
        config_file_(
            aos::testing::ArtifactPath("aos/events/pingpong_config.json")),
        config_(aos::configuration::ReadConfig(config_file_)),
        event_loop_(&config_.message()) {
    FLAGS_shm_base = shm_dir_;

    // Nuke the shm dir, to ensure we aren't being affected by any preexisting
    // tests.
    aos::util::UnlinkRecursive(shm_dir_);
  }
  ~TopTest() {
    stop_flag_ = true;
    cpu_consumer_.join();
  }

  gflags::FlagSaver flag_saver_;
  std::string shm_dir_;

  std::thread cpu_consumer_;
  std::atomic<bool> stop_flag_{false};
  const std::string config_file_;
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::ShmEventLoop event_loop_;
};

TEST_F(TopTest, TestSelfStat) {
  const pid_t pid = getpid();
  std::optional<ProcStat> proc_stat = ReadProcStat(pid);
  ASSERT_TRUE(proc_stat.has_value());
  ASSERT_EQ(pid, proc_stat->pid);
  ASSERT_EQ("top_test", proc_stat->name);
  ASSERT_EQ('R', proc_stat->state);
  ASSERT_LT(1, proc_stat->num_threads);
}

TEST_F(TopTest, QuerySingleProcess) {
  const pid_t pid = getpid();
  Top top(&event_loop_);
  top.set_track_pids({pid});
  event_loop_.AddTimer([this]() { event_loop_.Exit(); })
      ->Schedule(event_loop_.monotonic_now() + std::chrono::seconds(2));
  event_loop_.Run();
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.Finish(top.InfoForProcess(&fbb, pid));
  aos::FlatbufferDetachedBuffer<ProcessInfo> info = fbb.Release();
  ASSERT_EQ(pid, info.message().pid());
  ASSERT_TRUE(info.message().has_name());
  ASSERT_EQ("top_test", info.message().name()->string_view());
  // Check that we did indeed consume ~1 CPU core (because we're multi-threaded,
  // we could've consumed a bit more; and on systems where we are competing with
  // other processes for CPU time, we may not get a full 100% load).
  ASSERT_LT(0.5, info.message().cpu_usage());
  ASSERT_GT(1.1, info.message().cpu_usage());
  // Sanity check memory usage.
  ASSERT_LT(1000000, info.message().physical_memory());
  ASSERT_GT(1000000000, info.message().physical_memory());
}

TEST_F(TopTest, TopProcesses) {
  // Make some dummy processes that will just spin and get killed off at the
  // end, so that we actually have things to query.
  constexpr int kNProcesses = 2;
  std::vector<pid_t> children;
  // This will create kNProcesses children + ourself, which means we have enough
  // processes to test that we correctly exclude extras when requesting fewer
  // processes than exist.
  for (int ii = 0; ii < kNProcesses; ++ii) {
    const pid_t pid = fork();
    PCHECK(pid >= 0);
    if (pid == 0) {
      LOG(INFO) << "In child process.";
      while (true) {
        // This is a "please don't optimize me out" thing for the compiler.
        // Otherwise, the entire if (pid == 0) block can get optimized away...
        asm("");
        continue;
      }
      LOG(FATAL) << "This should be unreachable.";
    } else {
      CHECK_NE(0, pid) << "The compiler is messing with you.";
      children.push_back(pid);
    }
  }

  Top top(&event_loop_);
  top.set_track_top_processes(true);
  event_loop_.AddTimer([this]() { event_loop_.Exit(); })
      ->Schedule(event_loop_.monotonic_now() + std::chrono::seconds(2));
  event_loop_.SkipTimingReport();
  event_loop_.SkipAosLog();
  event_loop_.Run();
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.Finish(top.TopProcesses(&fbb, kNProcesses));
  aos::FlatbufferDetachedBuffer<TopProcessesFbs> info = fbb.Release();
  ASSERT_EQ(kNProcesses, info.message().processes()->size());
  double last_cpu = std::numeric_limits<double>::infinity();
  std::set<pid_t> observed_pids;
  int process_index = 0;
  for (const ProcessInfo *info : *info.message().processes()) {
    SCOPED_TRACE(aos::FlatbufferToJson(info));
    ASSERT_EQ(0, observed_pids.count(info->pid()));
    observed_pids.insert(info->pid());
    ASSERT_TRUE(info->has_name());
    // Confirm that the top process has non-zero CPU usage, but allow the
    // lower-down processes to have not been scheduled in the last measurement
    // cycle.
    if (process_index < 1) {
      ASSERT_LT(0.0, info->cpu_usage());
    } else {
      ASSERT_LE(0.0, info->cpu_usage());
    }
    ++process_index;
    ASSERT_GE(last_cpu, info->cpu_usage());
    last_cpu = info->cpu_usage();
    ASSERT_LT(0, info->physical_memory());
  }

  for (const pid_t child : children) {
    kill(child, SIGINT);
  }
}

// Test thgat if we request arbitrarily many processes that we only get back as
// many processes as actually exist and that nothing breaks.
TEST_F(TopTest, AllTopProcesses) {
  constexpr int kNProcesses = 1000000;

  Top top(&event_loop_);
  top.set_track_top_processes(true);
  event_loop_.AddTimer([this]() { event_loop_.Exit(); })
      ->Schedule(event_loop_.monotonic_now() + std::chrono::seconds(2));
  event_loop_.Run();
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  // There should only be at most 2-3 processes visible inside the bazel
  // sandbox.
  fbb.Finish(top.TopProcesses(&fbb, kNProcesses));
  aos::FlatbufferDetachedBuffer<TopProcessesFbs> info = fbb.Release();
  ASSERT_GT(kNProcesses, info.message().processes()->size());
  double last_cpu = std::numeric_limits<double>::infinity();
  std::set<pid_t> observed_pids;
  for (const ProcessInfo *info : *info.message().processes()) {
    SCOPED_TRACE(aos::FlatbufferToJson(info));
    LOG(INFO) << aos::FlatbufferToJson(info);
    ASSERT_EQ(0, observed_pids.count(info->pid()));
    observed_pids.insert(info->pid());
    ASSERT_TRUE(info->has_name());
    ASSERT_LE(0.0, info->cpu_usage());
    ASSERT_GE(last_cpu, info->cpu_usage());
    last_cpu = info->cpu_usage();
    ASSERT_LE(0, info->physical_memory());
  }
}

}  // namespace aos::util::testing
