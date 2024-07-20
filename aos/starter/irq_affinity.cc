#include <linux/securebits.h>
#include <pwd.h>
#include <sched.h>
#include <stdint.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/starter/irq_affinity_lib.h"
#include "aos/starter/kthread_generated.h"
#include "aos/util/file.h"
#include "aos/util/top.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");

ABSL_FLAG(std::string, user, "",
          "Starter runs as though this user ran a SUID binary if set.");
ABSL_FLAG(std::string, irq_config, "rockpi_config.json",
          "File path of rockpi configuration");

namespace aos {

cpu_set_t AffinityFromFlatbuffer(const flatbuffers::Vector<uint8_t> *v) {
  cpu_set_t affinity;
  CPU_ZERO(&affinity);
  if (v == nullptr) {
    for (int i = 0; i < CPU_SETSIZE; ++i) {
      CPU_SET(i, &affinity);
    }
  } else {
    for (uint8_t cpu : *v) {
      CPU_SET(cpu, &affinity);
    }
  }
  return affinity;
}

// Class to hold the configuration for an IRQ.
struct ParsedIrqConfig {
  std::string name;
  cpu_set_t affinity;

  void ConfigureIrq(int interrupt_number) const {
    const std::string affinity_filename =
        absl::StrCat("/proc/irq/", interrupt_number, "/smp_affinity");
    const std::string contents = util::ReadFileToStringOrDie(affinity_filename);

    std::string new_contents = std::string(contents.size() - 1, '0');

    // Contents will be a padded string which is the size of the number of
    // IRQs.
    CHECK(!(CPU_SETSIZE & 0xf));
    for (size_t i = 0; i < CPU_SETSIZE; i += 4) {
      if (i / 4 >= new_contents.size()) {
        break;
      }
      uint8_t byte = 0;
      if (CPU_ISSET(i + 0, &affinity)) {
        byte |= 1;
      }
      if (CPU_ISSET(i + 1, &affinity)) {
        byte |= 2;
      }
      if (CPU_ISSET(i + 2, &affinity)) {
        byte |= 4;
      }
      if (CPU_ISSET(i + 3, &affinity)) {
        byte |= 8;
      }
      if (byte < 10) {
        new_contents[new_contents.size() - 1 - i / 4] = '0' + byte;
      } else {
        new_contents[new_contents.size() - 1 - i / 4] = 'a' + (byte - 10);
      }
    }

    if (contents != new_contents) {
      util::WriteStringToFileOrDie(affinity_filename, new_contents);
    }
  }
};

// Class to hold the configuration for a kthread.
struct ParsedKThreadConfig {
  bool full_match = false;
  std::string prefix;
  std::string postfix;
  starter::Scheduler scheduler;
  int priority;
  std::optional<int> nice;
  cpu_set_t affinity;

  bool Matches(std::string_view candidate) const {
    if (full_match) {
      return candidate == prefix;
    } else {
      if (candidate.size() < prefix.size() + postfix.size()) {
        return false;
      }
      if (candidate.substr(0, prefix.size()) != prefix) {
        return false;
      }
      if (candidate.substr(candidate.size() - postfix.size(), postfix.size()) !=
          postfix) {
        return false;
      }
      return true;
    }
  }

  void ConfigurePid(pid_t pid, std::string_view name) const {
    struct sched_param param;
    param.sched_priority = priority;
    int new_scheduler;
    switch (scheduler) {
      case starter::Scheduler::SCHEDULER_OTHER:
        new_scheduler = SCHED_OTHER;
        break;
      case starter::Scheduler::SCHEDULER_RR:
        new_scheduler = SCHED_RR;
        break;
      case starter::Scheduler::SCHEDULER_FIFO:
        new_scheduler = SCHED_FIFO;
        break;
      default:
        LOG(FATAL) << "Unknown scheduler";
    }
    PCHECK(sched_setscheduler(pid, new_scheduler, &param) == 0)
        << ", Failed to set " << name << "(" << pid << ") to "
        << (new_scheduler == SCHED_OTHER
                ? "SCHED_OTHER"
                : (new_scheduler == SCHED_RR ? "SCHED_RR" : "SCHED_FIFO"));

    if (scheduler == starter::Scheduler::SCHEDULER_OTHER && nice.has_value()) {
      PCHECK(setpriority(PRIO_PROCESS, pid, *nice) == 0)
          << ": Failed to set priority";
    }

    PCHECK(sched_setaffinity(pid, sizeof(affinity), &affinity) == 0);
  }
};

// TODO(austin): Clean this up a bit, and maybe we can add some tests.
class IrqAffinity {
 public:
  IrqAffinity(
      EventLoop *event_loop,
      const aos::FlatbufferDetachedBuffer<aos::starter::IrqAffinityConfig>
          &irq_affinity_config)
      : top_(event_loop, aos::util::Top::TrackThreadsMode::kDisabled,
             aos::util::Top::TrackPerThreadInfoMode::kDisabled) {
    if (irq_affinity_config.message().has_kthreads()) {
      PopulateThreads(irq_affinity_config.message().kthreads(), &kthreads_);
    }
    if (irq_affinity_config.message().has_threads()) {
      PopulateThreads(irq_affinity_config.message().threads(), &threads_);
    }

    if (irq_affinity_config.message().has_irqs()) {
      irqs_.reserve(irq_affinity_config.message().irqs()->size());
      for (const starter::IrqConfig *irq_config :
           *irq_affinity_config.message().irqs()) {
        CHECK(irq_config->has_name()) << ": Name required";
        LOG(INFO) << "IRQ " << aos::FlatbufferToJson(irq_config);
        irqs_.push_back(ParsedIrqConfig{
            .name = irq_config->name()->str(),
            .affinity = AffinityFromFlatbuffer(irq_config->affinity()),
        });
      }
    }

    top_.set_track_top_processes(true);
    top_.set_on_reading_update([this]() {
      for (const std::pair<const pid_t, util::Top::ProcessReadings> &reading :
           top_.readings()) {
        if (reading.second.kthread) {
          for (const ParsedKThreadConfig &match : kthreads_) {
            if (match.Matches(reading.second.name)) {
              match.ConfigurePid(reading.first, reading.second.name);
              break;
            }
          }
        } else {
          for (const ParsedKThreadConfig &match : threads_) {
            if (match.Matches(reading.second.name)) {
              match.ConfigurePid(reading.first, reading.second.name);
              break;
            }
          }
        }
      }

      interrupts_status_.Update();

      for (const InterruptsStatus::InterruptState &state :
           interrupts_status_.states()) {
        for (const ParsedIrqConfig &match : irqs_) {
          bool matched = false;
          for (const std::string &action : state.actions) {
            if (match.name == action) {
              matched = true;
              break;
            }
          }
          if (matched) {
            match.ConfigureIrq(state.interrupt_number);
          }
        }
      }
    });
  }

 private:
  void PopulateThreads(
      const flatbuffers::Vector<flatbuffers::Offset<starter::KthreadConfig>>
          *threads_config,
      std::vector<ParsedKThreadConfig> *threads) {
    threads->reserve(threads_config->size());
    for (const starter::KthreadConfig *kthread_config : *threads_config) {
      LOG(INFO) << "Kthread " << aos::FlatbufferToJson(kthread_config);
      CHECK(kthread_config->has_name()) << ": Name required";
      const size_t star_position =
          kthread_config->name()->string_view().find('*');
      const bool has_star = star_position != std::string_view::npos;

      threads->push_back(ParsedKThreadConfig{
          .full_match = !has_star,
          .prefix = std::string(
              !has_star ? kthread_config->name()->string_view()
                        : kthread_config->name()->string_view().substr(
                              0, star_position)),
          .postfix = std::string(
              !has_star ? ""
                        : kthread_config->name()->string_view().substr(
                              star_position + 1)),
          .scheduler = kthread_config->scheduler(),
          .priority = kthread_config->priority(),
          .nice = kthread_config->nice(),
          .affinity = AffinityFromFlatbuffer(kthread_config->affinity()),
      });
    }
  }

  util::Top top_;

  // TODO(austin): Publish message with everything in it.
  // TODO(austin): Make Top report out affinity + priority + scheduler for
  // posterity.

  std::vector<ParsedKThreadConfig> kthreads_;
  std::vector<ParsedKThreadConfig> threads_;
  std::vector<ParsedIrqConfig> irqs_;

  InterruptsStatus interrupts_status_;
};

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  if (!absl::GetFlag(FLAGS_user).empty()) {
    // Maintain root permissions as we switch to become the user so we can
    // actually manipulate priorities.
    PCHECK(prctl(PR_SET_SECUREBITS, SECBIT_NO_SETUID_FIXUP | SECBIT_NOROOT) ==
           0);

    uid_t uid;
    uid_t gid;
    {
      struct passwd *user_data = getpwnam(absl::GetFlag(FLAGS_user).c_str());
      if (user_data != nullptr) {
        uid = user_data->pw_uid;
        gid = user_data->pw_gid;
      } else {
        LOG(FATAL) << "Could not find user " << absl::GetFlag(FLAGS_user);
        return 1;
      }
    }
    // Change the real and effective IDs to the user we're running as. The
    // effective IDs mean files we access (like shared memory) will happen as
    // that user. The real IDs allow child processes with an different effective
    // ID to still participate in signal sending/receiving.
    constexpr int kUnchanged = -1;
    if (setresgid(/* ruid */ gid, /* euid */ gid,
                  /* suid */ kUnchanged) != 0) {
      PLOG(FATAL) << "Failed to change GID to " << absl::GetFlag(FLAGS_user);
    }

    if (setresuid(/* ruid */ uid, /* euid */ uid,
                  /* suid */ kUnchanged) != 0) {
      PLOG(FATAL) << "Failed to change UID to " << absl::GetFlag(FLAGS_user);
    }
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::FlatbufferDetachedBuffer<aos::starter::IrqAffinityConfig>
      irq_affinity_config =
          aos::JsonFileToFlatbuffer<aos::starter::IrqAffinityConfig>(
              absl::GetFlag(FLAGS_irq_config));

  aos::ShmEventLoop shm_event_loop(&config.message());

  aos::IrqAffinity irq_affinity(&shm_event_loop, irq_affinity_config);

  shm_event_loop.Run();

  return 0;
}
