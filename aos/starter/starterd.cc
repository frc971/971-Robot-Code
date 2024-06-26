#include <pwd.h>
#include <unistd.h>

#include <ostream>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/starter/starterd_lib.h"
#include "aos/util/file.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, user, "",
          "Starter runs as though this user ran a SUID binary if set.");
ABSL_FLAG(std::string, version_string, "",
          "Version to report for starterd and subprocesses.");

ABSL_DECLARE_FLAG(std::string, shm_base);
ABSL_FLAG(bool, purge_shm_base, false,
          "If true, delete everything in --shm_base before starting.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  if (absl::GetFlag(FLAGS_purge_shm_base)) {
    aos::util::UnlinkRecursive(absl::GetFlag(FLAGS_shm_base));
  }

  if (!absl::GetFlag(FLAGS_user).empty()) {
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
      PLOG(FATAL) << "Failed to change GID to " << absl::GetFlag(FLAGS_user)
                  << ", group " << gid;
    }

    if (setresuid(/* ruid */ uid, /* euid */ uid,
                  /* suid */ kUnchanged) != 0) {
      PLOG(FATAL) << "Failed to change UID to " << absl::GetFlag(FLAGS_user);
    }
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  const aos::Configuration *config_msg = &config.message();

  aos::starter::Starter starter(config_msg);
  if (!absl::GetFlag(FLAGS_version_string).empty()) {
    starter.event_loop()->SetVersionString(absl::GetFlag(FLAGS_version_string));
  }

  starter.Run();

  return 0;
}
