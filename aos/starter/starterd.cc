#include <pwd.h>
#include <sys/types.h>

#include "aos/init.h"
#include "gflags/gflags.h"
#include "starterd_lib.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");
DEFINE_string(user, "",
              "Starter runs as though this user ran a SUID binary if set.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  if (!FLAGS_user.empty()) {
    uid_t uid;
    uid_t gid;
    {
      struct passwd *user_data = getpwnam(FLAGS_user.c_str());
      if (user_data != nullptr) {
        uid = user_data->pw_uid;
        gid = user_data->pw_gid;
      } else {
        LOG(FATAL) << "Could not find user " << FLAGS_user;
        return 1;
      }
    }
    constexpr int kUnchanged = -1;
    if (setresgid(/* ruid */ gid, /* euid */ gid,
                  /* suid */ kUnchanged) != 0) {
      PLOG(FATAL) << "Failed to change GID to " << FLAGS_user;
    }

    if (setresuid(/* ruid */ uid, /* euid */ uid,
                  /* suid */ kUnchanged) != 0) {
      PLOG(FATAL) << "Failed to change UID to " << FLAGS_user;
    }
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  const aos::Configuration *config_msg = &config.message();

  aos::starter::Starter starter(config_msg);

  starter.Run();

  return 0;
}
