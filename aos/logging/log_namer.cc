#include "aos/logging/log_namer.h"

#include <dirent.h>
#include <fcntl.h>
#include <mntent.h>
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include "aos/configuration.h"
#include "glog/logging.h"

DEFINE_string(logging_folder,
#ifdef AOS_ARCHITECTURE_arm_frc
              "",
#else
              "./logs",
#endif
              "The folder to log to.  If empty, search for the /media/sd*1/ "
              "folder and place logs there.");

namespace aos {
namespace logging {
namespace {
void AllocateLogName(char **filename, const char *directory,
                     const char *basename) {
  int fileindex = 0;
  DIR *const d = opendir(directory);
  if (d == nullptr) {
    PLOG(FATAL) << "could not open directory" << directory;
  }
  int index = 0;
  while (true) {
    errno = 0;
    struct dirent *const dir = readdir(d);
    if (dir == nullptr) {
      if (errno == 0) {
        break;
      } else {
        PLOG(FATAL) << "readdir(" << d << ") failed";
      }
    } else {
      const std::string format_string = std::string(basename) + "-%d";
      if (sscanf(dir->d_name, format_string.c_str(), &index) == 1) {
        if (index >= fileindex) {
          fileindex = index + 1;
        }
      }
    }
  }
  closedir(d);

  char previous[512];
  ::std::string path = ::std::string(directory) + "/" + basename + "-current";
  ssize_t len = ::readlink(path.c_str(), previous, sizeof(previous));
  if (len != -1) {
    previous[len] = '\0';
  } else {
    previous[0] = '\0';
    LOG(INFO) << "Could not find " << path;
  }
  if (asprintf(filename, "%s/%s-%03d", directory, basename, fileindex) == -1) {
    PLOG(FATAL) << "couldn't create final name";
  }
  // Fix basename formatting.
  LOG(INFO) << "Created log file (" << filename << "). Previous file was ("
            << directory << "/" << previous << ").";
}

bool FoundThumbDrive(const char *path) {
  FILE *mnt_fp = setmntent("/etc/mtab", "r");
  if (mnt_fp == nullptr) {
    LOG(FATAL) << "Could not open /etc/mtab";
  }

  bool found = false;
  struct mntent mntbuf;
  char buf[256];
  while (!found) {
    struct mntent *mount_list = getmntent_r(mnt_fp, &mntbuf, buf, sizeof(buf));
    if (mount_list == nullptr) {
      break;
    }
    if (strcmp(mount_list->mnt_dir, path) == 0) {
      found = true;
    }
  }
  endmntent(mnt_fp);
  return found;
}

bool FindDevice(char *device, size_t device_size) {
  char test_device[10];
  for (char i = 'a'; i < 'z'; ++i) {
    snprintf(test_device, sizeof(test_device), "/dev/sd%c", i);
    VLOG(1) << "Trying to access" << test_device;
    if (access(test_device, F_OK) != -1) {
      snprintf(device, device_size, "sd%c", i);
      return true;
    }
  }
  return false;
}

}  // namespace

std::optional<std::string> MaybeGetLogName(const char *basename) {
  if (FLAGS_logging_folder.empty()) {
    char folder[128];
    {
      char dev_name[8];
      if (!FindDevice(dev_name, sizeof(dev_name))) {
        LOG(INFO) << "Waiting for a device";
        return std::nullopt;
      }
      snprintf(folder, sizeof(folder), "/media/%s1", dev_name);
      if (!FoundThumbDrive(folder)) {
        LOG(INFO) << "Waiting for" << folder;
        return std::nullopt;
      }
      snprintf(folder, sizeof(folder), "/media/%s1/", dev_name);
    }

    if (access(folder, F_OK) == -1) {
      LOG(FATAL) << "folder '" << folder
                 << "' does not exist. please create it.";
    }

    FLAGS_logging_folder = folder;
  }
  const char *folder = FLAGS_logging_folder.c_str();
  if (access(folder, R_OK | W_OK) == -1) {
    LOG(FATAL) << "folder '" << folder << "' does not exist. please create it.";
  }
  LOG(INFO) << "logging to folder '" << folder << "'";

  char *tmp;
  AllocateLogName(&tmp, folder, basename);

  std::string log_base_name = tmp;
  std::string log_roborio_name = log_base_name + "/";
  free(tmp);

  char *tmp2;
  if (asprintf(&tmp2, "%s/%s-current", folder, basename) == -1) {
    PLOG(WARNING) << "couldn't create current symlink name";
  } else {
    if (unlink(tmp2) == -1 && (errno != EROFS && errno != ENOENT)) {
      LOG(WARNING) << "unlink('" << tmp2 << "') failed";
    }
    if (symlink(log_roborio_name.c_str(), tmp2) == -1) {
      PLOG(WARNING) << "symlink('" << log_roborio_name.c_str() << "', '" << tmp2
                    << "') failed";
    }
    free(tmp2);
  }
  return log_base_name;
}

std::string GetLogName(const char *basename) {
  std::optional<std::string> log_base_name;

  while (true) {
    log_base_name = MaybeGetLogName(basename);

    if (log_base_name.has_value()) {
      break;
    }

    sleep(5);
  }

  return log_base_name.value();
}

}  // namespace logging
}  // namespace aos
