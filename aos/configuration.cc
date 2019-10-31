#include "aos/configuration.h"

#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>

#include "absl/container/btree_set.h"
#include "absl/strings/string_view.h"
#include "aos/configuration_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/unique_malloc_ptr.h"
#include "aos/util/file.h"
#include "glog/logging.h"
#include "absl/base/call_once.h"

namespace aos {

// Define the compare and equal operators for Channel and Application so we can
// insert them in the btree below.
//
// These are not in headers because they are only comparing part of the
// flatbuffer, and it seems weird to expose that as *the* compare operator.  And
// I can't put them in an anonymous namespace because they wouldn't be found
// that way by the btree.
bool operator<(const FlatbufferDetachedBuffer<Channel> &lhs,
               const FlatbufferDetachedBuffer<Channel> &rhs) {
  int name_compare = lhs.message().name()->string_view().compare(
      rhs.message().name()->string_view());
  if (name_compare == 0) {
    return lhs.message().type()->string_view() <
           rhs.message().type()->string_view();
  } else if (name_compare < 0) {
    return true;
  } else {
    return false;
  }
}

bool operator==(const FlatbufferDetachedBuffer<Channel> &lhs,
                const FlatbufferDetachedBuffer<Channel> &rhs) {
  return lhs.message().name()->string_view() ==
             rhs.message().name()->string_view() &&
         lhs.message().type()->string_view() ==
             rhs.message().type()->string_view();
}

bool operator==(const FlatbufferDetachedBuffer<Application> &lhs,
                const FlatbufferDetachedBuffer<Application> &rhs) {
  return lhs.message().name()->string_view() ==
         rhs.message().name()->string_view();
}

bool operator<(const FlatbufferDetachedBuffer<Application> &lhs,
               const FlatbufferDetachedBuffer<Application> &rhs) {
  return lhs.message().name()->string_view() <
         rhs.message().name()->string_view();
}

namespace configuration {
namespace {

// TODO(brians): This shouldn't be necesary for running tests.  Provide a way to
// set the IP address when running tests from the test.
const char *const kLinuxNetInterface = "eth0";

void DoGetOwnIPAddress(in_addr *retu) {
  static const char *kOverrideVariable = "FRC971_IP_OVERRIDE";
  const char *override_ip = getenv(kOverrideVariable);
  if (override_ip != NULL) {
    LOG(INFO) << "Override IP is " << override_ip;
    if (inet_aton(override_ip, retu) != 0) {
      return;
    } else {
      LOG(WARNING) << "error parsing " << kOverrideVariable << " value '"
                   << override_ip << "'";
    }
  } else {
    LOG(INFO) << "Couldn't get environmental variable.";
  }

  ifaddrs *addrs;
  if (getifaddrs(&addrs) != 0) {
    PLOG(FATAL) << "getifaddrs(" << &addrs << ") failed";
  }
  // Smart pointers don't work very well for iterating through a linked list,
  // but it does do a very nice job of making sure that addrs gets freed.
  unique_c_ptr<ifaddrs, freeifaddrs> addrs_deleter(addrs);

  for (; addrs != nullptr; addrs = addrs->ifa_next) {
    // ifa_addr tends to be nullptr on CAN interfaces.
    if (addrs->ifa_addr != nullptr && addrs->ifa_addr->sa_family == AF_INET) {
      if (strcmp(kLinuxNetInterface, addrs->ifa_name) == 0) {
        *retu = reinterpret_cast<sockaddr_in *>(__builtin_assume_aligned(
                addrs->ifa_addr, alignof(sockaddr_in)))->sin_addr;
        return;
      }
    }
  }
  LOG(FATAL) << "couldn't find an AF_INET interface named \""
             << kLinuxNetInterface << "\"";
}

void DoGetRootDirectory(char** retu) {
  ssize_t size = 0;
  *retu = NULL;
  while (true) {
    size += 256;
    if (*retu != nullptr) delete *retu;
    *retu = new char[size];

    ssize_t ret = readlink("/proc/self/exe", *retu, size);
    if (ret < 0) {
      if (ret != -1) {
        LOG(WARNING) << "it returned " << ret << ", not -1";
      }

      PLOG(FATAL) << "readlink(\"/proc/self/exe\", " << *retu << ", " << size
                  << ") failed";
    }
    if (ret < size) {
      void *last_slash = memrchr(*retu, '/', ret);
      if (last_slash == NULL) {
        *retu[ret] = '\0';
        LOG(FATAL) << "couldn't find a '/' in \"" << *retu << "\"";
      }
      *static_cast<char *>(last_slash) = '\0';
      LOG(INFO) << "got a root dir of \"" << *retu << "\"";
      return;
    }
  }
}

void DoGetLoggingDirectory(char** retu) {
  static const char kSuffix[] = "/../../tmp/robot_logs";
  const char *root = GetRootDirectory();
  *retu = new char[strlen(root) + sizeof(kSuffix)];
  strcpy(*retu, root);
  strcat(*retu, kSuffix);
}

// Extracts the folder part of a path.  Returns ./ if there is no path.
absl::string_view ExtractFolder(const absl::string_view filename) {
  auto last_slash_pos = filename.find_last_of("/\\");

  return last_slash_pos == absl::string_view::npos
             ? absl::string_view("./")
             : filename.substr(0, last_slash_pos + 1);
}

FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const absl::string_view path, absl::btree_set<std::string> *visited_paths) {
  FlatbufferDetachedBuffer<Configuration> config(JsonToFlatbuffer(
      util::ReadFileToStringOrDie(path), ConfigurationTypeTable()));
  // Depth first.  Take the following example:
  //
  // config1.json:
  // {
  //   "channels": [
  //     {
  //       "name": "/foo",
  //       "type": ".aos.bar",
  //       "max_size": 5
  //     }
  //   ],
  //   "imports": [
  //     "config2.json",
  //   ]
  // }
  //
  // config2.json:
  // {
  //   "channels": [
  //     {
  //       "name": "/foo",
  //       "type": ".aos.bar",
  //       "max_size": 7
  //     }
  //   ],
  // }
  //
  // We want the main config (config1.json) to be able to override the imported
  // config.  That means that it needs to be merged into the imported configs,
  // not the other way around.

  // Track that we have seen this file before recursing.
  visited_paths->insert(::std::string(path));

  if (config.message().has_imports()) {
    // Capture the imports.
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> *v =
        config.message().imports();

    // And then wipe them.  This gets GCed when we merge later.
    config.mutable_message()->clear_imports();

    // Start with an empty configuration to merge into.
    FlatbufferDetachedBuffer<Configuration> merged_config =
        FlatbufferDetachedBuffer<Configuration>::Empty();

    const ::std::string folder(ExtractFolder(path));

    for (const flatbuffers::String *str : *v) {
      const ::std::string included_config = folder + str->c_str();
      // Abort on any paths we have already seen.
      CHECK(visited_paths->find(included_config) == visited_paths->end())
          << ": Found duplicate file " << included_config << " while reading "
          << path;

      // And them merge everything in.
      merged_config = MergeFlatBuffers(
          merged_config, ReadConfig(included_config, visited_paths));
    }

    // Finally, merge this file in.
    config = MergeFlatBuffers(merged_config, config);
  }
  return config;
}

// Remove duplicate entries, and handle overrides.
FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config) {
  // Store all the channels in a sorted set.  This lets us track channels we
  // have seen before and merge the updates in.
  absl::btree_set<FlatbufferDetachedBuffer<Channel>> channels;

  if (config.message().has_channels()) {
    for (const Channel *c : *config.message().channels()) {
      // Ignore malformed entries.
      if (!c->has_name()) {
        continue;
      }
      if (!c->has_type()) {
        continue;
      }

      // Attempt to insert the channel.
      auto result = channels.insert(CopyFlatBuffer(c));
      if (!result.second) {
        // Already there, so merge the new table into the original.
        *result.first = MergeFlatBuffers(*result.first, CopyFlatBuffer(c));
      }
    }
  }

  // Now repeat this for the application list.
  absl::btree_set<FlatbufferDetachedBuffer<Application>> applications;
  if (config.message().has_applications()) {
    for (const Application *a : *config.message().applications()) {
      if (!a->has_name()) {
        continue;
      }

      auto result = applications.insert(CopyFlatBuffer(a));
      if (!result.second) {
        *result.first = MergeFlatBuffers(*result.first, CopyFlatBuffer(a));
      }
    }
  }

  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);

  // Start by building the vectors.  They need to come before the final table.
  // Channels
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset;
  {
    ::std::vector<flatbuffers::Offset<Channel>> channel_offsets;
    for (const FlatbufferDetachedBuffer<Channel> &c : channels) {
      channel_offsets.emplace_back(
          CopyFlatBuffer<Channel>(&c.message(), &fbb));
    }
    channels_offset = fbb.CreateVector(channel_offsets);
  }

  // Applications
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
      applications_offset;
  {
    ::std::vector<flatbuffers::Offset<Application>> applications_offsets;
    for (const FlatbufferDetachedBuffer<Application> &a : applications) {
      applications_offsets.emplace_back(
          CopyFlatBuffer<Application>(&a.message(), &fbb));
    }
    applications_offset = fbb.CreateVector(applications_offsets);
  }

  // Just copy the maps
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Map>>>
      maps_offset;
  {
    ::std::vector<flatbuffers::Offset<Map>> map_offsets;
    if (config.message().has_maps()) {
      for (const Map *m : *config.message().maps()) {
        map_offsets.emplace_back(CopyFlatBuffer<Map>(m, &fbb));
      }
      maps_offset = fbb.CreateVector(map_offsets);
    }
  }

  // And then build a Configuration with them all.
  ConfigurationBuilder configuration_builder(fbb);
  configuration_builder.add_channels(channels_offset);
  if (config.message().has_maps()) {
    configuration_builder.add_maps(maps_offset);
  }
  configuration_builder.add_applications(applications_offset);

  fbb.Finish(configuration_builder.Finish());
  return fbb.Release();
}

// Compares (c < p) a channel, and a name, type tuple.
bool CompareChannels(const Channel *c,
                     ::std::pair<absl::string_view, absl::string_view> p) {
  int name_compare = c->name()->string_view().compare(p.first);
  if (name_compare == 0) {
    return c->type()->string_view() < p.second;
  } else if (name_compare < 0) {
    return true;
  } else {
    return false;
  }
};

// Compares for equality (c == p) a channel, and a name, type tuple.
bool EqualsChannels(const Channel *c,
                     ::std::pair<absl::string_view, absl::string_view> p) {
  return c->name()->string_view() == p.first &&
         c->type()->string_view() == p.second;
}

// Compares (c < p) an application, and a name;
bool CompareApplications(const Application *a, absl::string_view name) {
  return a->name()->string_view() < name;
};

// Compares for equality (c == p) an application, and a name;
bool EqualsApplications(const Application *a, absl::string_view name) {
  return a->name()->string_view() == name;
}

// Maps name for the provided maps.  Modifies name.
void HandleMaps(const flatbuffers::Vector<flatbuffers::Offset<aos::Map>> *maps,
                absl::string_view *name) {
  // For the same reason we merge configs in reverse order, we want to process
  // maps in reverse order.  That lets the outer config overwrite channels from
  // the inner configs.
  for (auto i = maps->rbegin(); i != maps->rend(); ++i) {
    if (i->has_match() && i->match()->has_name() && i->has_rename() &&
        i->rename()->has_name() && i->match()->name()->string_view() == *name) {
      VLOG(1) << "Renamed \"" << *name << "\" to \""
              << i->rename()->name()->string_view() << "\"";
      *name = i->rename()->name()->string_view();
    }
  }
}

}  // namespace

const char *GetRootDirectory() {
  static  char* root_dir;// return value
  static absl::once_flag once_;
  absl::call_once(once_, DoGetRootDirectory, &root_dir);
  return root_dir;
}

const char *GetLoggingDirectory() {
  static char* retu;// return value
  static absl::once_flag once_;
  absl::call_once(once_, DoGetLoggingDirectory, &retu);
  return retu;
}

const in_addr &GetOwnIPAddress() {
  static in_addr retu;// return value
  static absl::once_flag once_;
  absl::call_once(once_, DoGetOwnIPAddress, &retu);
  return retu;
}

FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const absl::string_view path) {
  // We only want to read a file once.  So track the visited files in a set.
  absl::btree_set<std::string> visited_paths;
  return MergeConfiguration(ReadConfig(path, &visited_paths));
}

const Channel *GetChannel(const Configuration *config, absl::string_view name,
                          absl::string_view type,
                          absl::string_view application_name) {
  VLOG(1) << "Looking up { \"name\": \"" << name << "\", \"type\": \"" << type
          << "\" }";

  // First handle application specific maps.  Only do this if we have a matching
  // application name, and it has maps.
  if (config->has_applications()) {
    auto application_iterator = std::lower_bound(
        config->applications()->cbegin(), config->applications()->cend(),
        application_name, CompareApplications);
    if (application_iterator != config->applications()->cend() &&
        EqualsApplications(*application_iterator, application_name)) {
      if (application_iterator->has_maps()) {
        HandleMaps(application_iterator->maps(), &name);
      }
    }
  }

  // Now do global maps.
  if (config->has_maps()) {
    HandleMaps(config->maps(), &name);
  }

  // Then look for the channel.
  auto channel_iterator =
      std::lower_bound(config->channels()->cbegin(),
                       config->channels()->cend(),
                       std::make_pair(name, type), CompareChannels);

  // Make sure we actually found it, and it matches.
  if (channel_iterator != config->channels()->cend() &&
      EqualsChannels(*channel_iterator, std::make_pair(name, type))) {
    VLOG(1) << "Found: " << FlatbufferToJson(*channel_iterator);
    return *channel_iterator;
  } else {
    VLOG(1) << "No match for { \"name\": \"" << name << "\", \"type\": \""
            << type << "\" }";
    return nullptr;
  }
}

}  // namespace configuration
}  // namespace aos
