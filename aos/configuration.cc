#include "aos/configuration.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <map>
#include <set>
#include <string_view>

#include "absl/container/btree_set.h"
#include "absl/strings/str_cat.h"
#include "aos/configuration_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/unique_malloc_ptr.h"
#include "aos/util/file.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace aos {
namespace {
bool EndsWith(std::string_view str, std::string_view end) {
  if (str.size() < end.size()) {
    return false;
  }
  if (str.substr(str.size() - end.size(), end.size()) != end) {
    return false;
  }
  return true;
}

std::string MaybeReplaceExtension(std::string_view filename,
                                  std::string_view extension,
                                  std::string_view replacement) {
  if (!EndsWith(filename, extension)) {
    return std::string(filename);
  }
  filename.remove_suffix(extension.size());
  return absl::StrCat(filename, replacement);
}

FlatbufferDetachedBuffer<Configuration> ReadConfigFile(std::string_view path,
                                                       bool binary) {
  if (binary) {
    FlatbufferVector<Configuration> config =
        FileToFlatbuffer<Configuration>(path);
    return CopySpanAsDetachedBuffer(config.span());
  }

  flatbuffers::DetachedBuffer buffer = JsonToFlatbuffer(
      util::ReadFileToStringOrDie(path), ConfigurationTypeTable());

  CHECK_GT(buffer.size(), 0u) << ": Failed to parse JSON file";

  return FlatbufferDetachedBuffer<Configuration>(std::move(buffer));
}

}  // namespace

// Define the compare and equal operators for Channel and Application so we can
// insert them in the btree below.
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

bool operator==(const FlatbufferDetachedBuffer<Node> &lhs,
                const FlatbufferDetachedBuffer<Node> &rhs) {
  return lhs.message().name()->string_view() ==
         rhs.message().name()->string_view();
}

bool operator<(const FlatbufferDetachedBuffer<Node> &lhs,
               const FlatbufferDetachedBuffer<Node> &rhs) {
  return lhs.message().name()->string_view() <
         rhs.message().name()->string_view();
}

namespace configuration {
namespace {

// Extracts the folder part of a path.  Returns ./ if there is no path.
std::string_view ExtractFolder(const std::string_view filename) {
  auto last_slash_pos = filename.find_last_of("/\\");

  return last_slash_pos == std::string_view::npos
             ? std::string_view("./")
             : filename.substr(0, last_slash_pos + 1);
}

std::string AbsolutePath(const std::string_view filename) {
  // Uses an std::string so that we know the input will be null-terminated.
  const std::string terminated_file(filename);
  char buffer[PATH_MAX];
  PCHECK(NULL != realpath(terminated_file.c_str(), buffer));
  return buffer;
}

FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path, absl::btree_set<std::string> *visited_paths,
    const std::vector<std::string_view> &extra_import_paths) {
  std::string binary_path = MaybeReplaceExtension(path, ".json", ".bfbs");
  bool binary_path_exists = util::PathExists(binary_path);
  std::string raw_path(path);
  // For each .json file, look and see if we can find a .bfbs file next to it
  // with the same base name.  If we can, assume it is the same and use it
  // instead.  It is much faster to load .bfbs files than .json files.
  if (!binary_path_exists && !util::PathExists(raw_path)) {
    const bool path_is_absolute = raw_path.size() > 0 && raw_path[0] == '/';
    if (path_is_absolute) {
      CHECK(extra_import_paths.empty())
          << "Can't specify extra import paths if attempting to read a config "
             "file from an absolute path (path is "
          << raw_path << ").";
    }

    bool found_path = false;
    for (const auto &import_path : extra_import_paths) {
      raw_path = std::string(import_path) + "/" + std::string(path);
      binary_path = MaybeReplaceExtension(raw_path, ".json", ".bfbs");
      binary_path_exists = util::PathExists(binary_path);
      if (binary_path_exists) {
        found_path = true;
        break;
      }
      if (util::PathExists(raw_path)) {
        found_path = true;
        break;
      }
    }
    CHECK(found_path) << ": Failed to find file " << path << ".";
  }

  FlatbufferDetachedBuffer<Configuration> config = ReadConfigFile(
      binary_path_exists ? binary_path : raw_path, binary_path_exists);

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

  const std::string absolute_path =
      AbsolutePath(binary_path_exists ? binary_path : raw_path);
  // Track that we have seen this file before recursing.  Track the path we
  // actually loaded (which should be consistent if imported twice).
  if (!visited_paths->insert(absolute_path).second) {
    for (const auto &visited_path : *visited_paths) {
      LOG(INFO) << "Already visited: " << visited_path;
    }
    LOG(FATAL)
        << "Already imported " << path << " (i.e. " << absolute_path
        << "). See above for the files that have already been processed.";
  }

  if (config.message().has_imports()) {
    // Capture the imports.
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> *v =
        config.message().imports();

    // And then wipe them.  This gets GCed when we merge later.
    config.mutable_message()->clear_imports();

    // Start with an empty configuration to merge into.
    FlatbufferDetachedBuffer<Configuration> merged_config =
        FlatbufferDetachedBuffer<Configuration>::Empty();

    const std::string path_folder(ExtractFolder(path));
    for (const flatbuffers::String *str : *v) {
      const std::string included_config =
          path_folder + "/" + std::string(str->string_view());

      // And them merge everything in.
      merged_config = MergeFlatBuffers(
          merged_config,
          ReadConfig(included_config, visited_paths, extra_import_paths));
    }

    // Finally, merge this file in.
    config = MergeFlatBuffers(merged_config, config);
  }
  return config;
}

// Compares (c < p) a channel, and a name, type tuple.
bool CompareChannels(const Channel *c,
                     ::std::pair<std::string_view, std::string_view> p) {
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
                    ::std::pair<std::string_view, std::string_view> p) {
  return c->name()->string_view() == p.first &&
         c->type()->string_view() == p.second;
}

// Compares (c < p) an application, and a name;
bool CompareApplications(const Application *a, std::string_view name) {
  return a->name()->string_view() < name;
};

// Compares for equality (c == p) an application, and a name;
bool EqualsApplications(const Application *a, std::string_view name) {
  return a->name()->string_view() == name;
}

void ValidateConfiguration(const Flatbuffer<Configuration> &config) {
  // No imports should be left.
  CHECK(!config.message().has_imports());

  // Check that if there is a node list, all the source nodes are filled out and
  // valid, and all the destination nodes are valid (and not the source).  This
  // is a basic consistency check.
  if (config.message().has_channels()) {
    const Channel *last_channel = nullptr;
    for (const Channel *c : *config.message().channels()) {
      CHECK(c->has_name());
      CHECK(c->has_type());
      if (c->name()->string_view().back() == '/') {
        LOG(FATAL) << "Channel names can't end with '/'";
      }
      if (c->name()->string_view().find("//") != std::string_view::npos) {
        LOG(FATAL) << ": Invalid channel name " << c->name()->string_view()
                   << ", can't use //.";
      }
      for (const char data : c->name()->string_view()) {
        if (data >= '0' && data <= '9') {
          continue;
        }
        if (data >= 'a' && data <= 'z') {
          continue;
        }
        if (data >= 'A' && data <= 'Z') {
          continue;
        }
        if (data == '-' || data == '_' || data == '/') {
          continue;
        }
        LOG(FATAL) << "Invalid channel name " << c->name()->string_view()
                   << ", can only use [-a-zA-Z0-9_/]";
      }

      // Make sure everything is sorted while we are here...  If this fails,
      // there will be a bunch of weird errors.
      if (last_channel != nullptr) {
        CHECK(CompareChannels(
            last_channel,
            std::make_pair(c->name()->string_view(), c->type()->string_view())))
            << ": Channels not sorted!";
      }
      last_channel = c;
    }
  }

  if (config.message().has_nodes() && config.message().has_channels()) {
    for (const Channel *c : *config.message().channels()) {
      CHECK(c->has_source_node()) << ": Channel " << FlatbufferToJson(c)
                                  << " is missing \"source_node\"";
      CHECK(GetNode(&config.message(), c->source_node()->string_view()) !=
            nullptr)
          << ": Channel " << FlatbufferToJson(c)
          << " has an unknown \"source_node\"";

      if (c->has_destination_nodes()) {
        for (const Connection *connection : *c->destination_nodes()) {
          CHECK(connection->has_name());
          CHECK(GetNode(&config.message(), connection->name()->string_view()) !=
                nullptr)
              << ": Channel " << FlatbufferToJson(c)
              << " has an unknown \"destination_nodes\" "
              << connection->name()->string_view();

          switch (connection->timestamp_logger()) {
            case LoggerConfig::LOCAL_LOGGER:
            case LoggerConfig::NOT_LOGGED:
              CHECK(!connection->has_timestamp_logger_nodes());
              break;
            case LoggerConfig::REMOTE_LOGGER:
            case LoggerConfig::LOCAL_AND_REMOTE_LOGGER:
              CHECK(connection->has_timestamp_logger_nodes());
              CHECK_GT(connection->timestamp_logger_nodes()->size(), 0u);
              for (const flatbuffers::String *timestamp_logger_node :
                   *connection->timestamp_logger_nodes()) {
                CHECK(GetNode(&config.message(),
                              timestamp_logger_node->string_view()) != nullptr)
                    << ": Channel " << FlatbufferToJson(c)
                    << " has an unknown \"timestamp_logger_node\""
                    << connection->name()->string_view();
              }
              break;
          }

          CHECK_NE(connection->name()->string_view(),
                   c->source_node()->string_view())
              << ": Channel " << FlatbufferToJson(c)
              << " is forwarding data to itself";
        }
      }
    }
  }
}

}  // namespace

// Maps name for the provided maps.  Modifies name.
void HandleMaps(const flatbuffers::Vector<flatbuffers::Offset<aos::Map>> *maps,
                std::string *name, std::string_view type, const Node *node) {
  // For the same reason we merge configs in reverse order, we want to process
  // maps in reverse order.  That lets the outer config overwrite channels from
  // the inner configs.
  for (auto i = maps->rbegin(); i != maps->rend(); ++i) {
    if (!i->has_match() || !i->match()->has_name()) {
      continue;
    }
    if (!i->has_rename() || !i->rename()->has_name()) {
      continue;
    }

    // Handle normal maps (now that we know that match and rename are filled
    // out).
    const std::string_view match_name = i->match()->name()->string_view();
    if (match_name != *name) {
      if (match_name.back() == '*' &&
          std::string_view(*name).substr(
              0, std::min(name->size(), match_name.size() - 1)) ==
              match_name.substr(0, match_name.size() - 1)) {
        CHECK_EQ(match_name.find('*'), match_name.size() - 1);
      } else {
        continue;
      }
    }

    // Handle type specific maps.
    if (i->match()->has_type() && i->match()->type()->string_view() != type) {
      continue;
    }

    // Now handle node specific maps.
    if (node != nullptr && i->match()->has_source_node() &&
        i->match()->source_node()->string_view() !=
            node->name()->string_view()) {
      continue;
    }

    std::string new_name(i->rename()->name()->string_view());
    if (match_name.back() == '*') {
      new_name += std::string(name->substr(match_name.size() - 1));
    }
    VLOG(1) << "Renamed \"" << *name << "\" to \"" << new_name << "\"";
    *name = std::move(new_name);
  }
}

FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config) {
  // auto_merge_config will contain all the fields of the Configuration that are
  // to be passed through unmodified to the result of MergeConfiguration().
  // In the processing below, we mutate auto_merge_config to remove any fields
  // which we do need to alter (hence why we can't use the input config
  // directly), and then merge auto_merge_config back in at the end.
  aos::FlatbufferDetachedBuffer<aos::Configuration> auto_merge_config =
      aos::RecursiveCopyFlatBuffer(&config.message());

  // Store all the channels in a sorted set.  This lets us track channels we
  // have seen before and merge the updates in.
  absl::btree_set<FlatbufferDetachedBuffer<Channel>> channels;

  if (config.message().has_channels()) {
    auto_merge_config.mutable_message()->clear_channels();
    for (const Channel *c : *config.message().channels()) {
      // Ignore malformed entries.
      if (!c->has_name()) {
        continue;
      }
      if (!c->has_type()) {
        continue;
      }

      CHECK_EQ(c->read_method() == ReadMethod::PIN, c->num_readers() != 0)
          << ": num_readers may be set if and only if read_method is PIN,"
             " if you want 0 readers do not set PIN: "
          << CleanedChannelToString(c);

      // Attempt to insert the channel.
      auto result = channels.insert(RecursiveCopyFlatBuffer(c));
      if (!result.second) {
        // Already there, so merge the new table into the original.
        *result.first =
            MergeFlatBuffers(*result.first, RecursiveCopyFlatBuffer(c));
      }
    }
  }

  // Now repeat this for the application list.
  absl::btree_set<FlatbufferDetachedBuffer<Application>> applications;
  if (config.message().has_applications()) {
    auto_merge_config.mutable_message()->clear_applications();
    for (const Application *a : *config.message().applications()) {
      if (!a->has_name()) {
        continue;
      }

      auto result = applications.insert(RecursiveCopyFlatBuffer(a));
      if (!result.second) {
        *result.first =
            MergeFlatBuffers(*result.first, RecursiveCopyFlatBuffer(a));
      }
    }
  }

  // Now repeat this for the node list.
  absl::btree_set<FlatbufferDetachedBuffer<Node>> nodes;
  if (config.message().has_nodes()) {
    auto_merge_config.mutable_message()->clear_nodes();
    for (const Node *n : *config.message().nodes()) {
      if (!n->has_name()) {
        continue;
      }

      auto result = nodes.insert(RecursiveCopyFlatBuffer(n));
      if (!result.second) {
        *result.first =
            MergeFlatBuffers(*result.first, RecursiveCopyFlatBuffer(n));
      }
    }
  }

  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  // Start by building the vectors.  They need to come before the final table.
  // Channels
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset;
  {
    ::std::vector<flatbuffers::Offset<Channel>> channel_offsets;
    for (const FlatbufferDetachedBuffer<Channel> &c : channels) {
      channel_offsets.emplace_back(
          RecursiveCopyFlatBuffer<Channel>(&c.message(), &fbb));
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
          RecursiveCopyFlatBuffer<Application>(&a.message(), &fbb));
    }
    applications_offset = fbb.CreateVector(applications_offsets);
  }

  // Nodes
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Node>>>
      nodes_offset;
  {
    ::std::vector<flatbuffers::Offset<Node>> node_offsets;
    for (const FlatbufferDetachedBuffer<Node> &n : nodes) {
      node_offsets.emplace_back(
          RecursiveCopyFlatBuffer<Node>(&n.message(), &fbb));
    }
    nodes_offset = fbb.CreateVector(node_offsets);
  }

  // And then build a Configuration with them all.
  ConfigurationBuilder configuration_builder(fbb);
  configuration_builder.add_channels(channels_offset);
  if (config.message().has_applications()) {
    configuration_builder.add_applications(applications_offset);
  }
  if (config.message().has_nodes()) {
    configuration_builder.add_nodes(nodes_offset);
  }

  fbb.Finish(configuration_builder.Finish());

  aos::FlatbufferDetachedBuffer<aos::Configuration> modified_config(
      fbb.Release());

  // Now, validate that if there is a node list, every channel has a source
  // node.
  FlatbufferDetachedBuffer<Configuration> result =
      MergeFlatBuffers(modified_config, auto_merge_config);

  ValidateConfiguration(result);

  return result;
}

FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path,
    const std::vector<std::string_view> &import_paths) {
  // We only want to read a file once.  So track the visited files in a set.
  absl::btree_set<std::string> visited_paths;
  FlatbufferDetachedBuffer<Configuration> read_config =
      ReadConfig(path, &visited_paths, import_paths);

  // If we only read one file, and it had a .bfbs extension, it has to be a
  // fully formatted config.  Do a quick verification and return it.
  if (visited_paths.size() == 1 && EndsWith(*visited_paths.begin(), ".bfbs")) {
    ValidateConfiguration(read_config);
    return read_config;
  }

  return MergeConfiguration(read_config);
}

FlatbufferDetachedBuffer<Configuration> MergeWithConfig(
    const Configuration *config, const Flatbuffer<Configuration> &addition) {
  return MergeConfiguration(MergeFlatBuffers(config, &addition.message()));
}

FlatbufferDetachedBuffer<Configuration> MergeWithConfig(
    const Configuration *config, std::string_view json) {
  FlatbufferDetachedBuffer<Configuration> addition =
      JsonToFlatbuffer(json, Configuration::MiniReflectTypeTable());

  return MergeWithConfig(config, addition);
}

const Channel *GetChannel(const Configuration *config, std::string_view name,
                          std::string_view type,
                          std::string_view application_name, const Node *node,
                          bool quiet) {
  if (!config->has_channels()) {
    return nullptr;
  }

  const std::string_view original_name = name;
  std::string mutable_name;
  if (node != nullptr) {
    VLOG(1) << "Looking up { \"name\": \"" << name << "\", \"type\": \"" << type
            << "\" } on " << aos::FlatbufferToJson(node);
  } else {
    VLOG(1) << "Looking up { \"name\": \"" << name << "\", \"type\": \"" << type
            << "\" }";
  }

  // First handle application specific maps.  Only do this if we have a matching
  // application name, and it has maps.
  {
    const Application *application =
        GetApplication(config, node, application_name);
    if (application != nullptr && application->has_maps()) {
      mutable_name = std::string(name);
      HandleMaps(application->maps(), &mutable_name, type, node);
      name = std::string_view(mutable_name);
    }
  }

  // Now do global maps.
  if (config->has_maps()) {
    mutable_name = std::string(name);
    HandleMaps(config->maps(), &mutable_name, type, node);
    name = std::string_view(mutable_name);
  }

  if (original_name != name) {
    VLOG(1) << "Remapped to { \"name\": \"" << name << "\", \"type\": \""
            << type << "\" }";
  }

  // Then look for the channel (note that this relies on the channels being
  // sorted in the config).
  auto channel_iterator =
      std::lower_bound(config->channels()->cbegin(), config->channels()->cend(),
                       std::make_pair(name, type), CompareChannels);

  // Make sure we actually found it, and it matches.
  if (channel_iterator != config->channels()->cend() &&
      EqualsChannels(*channel_iterator, std::make_pair(name, type))) {
    if (VLOG_IS_ON(2)) {
      VLOG(2) << "Found: " << FlatbufferToJson(*channel_iterator);
    } else if (VLOG_IS_ON(1)) {
      VLOG(1) << "Found: " << CleanedChannelToString(*channel_iterator);
    }
    return *channel_iterator;
  } else {
    VLOG(1) << "No match for { \"name\": \"" << name << "\", \"type\": \""
            << type << "\" }";
    if (original_name != name && !quiet) {
      LOG(WARNING) << "Remapped from {\"name\": \"" << original_name
                   << "\", \"type\": \"" << type << "\"}, to {\"name\": \""
                   << name << "\", \"type\": \"" << type
                   << "\"}, but no channel by that name exists.";
    }
    return nullptr;
  }
}

size_t ChannelIndex(const Configuration *configuration,
                    const Channel *channel) {
  CHECK(configuration->channels() != nullptr) << ": No channels";

  auto c = std::find(configuration->channels()->begin(),
                     configuration->channels()->end(), channel);
  CHECK(c != configuration->channels()->end())
      << ": Channel pointer not found in configuration()->channels()";

  return std::distance(configuration->channels()->begin(), c);
}

std::string CleanedChannelToString(const Channel *channel) {
  FlatbufferDetachedBuffer<Channel> cleaned_channel = CopyFlatBuffer(channel);
  cleaned_channel.mutable_message()->clear_schema();
  return FlatbufferToJson(cleaned_channel);
}

std::string StrippedChannelToString(const Channel *channel) {
  return absl::StrCat("{ \"name\": \"", channel->name()->string_view(),
                      "\", \"type\": \"", channel->type()->string_view(),
                      "\" }");
}

FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config,
    const std::vector<aos::FlatbufferVector<reflection::Schema>> &schemas) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  // Cache for holding already inserted schemas.
  std::map<std::string_view, flatbuffers::Offset<reflection::Schema>>
      schema_cache;

  CHECK_EQ(Channel::MiniReflectTypeTable()->num_elems, 13u)
      << ": Merging logic needs to be updated when the number of channel "
         "fields changes.";

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset;
  if (config.message().has_channels()) {
    std::vector<flatbuffers::Offset<Channel>> channel_offsets;
    for (const Channel *c : *config.message().channels()) {
      // Search for a schema with a matching type.
      const aos::FlatbufferVector<reflection::Schema> *found_schema = nullptr;
      for (const aos::FlatbufferVector<reflection::Schema> &schema : schemas) {
        if (schema.message().root_table() != nullptr) {
          if (schema.message().root_table()->name()->string_view() ==
              c->type()->string_view()) {
            found_schema = &schema;
          }
        }
      }

      CHECK(found_schema != nullptr)
          << ": Failed to find schema for " << FlatbufferToJson(c);

      // Now copy the message manually.
      auto cached_schema = schema_cache.find(c->type()->string_view());
      flatbuffers::Offset<reflection::Schema> schema_offset;
      if (cached_schema != schema_cache.end()) {
        schema_offset = cached_schema->second;
      } else {
        schema_offset = RecursiveCopyFlatBuffer<reflection::Schema>(
            &found_schema->message(), &fbb);
        schema_cache.emplace(c->type()->string_view(), schema_offset);
      }

      flatbuffers::Offset<flatbuffers::String> name_offset =
          fbb.CreateSharedString(c->name()->str());
      flatbuffers::Offset<flatbuffers::String> type_offset =
          fbb.CreateSharedString(c->type()->str());
      flatbuffers::Offset<flatbuffers::String> source_node_offset =
          c->has_source_node() ? fbb.CreateSharedString(c->source_node()->str())
                               : 0;

      flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Connection>>>
          destination_nodes_offset =
              aos::RecursiveCopyVectorTable(c->destination_nodes(), &fbb);

      flatbuffers::Offset<
          flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
          logger_nodes_offset =
              aos::CopyVectorSharedString(c->logger_nodes(), &fbb);

      Channel::Builder channel_builder(fbb);
      channel_builder.add_name(name_offset);
      channel_builder.add_type(type_offset);
      if (c->has_frequency()) {
        channel_builder.add_frequency(c->frequency());
      }
      if (c->has_max_size()) {
        channel_builder.add_max_size(c->max_size());
      }
      if (c->has_num_senders()) {
        channel_builder.add_num_senders(c->num_senders());
      }
      if (c->has_num_watchers()) {
        channel_builder.add_num_watchers(c->num_watchers());
      }
      channel_builder.add_schema(schema_offset);
      if (!source_node_offset.IsNull()) {
        channel_builder.add_source_node(source_node_offset);
      }
      if (!destination_nodes_offset.IsNull()) {
        channel_builder.add_destination_nodes(destination_nodes_offset);
      }
      if (c->has_logger()) {
        channel_builder.add_logger(c->logger());
      }
      if (!logger_nodes_offset.IsNull()) {
        channel_builder.add_logger_nodes(logger_nodes_offset);
      }
      if (c->has_read_method()) {
        channel_builder.add_read_method(c->read_method());
      }
      if (c->has_num_readers()) {
        channel_builder.add_num_readers(c->num_readers());
      }
      channel_offsets.emplace_back(channel_builder.Finish());
    }
    channels_offset = fbb.CreateVector(channel_offsets);
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Map>>>
      maps_offset =
          aos::RecursiveCopyVectorTable(config.message().maps(), &fbb);

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Node>>>
      nodes_offset =
          aos::RecursiveCopyVectorTable(config.message().nodes(), &fbb);

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
      applications_offset =
          aos::RecursiveCopyVectorTable(config.message().applications(), &fbb);

  // Now insert everything else in unmodified.
  ConfigurationBuilder configuration_builder(fbb);
  if (config.message().has_channels()) {
    configuration_builder.add_channels(channels_offset);
  }
  if (!maps_offset.IsNull()) {
    configuration_builder.add_maps(maps_offset);
  }
  if (!nodes_offset.IsNull()) {
    configuration_builder.add_nodes(nodes_offset);
  }
  if (!applications_offset.IsNull()) {
    configuration_builder.add_applications(applications_offset);
  }

  if (config.message().has_channel_storage_duration()) {
    configuration_builder.add_channel_storage_duration(
        config.message().channel_storage_duration());
  }

  CHECK_EQ(Configuration::MiniReflectTypeTable()->num_elems, 6u)
      << ": Merging logic needs to be updated when the number of configuration "
         "fields changes.";

  fbb.Finish(configuration_builder.Finish());
  aos::FlatbufferDetachedBuffer<aos::Configuration> modified_config(
      fbb.Release());

  return modified_config;
}

const Node *GetNodeFromHostname(const Configuration *config,
                                std::string_view hostname) {
  for (const Node *node : *config->nodes()) {
    if (node->has_hostname() && node->hostname()->string_view() == hostname) {
      return node;
    }
    if (node->has_hostnames()) {
      for (const auto &candidate : *node->hostnames()) {
        if (candidate->string_view() == hostname) {
          return node;
        }
      }
    }
  }
  return nullptr;
}

const Node *GetMyNode(const Configuration *config) {
  const std::string hostname = (FLAGS_override_hostname.size() > 0)
                                   ? FLAGS_override_hostname
                                   : network::GetHostname();
  const Node *node = GetNodeFromHostname(config, hostname);
  if (node != nullptr) return node;

  LOG(FATAL) << "Unknown node for host: " << hostname
             << ".  Consider using --override_hostname if hostname detection "
                "is wrong.";
  return nullptr;
}

const Node *GetNode(const Configuration *config, const Node *node) {
  if (!MultiNode(config)) {
    CHECK(node == nullptr) << ": Provided a node in a single node world.";
    return nullptr;
  } else {
    CHECK(node != nullptr);
    CHECK(node->has_name());
    return GetNode(config, node->name()->string_view());
  }
}

const Node *GetNode(const Configuration *config, std::string_view name) {
  CHECK(config->has_nodes())
      << ": Asking for a node from a single node configuration.";
  for (const Node *node : *config->nodes()) {
    CHECK(node->has_name()) << ": Malformed node " << FlatbufferToJson(node);
    if (node->name()->string_view() == name) {
      return node;
    }
  }
  return nullptr;
}

const Node *GetNode(const Configuration *config, size_t node_index) {
  if (!MultiNode(config)) {
    CHECK_EQ(node_index, 0u) << ": Invalid node in a single node world.";
    return nullptr;
  } else {
    CHECK_LT(node_index, config->nodes()->size());
    return config->nodes()->Get(node_index);
  }
}

const Node *GetNodeOrDie(const Configuration *config, const Node *node) {
  if (!MultiNode(config)) {
    CHECK(node == nullptr) << ": Provided a node in a single node world.";
    return nullptr;
  } else {
    const Node *config_node = GetNode(config, node);
    if (config_node == nullptr) {
      LOG(FATAL) << "Couldn't find node matching " << FlatbufferToJson(node);
    }
    return config_node;
  }
}

namespace {
int GetNodeIndexFromConfig(const Configuration *config, const Node *node) {
  int node_index = 0;
  for (const Node *iterated_node : *config->nodes()) {
    if (iterated_node == node) {
      return node_index;
    }
    ++node_index;
  }
  return -1;
}
}  // namespace

aos::FlatbufferDetachedBuffer<aos::Configuration> AddSchema(
    std::string_view json,
    const std::vector<aos::FlatbufferVector<reflection::Schema>> &schemas) {
  FlatbufferDetachedBuffer<Configuration> addition =
      JsonToFlatbuffer(json, Configuration::MiniReflectTypeTable());
  return MergeConfiguration(addition, schemas);
}

int GetNodeIndex(const Configuration *config, const Node *node) {
  if (!MultiNode(config)) {
    return 0;
  }

  {
    int node_index = GetNodeIndexFromConfig(config, node);
    if (node_index != -1) {
      return node_index;
    }
  }

  const Node *result = GetNode(config, node);
  CHECK(result != nullptr);

  {
    int node_index = GetNodeIndexFromConfig(config, result);
    if (node_index != -1) {
      return node_index;
    }
  }

  LOG(FATAL) << "Node " << FlatbufferToJson(node)
             << " not found in the configuration.";
}

int GetNodeIndex(const Configuration *config, std::string_view name) {
  if (!MultiNode(config)) {
    return 0;
  }

  {
    int node_index = 0;
    for (const Node *iterated_node : *config->nodes()) {
      if (iterated_node->name()->string_view() == name) {
        return node_index;
      }
      ++node_index;
    }
  }
  LOG(FATAL) << "Node " << name << " not found in the configuration.";
}

size_t NodesCount(const Configuration *config) {
  if (!MultiNode(config)) {
    return 1u;
  }

  return config->nodes()->size();
}

std::vector<const Node *> GetNodes(const Configuration *config) {
  std::vector<const Node *> nodes;
  if (MultiNode(config)) {
    for (const Node *node : *config->nodes()) {
      nodes.emplace_back(node);
    }
  } else {
    nodes.emplace_back(nullptr);
  }
  return nodes;
}

std::vector<const Node *> GetNodesWithTag(const Configuration *config,
                                          std::string_view tag) {
  std::vector<const Node *> nodes;
  if (!MultiNode(config)) {
    nodes.emplace_back(nullptr);
  } else {
    for (const Node *node : *config->nodes()) {
      if (!node->has_tags()) {
        continue;
      }
      bool did_found_tag = false;
      for (const flatbuffers::String *found_tag : *node->tags()) {
        if (found_tag->string_view() == tag) {
          did_found_tag = true;
          break;
        }
      }
      if (did_found_tag) {
        nodes.emplace_back(node);
      }
    }
  }
  return nodes;
}

bool MultiNode(const Configuration *config) { return config->has_nodes(); }

bool ChannelIsSendableOnNode(const Channel *channel, const Node *node) {
  if (node == nullptr) {
    return true;
  }
  CHECK(channel->has_source_node()) << FlatbufferToJson(channel);
  CHECK(node->has_name()) << FlatbufferToJson(node);
  return (CHECK_NOTNULL(channel)->source_node()->string_view() ==
          node->name()->string_view());
}

bool ChannelIsReadableOnNode(const Channel *channel, const Node *node) {
  if (node == nullptr) {
    return true;
  }

  if (channel->source_node()->string_view() == node->name()->string_view()) {
    return true;
  }

  if (!channel->has_destination_nodes()) {
    return false;
  }

  for (const Connection *connection : *channel->destination_nodes()) {
    CHECK(connection->has_name());
    if (connection->name()->string_view() == node->name()->string_view()) {
      return true;
    }
  }

  return false;
}

bool ChannelMessageIsLoggedOnNode(const Channel *channel, const Node *node) {
  if (node == nullptr) {
    // Single node world.  If there is a local logger, then we want to use
    // it.
    if (channel->logger() == LoggerConfig::LOCAL_LOGGER) {
      return true;
    } else if (channel->logger() == LoggerConfig::NOT_LOGGED) {
      return false;
    }
    LOG(FATAL) << "Unsupported logging configuration in a single node world: "
               << CleanedChannelToString(channel);
  }
  return ChannelMessageIsLoggedOnNode(
      channel, CHECK_NOTNULL(node)->name()->string_view());
}

bool ChannelMessageIsLoggedOnNode(const Channel *channel,
                                  std::string_view node_name) {
  switch (channel->logger()) {
    case LoggerConfig::LOCAL_LOGGER:
      return channel->source_node()->string_view() == node_name;
    case LoggerConfig::LOCAL_AND_REMOTE_LOGGER:
      CHECK(channel->has_logger_nodes());
      CHECK_GT(channel->logger_nodes()->size(), 0u);

      if (channel->source_node()->string_view() == node_name) {
        return true;
      }

      [[fallthrough]];
    case LoggerConfig::REMOTE_LOGGER:
      CHECK(channel->has_logger_nodes());
      CHECK_GT(channel->logger_nodes()->size(), 0u);
      for (const flatbuffers::String *logger_node : *channel->logger_nodes()) {
        if (logger_node->string_view() == node_name) {
          return true;
        }
      }

      return false;
    case LoggerConfig::NOT_LOGGED:
      return false;
  }

  LOG(FATAL) << "Unknown logger config " << static_cast<int>(channel->logger());
}

const Connection *ConnectionToNode(const Channel *channel, const Node *node) {
  if (!channel->has_destination_nodes()) {
    return nullptr;
  }
  for (const Connection *connection : *channel->destination_nodes()) {
    if (connection->name()->string_view() == node->name()->string_view()) {
      return connection;
    }
  }
  return nullptr;
}

bool ConnectionDeliveryTimeIsLoggedOnNode(const Channel *channel,
                                          const Node *node,
                                          const Node *logger_node) {
  const Connection *connection = ConnectionToNode(channel, node);
  if (connection == nullptr) {
    return false;
  }
  return ConnectionDeliveryTimeIsLoggedOnNode(connection, logger_node);
}

bool ConnectionDeliveryTimeIsLoggedOnNode(const Connection *connection,
                                          const Node *node) {
  switch (connection->timestamp_logger()) {
    case LoggerConfig::LOCAL_AND_REMOTE_LOGGER:
      CHECK(connection->has_timestamp_logger_nodes());
      CHECK_GT(connection->timestamp_logger_nodes()->size(), 0u);
      if (connection->name()->string_view() == node->name()->string_view()) {
        return true;
      }

      [[fallthrough]];
    case LoggerConfig::REMOTE_LOGGER:
      CHECK(connection->has_timestamp_logger_nodes());
      CHECK_GT(connection->timestamp_logger_nodes()->size(), 0u);
      for (const flatbuffers::String *timestamp_logger_node :
           *connection->timestamp_logger_nodes()) {
        if (timestamp_logger_node->string_view() ==
            node->name()->string_view()) {
          return true;
        }
      }

      return false;
    case LoggerConfig::LOCAL_LOGGER:
      return connection->name()->string_view() == node->name()->string_view();
    case LoggerConfig::NOT_LOGGED:
      return false;
  }

  LOG(FATAL) << "Unknown logger config "
             << static_cast<int>(connection->timestamp_logger());
}

std::vector<std::string_view> SourceNodeNames(const Configuration *config,
                                              const Node *my_node) {
  std::set<std::string_view> result_set;

  for (const Channel *channel : *config->channels()) {
    if (channel->has_destination_nodes()) {
      for (const Connection *connection : *channel->destination_nodes()) {
        if (connection->name()->string_view() ==
            my_node->name()->string_view()) {
          result_set.insert(channel->source_node()->string_view());
        }
      }
    }
  }

  std::vector<std::string_view> result;
  for (const std::string_view source : result_set) {
    VLOG(1) << "Found a source node of " << source;
    result.emplace_back(source);
  }
  return result;
}

std::vector<std::string_view> DestinationNodeNames(const Configuration *config,
                                                   const Node *my_node) {
  std::vector<std::string_view> result;

  for (const Channel *channel : *config->channels()) {
    if (channel->has_source_node() && channel->source_node()->string_view() ==
                                          my_node->name()->string_view()) {
      if (!channel->has_destination_nodes()) continue;

      if (channel->source_node()->string_view() !=
          my_node->name()->string_view()) {
        continue;
      }

      for (const Connection *connection : *channel->destination_nodes()) {
        if (std::find(result.begin(), result.end(),
                      connection->name()->string_view()) == result.end()) {
          result.emplace_back(connection->name()->string_view());
        }
      }
    }
  }

  for (const std::string_view destination : result) {
    VLOG(1) << "Found a destination node of " << destination;
  }
  return result;
}

std::vector<const Node *> TimestampNodes(const Configuration *config,
                                         const Node *my_node) {
  if (!configuration::MultiNode(config)) {
    CHECK(my_node == nullptr);
    return std::vector<const Node *>{};
  }

  std::set<const Node *> timestamp_logger_nodes;
  for (const Channel *channel : *config->channels()) {
    if (!configuration::ChannelIsSendableOnNode(channel, my_node)) {
      continue;
    }
    if (!channel->has_destination_nodes()) {
      continue;
    }
    for (const Connection *connection : *channel->destination_nodes()) {
      const Node *other_node =
          configuration::GetNode(config, connection->name()->string_view());

      if (configuration::ConnectionDeliveryTimeIsLoggedOnNode(connection,
                                                              my_node)) {
        VLOG(1) << "Timestamps are logged from "
                << FlatbufferToJson(other_node);
        timestamp_logger_nodes.insert(other_node);
      }
    }
  }

  std::vector<const Node *> result;
  for (const Node *node : timestamp_logger_nodes) {
    result.emplace_back(node);
  }
  return result;
}

const Application *GetApplication(const Configuration *config,
                                  const Node *my_node,
                                  std::string_view application_name) {
  if (config->has_applications()) {
    auto application_iterator = std::lower_bound(
        config->applications()->cbegin(), config->applications()->cend(),
        application_name, CompareApplications);
    if (application_iterator != config->applications()->cend() &&
        EqualsApplications(*application_iterator, application_name)) {
      if (MultiNode(config)) {
        // Ok, we need
        CHECK(application_iterator->has_nodes());
        CHECK(my_node != nullptr);
        for (const flatbuffers::String *str : *application_iterator->nodes()) {
          if (str->string_view() == my_node->name()->string_view()) {
            return *application_iterator;
          }
        }
      } else {
        return *application_iterator;
      }
    }
  }
  return nullptr;
}

}  // namespace configuration
}  // namespace aos
