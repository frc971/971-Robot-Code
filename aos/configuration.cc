#include "aos/configuration.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/container/btree_set.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/configuration_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/ipc_lib/index.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/unique_malloc_ptr.h"
#include "aos/util/file.h"

DEFINE_uint32(max_queue_size_override, 0,
              "If nonzero, this is the max number of elements in a queue to "
              "enforce.  If zero, use the number that the processor that this "
              "application is compiled for can support.  This is mostly useful "
              "for config validation, and shouldn't be touched.");

namespace aos {
namespace configuration {
namespace {
namespace chrono = std::chrono;

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

void ValidateUnmergedConfiguration(const Flatbuffer<Configuration> &config);

FlatbufferDetachedBuffer<Configuration> ReadConfigFile(std::string_view path,
                                                       bool binary) {
  if (binary) {
    FlatbufferVector<Configuration> config =
        FileToFlatbuffer<Configuration>(path);
    return CopySpanAsDetachedBuffer(config.span());
  }

  flatbuffers::DetachedBuffer buffer = JsonToFlatbuffer(
      util::ReadFileToStringOrDie(path), ConfigurationTypeTable());

  CHECK_GT(buffer.size(), 0u) << ": Failed to parse JSON file: " << path;

  FlatbufferDetachedBuffer<Configuration> result(std::move(buffer));
  configuration::ValidateUnmergedConfiguration(result);
  return result;
}

// Struct representing a Connection in a channel in a way that is easy to work
// with.
struct MutableConnection {
  // See configuration.fbs for a description of what each of these fields
  // represents.
  std::string_view name;
  std::optional<LoggerConfig> timestamp_logger;
  absl::btree_set<std::string_view> timestamp_logger_nodes;
  std::optional<uint16_t> priority;
  std::optional<uint32_t> time_to_live;
};

// The name of a channel.
struct MutableChannelName {
  std::string_view name;
  std::string_view type;

  bool operator==(const MutableChannelName &other) const {
    return std::make_tuple(name, type) ==
           std::make_tuple(other.name, other.type);
  }
  std::strong_ordering operator<=>(const MutableChannelName &other) const {
    return std::make_tuple(name, type) <=>
           std::make_tuple(other.name, other.type);
  }
};

// Struct representing a Channel in a way that is easy to work with.
struct MutableChannel {
  // See configuration.fbs for a description of what each of these fields
  // represents.
  std::string_view name;
  std::string_view type;
  std::optional<int32_t> frequency;
  std::optional<int32_t> max_size;
  std::optional<int32_t> num_senders;
  std::optional<int32_t> num_watchers;

  std::string_view source_node;
  absl::btree_map<std::string_view, MutableConnection> destination_nodes;
  std::optional<LoggerConfig> logger;
  absl::btree_set<std::string_view> logger_nodes;
  std::optional<ReadMethod> read_method;
  std::optional<int32_t> num_readers;

  std::optional<int64_t> channel_storage_duration;
};

// Struct representing a Node in a way that is easy to work with.
struct MutableNode {
  // See configuration.fbs for a description of what each of these fields
  // represents.
  std::string_view name;
  std::string_view hostname;
  std::optional<uint16_t> port;
  absl::btree_set<std::string_view> hostnames;
  absl::btree_set<std::string_view> tags;
};

// Struct representing a Map in a way that is easy to work with.
struct MutableMap {
  // See configuration.fbs for a description of what each of these fields
  // represents.
  MutableChannel match;
  MutableChannel rename;
};

// Struct representing an Application in a way that is easy to work with.
struct MutableApplication {
  // See configuration.fbs for a description of what each of these fields
  // represents.
  std::string_view name;
  std::string_view executable_name;
  std::vector<MutableMap> maps;
  absl::btree_set<std::string_view> nodes;
  std::string_view user;
  std::vector<std::string_view> args;
  std::optional<bool> autostart;
  std::optional<bool> autorestart;
  std::optional<uint64_t> memory_limit;
  std::optional<int64_t> stop_time;

  bool operator==(const MutableApplication &other) const {
    return name == other.name;
  }
  std::strong_ordering operator<=>(const MutableApplication &other) const {
    return name <=> other.name;
  }
};

// Struct representing a Configuration in a way that is easy to work with.  To
// use this class, start with a flatbuffer configuration, call
// UnpackConfiguration() on it, and then manipulate from there.  (Note: this API
// generally assumes that the lifetime of strings is managed by something else,
// and a string_view is good enough).
// PackConfiguration() creates the corresponding flatbuffer from a mutable
// configuration for downstream use.
struct MutableConfiguration {
  // See configuration.fbs for a description of what each of these fields
  // represents.
  // TODO(austin): Is this a better object for LogReader to manipulate than raw
  // configs?
  absl::btree_map<MutableChannelName, MutableChannel> channels;
  std::vector<MutableMap> maps;
  absl::btree_map<std::string_view, MutableNode> nodes;
  absl::btree_map<std::string_view, MutableApplication> applications;
  std::optional<uint64_t> channel_storage_duration;

  absl::btree_map<std::string_view, const reflection::Schema *> schemas;
};

// Unpacks a vector of strings into a set.
void UnpackStringSet(
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>
        *strings,
    absl::btree_set<std::string_view> *result) {
  for (const flatbuffers::String *str : *strings) {
    result->insert(str->string_view());
  }
}

void UnpackConnection(const Connection *destination_node,
                      MutableConnection *result) {
  CHECK_EQ(Connection::MiniReflectTypeTable()->num_elems, 5u)
      << ": Merging logic needs to be updated when the number of connection "
         "fields changes.";
  if (destination_node->has_timestamp_logger()) {
    result->timestamp_logger = destination_node->timestamp_logger();
  }

  if (destination_node->has_timestamp_logger_nodes()) {
    UnpackStringSet(destination_node->timestamp_logger_nodes(),
                    &result->timestamp_logger_nodes);
  }
  if (destination_node->has_priority()) {
    result->priority = destination_node->priority();
  }
  if (destination_node->has_time_to_live()) {
    result->time_to_live = destination_node->time_to_live();
  }
}

void UnpackChannel(const Channel *channel, MutableChannel *result) {
  CHECK_EQ(Channel::MiniReflectTypeTable()->num_elems, 14u)
      << ": Merging logic needs to be updated when the number of channel "
         "fields changes.";

  if (channel->has_name()) {
    result->name = channel->name()->string_view();
  }
  if (channel->has_type()) {
    result->type = channel->type()->string_view();
  }
  if (channel->has_frequency()) {
    result->frequency = channel->frequency();
  }
  if (channel->has_max_size()) {
    result->max_size = channel->max_size();
  }
  if (channel->has_num_senders()) {
    result->num_senders = channel->num_senders();
  }
  if (channel->has_num_watchers()) {
    result->num_watchers = channel->num_watchers();
  }
  if (channel->has_source_node()) {
    result->source_node = channel->source_node()->string_view();
  }
  if (channel->has_destination_nodes()) {
    for (const Connection *destination_node : *channel->destination_nodes()) {
      MutableConnection &destination =
          result->destination_nodes
              .try_emplace(destination_node->name()->string_view(),
                           MutableConnection{
                               .name = destination_node->name()->string_view(),
                           })
              .first->second;
      UnpackConnection(destination_node, &destination);
    }
  }
  if (channel->has_logger()) {
    result->logger = channel->logger();
  }
  if (channel->has_logger_nodes()) {
    UnpackStringSet(channel->logger_nodes(), &(result->logger_nodes));
  }

  if (channel->has_read_method()) {
    result->read_method = channel->read_method();
  }
  if (channel->has_num_readers()) {
    result->num_readers = channel->num_readers();
  }
  if (channel->has_channel_storage_duration()) {
    result->channel_storage_duration = channel->channel_storage_duration();
  }
}

void UnpackNode(const Node *node, MutableNode *result) {
  CHECK_EQ(node->name()->string_view(), result->name);
  CHECK_EQ(Node::MiniReflectTypeTable()->num_elems, 5u)
      << ": Merging logic needs to be updated when the number of node "
         "fields changes.";
  if (node->has_hostname()) {
    result->hostname = node->hostname()->string_view();
  }
  if (node->has_port()) {
    result->port = node->port();
  }

  if (node->has_hostnames()) {
    UnpackStringSet(node->hostnames(), &(result->hostnames));
  }
  if (node->has_tags()) {
    UnpackStringSet(node->tags(), &(result->tags));
  }
}

void UnpackMap(const Map *map, MutableMap *result) {
  CHECK(map->has_match());
  CHECK(map->has_rename());
  UnpackChannel(map->match(), &(result->match));
  UnpackChannel(map->rename(), &(result->rename));
}

void UnpackApplication(const Application *application,
                       MutableApplication *result) {
  CHECK_EQ(application->name()->string_view(), result->name);

  if (application->has_executable_name()) {
    result->executable_name = application->executable_name()->string_view();
  }
  if (application->has_maps()) {
    result->maps.reserve(application->maps()->size());
    for (const Map *map : *application->maps()) {
      result->maps.emplace_back();
      UnpackMap(map, &(result->maps.back()));
    }
  }

  if (application->has_nodes()) {
    UnpackStringSet(application->nodes(), &(result->nodes));
  }

  if (application->has_user()) {
    result->user = application->user()->string_view();
  }

  if (application->has_args()) {
    // Very important, arguments replace old arguments.
    result->args.clear();

    result->args.reserve(application->args()->size());
    for (const flatbuffers::String *arg : *application->args()) {
      result->args.emplace_back(arg->string_view());
    }
  }

  if (application->has_autostart()) {
    result->autostart = application->autostart();
  }
  if (application->has_autorestart()) {
    result->autorestart = application->autorestart();
  }
  if (application->has_memory_limit()) {
    result->memory_limit = application->memory_limit();
  }
  if (application->has_stop_time()) {
    result->stop_time = application->stop_time();
  }
}

void UnpackConfiguration(const Configuration *configuration,
                         MutableConfiguration *result) {
  if (configuration->has_channels()) {
    for (const Channel *channel : *configuration->channels()) {
      // Explode on malformed entries.
      CHECK(channel->has_name() && channel->has_type());

      // Attempt to insert the channel.
      MutableChannel &unpacked_channel =
          result->channels
              .try_emplace(
                  MutableChannelName{
                      .name = channel->name()->string_view(),
                      .type = channel->type()->string_view(),
                  },
                  MutableChannel{
                      .name = channel->name()->string_view(),
                      .type = channel->type()->string_view(),
                  })
              .first->second;

      UnpackChannel(channel, &unpacked_channel);
      if (channel->has_schema()) {
        result->schemas.emplace(channel->type()->string_view(),
                                channel->schema());
      }
    }
  }

  if (configuration->has_maps()) {
    result->maps.reserve(configuration->maps()->size());

    for (const Map *map : *configuration->maps()) {
      CHECK(map->has_match());
      CHECK(map->has_rename());

      result->maps.emplace_back();
      UnpackMap(map, &(result->maps.back()));
    }
  }

  if (configuration->has_nodes()) {
    for (const Node *node : *configuration->nodes()) {
      CHECK(node->has_name());

      MutableNode &unpacked_node =
          result->nodes
              .try_emplace(node->name()->string_view(),
                           MutableNode{
                               .name = node->name()->string_view(),
                           })
              .first->second;
      UnpackNode(node, &unpacked_node);
    }
  }

  if (configuration->has_applications()) {
    for (const Application *application : *configuration->applications()) {
      CHECK(application->has_name());

      MutableApplication &unpacked_application =
          result->applications
              .try_emplace(application->name()->string_view(),
                           MutableApplication{
                               .name = application->name()->string_view(),
                           })
              .first->second;
      UnpackApplication(application, &unpacked_application);
    }
  }

  if (configuration->has_channel_storage_duration()) {
    result->channel_storage_duration =
        configuration->channel_storage_duration();
  }
}

flatbuffers::Offset<
    flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
PackStringSet(const absl::btree_set<std::string_view> &set,
              flatbuffers::FlatBufferBuilder *fbb) {
  std::vector<flatbuffers::Offset<flatbuffers::String>> strings_offsets;
  for (const std::string_view &str : set) {
    strings_offsets.push_back(fbb->CreateSharedString(str));
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      result;

  if (!strings_offsets.empty()) {
    result = fbb->CreateVector(strings_offsets);
  }
  return result;
}

flatbuffers::Offset<Connection> PackConnection(
    const MutableConnection &destination_node,
    flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb->CreateSharedString(destination_node.name);

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      timestamp_logger_nodes_offset =
          PackStringSet(destination_node.timestamp_logger_nodes, fbb);

  Connection::Builder connection_builder(*fbb);
  connection_builder.add_name(name_offset);
  if (destination_node.timestamp_logger.has_value()) {
    connection_builder.add_timestamp_logger(
        destination_node.timestamp_logger.value());
  }

  if (!timestamp_logger_nodes_offset.IsNull()) {
    connection_builder.add_timestamp_logger_nodes(
        timestamp_logger_nodes_offset);
  }
  if (destination_node.priority.has_value()) {
    connection_builder.add_priority(destination_node.priority.value());
  }
  if (destination_node.time_to_live.has_value()) {
    connection_builder.add_time_to_live(destination_node.time_to_live.value());
  }
  return connection_builder.Finish();
}

flatbuffers::Offset<Channel> PackChannel(
    const MutableChannel &channel,
    flatbuffers::Offset<reflection::Schema> schema_offset,
    flatbuffers::FlatBufferBuilder *fbb) {
  std::vector<flatbuffers::Offset<Connection>> connection_offsets;

  for (const std::pair<const std::string_view, MutableConnection>
           &destination_node : channel.destination_nodes) {
    CHECK_EQ(destination_node.first, destination_node.second.name);
    connection_offsets.push_back(PackConnection(destination_node.second, fbb));
  }

  flatbuffers::Offset<flatbuffers::String> name_offset;
  if (!channel.name.empty()) {
    name_offset = fbb->CreateSharedString(channel.name);
  }
  flatbuffers::Offset<flatbuffers::String> type_offset;
  if (!channel.type.empty()) {
    type_offset = fbb->CreateSharedString(channel.type);
  }
  flatbuffers::Offset<flatbuffers::String> source_node_offset;
  if (!channel.source_node.empty()) {
    source_node_offset = fbb->CreateSharedString(channel.source_node);
  }
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Connection>>>
      destination_nodes_offset;
  if (!connection_offsets.empty()) {
    destination_nodes_offset = fbb->CreateVector(connection_offsets);
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      logger_nodes_offset = PackStringSet(channel.logger_nodes, fbb);

  Channel::Builder channel_builder(*fbb);

  if (!name_offset.IsNull()) {
    channel_builder.add_name(name_offset);
  }
  if (!type_offset.IsNull()) {
    channel_builder.add_type(type_offset);
  }
  if (channel.frequency.has_value()) {
    channel_builder.add_frequency(channel.frequency.value());
  }
  if (channel.max_size.has_value()) {
    channel_builder.add_max_size(channel.max_size.value());
  }
  if (channel.num_senders.has_value()) {
    channel_builder.add_num_senders(channel.num_senders.value());
  }
  if (channel.num_watchers.has_value()) {
    channel_builder.add_num_watchers(channel.num_watchers.value());
  }

  if (!schema_offset.IsNull()) {
    channel_builder.add_schema(schema_offset);
  }

  if (!source_node_offset.IsNull()) {
    channel_builder.add_source_node(source_node_offset);
  }
  if (!destination_nodes_offset.IsNull()) {
    channel_builder.add_destination_nodes(destination_nodes_offset);
  }

  if (channel.logger.has_value()) {
    channel_builder.add_logger(channel.logger.value());
  }
  if (!logger_nodes_offset.IsNull()) {
    channel_builder.add_logger_nodes(logger_nodes_offset);
  }
  if (channel.read_method.has_value()) {
    channel_builder.add_read_method(channel.read_method.value());
  }
  if (channel.num_readers.has_value()) {
    channel_builder.add_num_readers(channel.num_readers.value());
  }
  if (channel.channel_storage_duration.has_value()) {
    channel_builder.add_channel_storage_duration(
        channel.channel_storage_duration.value());
  }

  return channel_builder.Finish();
}

flatbuffers::Offset<Node> PackNode(const MutableNode &node,
                                   flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb->CreateSharedString(node.name);
  flatbuffers::Offset<flatbuffers::String> hostname_offset;
  if (!node.hostname.empty()) {
    hostname_offset = fbb->CreateSharedString(node.hostname);
  }
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      hostnames_offset = PackStringSet(node.hostnames, fbb);
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      tags_offset = PackStringSet(node.tags, fbb);
  Node::Builder node_builder(*fbb);
  node_builder.add_name(name_offset);
  if (!hostname_offset.IsNull()) {
    node_builder.add_hostname(hostname_offset);
  }
  if (node.port) {
    node_builder.add_port(node.port.value());
  }
  if (!hostnames_offset.IsNull()) {
    node_builder.add_hostnames(hostnames_offset);
  }
  if (!tags_offset.IsNull()) {
    node_builder.add_tags(tags_offset);
  }
  return node_builder.Finish();
}

flatbuffers::Offset<Map> PackMap(const MutableMap &map,
                                 flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::Offset<Channel> match_offset =
      PackChannel(map.match, flatbuffers::Offset<reflection::Schema>(), fbb);
  flatbuffers::Offset<Channel> rename_offset =
      PackChannel(map.rename, flatbuffers::Offset<reflection::Schema>(), fbb);
  Map::Builder map_builder(*fbb);
  map_builder.add_match(match_offset);
  map_builder.add_rename(rename_offset);
  return map_builder.Finish();
}

flatbuffers::Offset<Application> PackApplication(
    const MutableApplication &application,
    flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb->CreateSharedString(application.name);

  flatbuffers::Offset<flatbuffers::String> executable_name_offset;
  if (!application.executable_name.empty()) {
    executable_name_offset =
        fbb->CreateSharedString(application.executable_name);
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Map>>>
      maps_offset;
  if (!application.maps.empty()) {
    std::vector<flatbuffers::Offset<Map>> maps_offsets;
    maps_offsets.reserve(application.maps.size());
    for (const MutableMap &map : application.maps) {
      maps_offsets.emplace_back(PackMap(map, fbb));
    }
    maps_offset = fbb->CreateVector(maps_offsets);
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      nodes_offset = PackStringSet(application.nodes, fbb);

  flatbuffers::Offset<flatbuffers::String> user_offset;
  if (!application.user.empty()) {
    user_offset = fbb->CreateSharedString(application.user);
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      args_offset;
  if (!application.args.empty()) {
    std::vector<flatbuffers::Offset<flatbuffers::String>> args_offsets;
    for (const std::string_view arg : application.args) {
      args_offsets.emplace_back(fbb->CreateSharedString(arg));
    }
    args_offset = fbb->CreateVector(args_offsets);
  }

  Application::Builder application_builder(*fbb);
  application_builder.add_name(name_offset);
  if (!executable_name_offset.IsNull()) {
    application_builder.add_executable_name(executable_name_offset);
  }
  if (!maps_offset.IsNull()) {
    application_builder.add_maps(maps_offset);
  }
  if (!nodes_offset.IsNull()) {
    application_builder.add_nodes(nodes_offset);
  }
  if (!user_offset.IsNull()) {
    application_builder.add_user(user_offset);
  }
  if (!args_offset.IsNull()) {
    application_builder.add_args(args_offset);
  }
  if (application.autostart) {
    application_builder.add_autostart(application.autostart.value());
  }
  if (application.autorestart) {
    application_builder.add_autorestart(application.autorestart.value());
  }
  if (application.memory_limit) {
    application_builder.add_memory_limit(application.memory_limit.value());
  }
  if (application.stop_time) {
    application_builder.add_stop_time(application.stop_time.value());
  }
  return application_builder.Finish();
}

flatbuffers::Offset<Configuration> PackConfiguration(
    const MutableConfiguration &configuration,
    flatbuffers::FlatBufferBuilder *fbb) {
  // Start by building the vectors.  They need to come before the final table.
  // Channels
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset;
  {
    // We want to add channels unconditionally since everyone expects them to be
    // there.
    std::map<std::string_view, flatbuffers::Offset<reflection::Schema>>
        schema_cache;

    std::vector<flatbuffers::Offset<Channel>> channel_offsets;
    for (const std::pair<const MutableChannelName, MutableChannel>
             &channel_key : configuration.channels) {
      CHECK_EQ(channel_key.first.name, channel_key.second.name);
      CHECK_EQ(channel_key.first.type, channel_key.second.type);
      const MutableChannel &channel = channel_key.second;
      auto cached_schema = schema_cache.find(channel.type);
      flatbuffers::Offset<reflection::Schema> schema_offset;
      if (cached_schema != schema_cache.end()) {
        schema_offset = cached_schema->second;
      } else {
        auto schema_to_copy_it = configuration.schemas.find(channel.type);
        if (schema_to_copy_it != configuration.schemas.end()) {
          schema_offset = RecursiveCopyFlatBuffer<reflection::Schema>(
              schema_to_copy_it->second, fbb);
          schema_cache.emplace(channel.type, schema_offset);
        }
      }

      channel_offsets.emplace_back(PackChannel(channel, schema_offset, fbb));
    }
    channels_offset = fbb->CreateVector(channel_offsets);
  }

  // Maps
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Map>>>
      maps_offset;
  if (!configuration.maps.empty()) {
    std::vector<flatbuffers::Offset<Map>> map_offsets;
    for (const MutableMap &map : configuration.maps) {
      map_offsets.emplace_back(PackMap(map, fbb));
    }
    maps_offset = fbb->CreateVector(map_offsets);
  }

  // Nodes
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Node>>>
      nodes_offset;
  if (!configuration.nodes.empty()) {
    std::vector<flatbuffers::Offset<Node>> node_offsets;
    for (const std::pair<const std::string_view, MutableNode> &node :
         configuration.nodes) {
      CHECK_EQ(node.first, node.second.name);
      node_offsets.emplace_back(PackNode(node.second, fbb));
    }
    nodes_offset = fbb->CreateVector(node_offsets);
  }

  // Applications
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
      applications_offset;
  if (!configuration.applications.empty()) {
    std::vector<flatbuffers::Offset<Application>> applications_offsets;
    for (const std::pair<const std::string_view, MutableApplication>
             &application : configuration.applications) {
      CHECK_EQ(application.first, application.second.name);
      applications_offsets.emplace_back(
          PackApplication(application.second, fbb));
    }
    applications_offset = fbb->CreateVector(applications_offsets);
  }

  // And then build a Configuration with them all.
  ConfigurationBuilder configuration_builder(*fbb);
  configuration_builder.add_channels(channels_offset);
  if (!maps_offset.IsNull()) {
    configuration_builder.add_maps(maps_offset);
  }
  if (!nodes_offset.IsNull()) {
    configuration_builder.add_nodes(nodes_offset);
  }
  if (!applications_offset.IsNull()) {
    configuration_builder.add_applications(applications_offset);
  }
  if (configuration.channel_storage_duration) {
    configuration_builder.add_channel_storage_duration(
        configuration.channel_storage_duration.value());
  }

  return configuration_builder.Finish();
}

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

std::string RemoveDotDots(const std::string_view filename) {
  std::vector<std::string> split = absl::StrSplit(filename, '/');
  auto iterator = split.begin();
  while (iterator != split.end()) {
    if (iterator->empty()) {
      iterator = split.erase(iterator);
    } else if (*iterator == ".") {
      iterator = split.erase(iterator);
    } else if (*iterator == "..") {
      CHECK(iterator != split.begin())
          << ": Import path may not start with ..: " << filename;
      auto previous = iterator;
      --previous;
      split.erase(iterator);
      iterator = split.erase(previous);
    } else {
      ++iterator;
    }
  }
  return absl::StrJoin(split, "/");
}

std::optional<FlatbufferDetachedBuffer<Configuration>> MaybeReadConfig(
    const std::string_view path, absl::btree_set<std::string> *visited_paths,
    const std::vector<std::string_view> &extra_import_paths) {
  std::string binary_path = MaybeReplaceExtension(path, ".json", ".bfbs");
  VLOG(1) << "Looking up: " << path << ", starting with: " << binary_path;
  bool binary_path_exists = util::PathExists(binary_path);
  std::string raw_path(path);
  // For each .json file, look and see if we can find a .bfbs file next to it
  // with the same base name.  If we can, assume it is the same and use it
  // instead.  It is much faster to load .bfbs files than .json files.
  if (!binary_path_exists && !util::PathExists(raw_path)) {
    const bool path_is_absolute = raw_path.size() > 0 && raw_path[0] == '/';
    if (path_is_absolute) {
      // Nowhere else to look up an absolute path, so fail now. Note that we
      // always have at least one extra import path based on /proc/self/exe, so
      // warning about those paths existing isn't helpful.
      LOG(ERROR) << ": Failed to find file " << path << ".";
      return std::nullopt;
    }

    bool found_path = false;
    for (const auto &import_path : extra_import_paths) {
      raw_path = std::string(import_path) + "/" + RemoveDotDots(path);
      binary_path = MaybeReplaceExtension(raw_path, ".json", ".bfbs");
      VLOG(1) << "Checking: " << binary_path;
      binary_path_exists = util::PathExists(binary_path);
      if (binary_path_exists) {
        found_path = true;
        break;
      }
      VLOG(1) << "Checking: " << raw_path;
      if (util::PathExists(raw_path)) {
        found_path = true;
        break;
      }
    }
    if (!found_path) {
      LOG(ERROR) << ": Failed to find file " << path << ".";
      return std::nullopt;
    }
  }

  std::optional<FlatbufferDetachedBuffer<Configuration>> config =
      ReadConfigFile(binary_path_exists ? binary_path : raw_path,
                     binary_path_exists);

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
    return std::nullopt;
  }

  if (config->message().has_imports()) {
    // Capture the imports.
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> *v =
        config->message().imports();

    // And then wipe them.  This gets GCed when we merge later.
    config->mutable_message()->clear_imports();

    // Start with an empty configuration to merge into.
    FlatbufferDetachedBuffer<Configuration> merged_config =
        FlatbufferDetachedBuffer<Configuration>::Empty();

    const std::string path_folder(ExtractFolder(path));
    for (const flatbuffers::String *str : *v) {
      const std::string included_config =
          path_folder + "/" + std::string(str->string_view());

      const auto optional_config =
          MaybeReadConfig(included_config, visited_paths, extra_import_paths);
      if (!optional_config.has_value()) {
        return std::nullopt;
      }
      // And them merge everything in.
      merged_config = MergeFlatBuffers(merged_config, *optional_config);
    }

    // Finally, merge this file in.
    config = MergeFlatBuffers(merged_config, *config);
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

void ValidateUnmergedConfiguration(const Flatbuffer<Configuration> &config) {
  // Check that if there is a node list, all the source nodes are filled out and
  // valid, and all the destination nodes are valid (and not the source).  This
  // is a basic consistency check.
  if (config.message().has_channels()) {
    for (const Channel *c : *config.message().channels()) {
      CHECK(c->has_name());
      CHECK(c->has_type());
      if (c->name()->string_view().back() == '/') {
        LOG(FATAL) << "Channel names can't end with '/'";
      }
      if (c->name()->string_view().front() != '/') {
        LOG(FATAL) << "Channel names must start with '/'";
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

      CHECK_LT(QueueSize(&config.message(), c) + QueueScratchBufferSize(c),
               FLAGS_max_queue_size_override != 0
                   ? FLAGS_max_queue_size_override
                   : std::numeric_limits<
                         ipc_lib::QueueIndex::PackedIndexType>::max())
          << ": More messages/second configured than the queue can hold on "
          << CleanedChannelToString(c) << ", " << c->frequency() << "hz for "
          << ChannelStorageDuration(&config.message(), c).count() << "ns";

      if (c->has_logger_nodes()) {
        // Confirm that we don't have duplicate logger nodes.
        absl::btree_set<std::string_view> logger_nodes;
        for (const flatbuffers::String *s : *c->logger_nodes()) {
          logger_nodes.insert(s->string_view());
        }
        CHECK_EQ(static_cast<size_t>(logger_nodes.size()),
                 c->logger_nodes()->size())
            << ": Found duplicate logger_nodes in "
            << CleanedChannelToString(c);
      }

      if (c->has_destination_nodes()) {
        // Confirm that we don't have duplicate timestamp logger nodes.
        for (const Connection *d : *c->destination_nodes()) {
          if (d->has_timestamp_logger_nodes()) {
            absl::btree_set<std::string_view> timestamp_logger_nodes;
            for (const flatbuffers::String *s : *d->timestamp_logger_nodes()) {
              timestamp_logger_nodes.insert(s->string_view());
            }
            CHECK_EQ(static_cast<size_t>(timestamp_logger_nodes.size()),
                     d->timestamp_logger_nodes()->size())
                << ": Found duplicate timestamp_logger_nodes in "
                << CleanedChannelToString(c);
          }
        }

        // There is no good use case today for logging timestamps but not the
        // corresponding data.  Instead of plumbing through all of this on the
        // reader side, let'd just disallow it for now.
        if (c->logger() == LoggerConfig::NOT_LOGGED) {
          for (const Connection *d : *c->destination_nodes()) {
            CHECK(d->timestamp_logger() == LoggerConfig::NOT_LOGGED)
                << ": Logging timestamps without data is not supported.  If "
                   "you have a good use case, let's talk.  "
                << CleanedChannelToString(c);
          }
        }
      }
      CHECK_EQ(c->read_method() == ReadMethod::PIN, c->num_readers() != 0)
          << ": num_readers may be set if and only if read_method is PIN,"
             " if you want 0 readers do not set PIN: "
          << CleanedChannelToString(c);
    }
  }
}

void ValidateConfiguration(const Flatbuffer<Configuration> &config) {
  // No imports should be left.
  CHECK(!config.message().has_imports());

  ValidateUnmergedConfiguration(config);

  // A final config is also sorted.  Lookups in the config assume it is sorted.
  if (config.message().has_channels()) {
    const Channel *last_channel = nullptr;
    for (const Channel *c : *config.message().channels()) {
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
              CHECK(!connection->has_timestamp_logger_nodes())
                  << ": " << CleanedChannelToString(c);
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

void HandleReverseMaps(
    const flatbuffers::Vector<flatbuffers::Offset<aos::Map>> *maps,
    std::string_view type, const Node *node, std::set<std::string> *names) {
  for (const Map *map : *maps) {
    CHECK_NOTNULL(map);
    const Channel *const match = CHECK_NOTNULL(map->match());
    const Channel *const rename = CHECK_NOTNULL(map->rename());

    // Handle type specific maps.
    const flatbuffers::String *const match_type_string = match->type();
    if (match_type_string != nullptr &&
        match_type_string->string_view() != type) {
      continue;
    }

    // Now handle node specific maps.
    const flatbuffers::String *const match_source_node_string =
        match->source_node();
    if (node != nullptr && match_source_node_string != nullptr &&
        match_source_node_string->string_view() !=
            node->name()->string_view()) {
      continue;
    }

    const flatbuffers::String *const match_name_string = match->name();
    const flatbuffers::String *const rename_name_string = rename->name();
    if (match_name_string == nullptr || rename_name_string == nullptr) {
      continue;
    }

    const std::string rename_name = rename_name_string->str();
    const std::string_view match_name = match_name_string->string_view();

    std::set<std::string> possible_renames;

    // Check if the current name(s) could have been reached using the provided
    // rename.
    if (match_name.back() == '*') {
      for (const std::string &option : *names) {
        if (option.substr(0, rename_name.size()) == rename_name) {
          possible_renames.insert(
              absl::StrCat(match_name.substr(0, match_name.size() - 1),
                           option.substr(rename_name.size())));
        }
      }
      names->insert(possible_renames.begin(), possible_renames.end());
    } else if (names->count(rename_name) != 0) {
      names->insert(std::string(match_name));
    }
  }
}

}  // namespace

// Maps name for the provided maps.  Modifies name.
//
// This is called many times during startup, and it dereferences a lot of
// pointers. These combine to make it a performance hotspot during many tests
// under msan, so there is some optimizing around caching intermediates instead
// of dereferencing the pointer multiple times.
//
// Deliberately not in an anonymous namespace so that the log-reading code can
// reference it.
void HandleMaps(const flatbuffers::Vector<flatbuffers::Offset<aos::Map>> *maps,
                std::string *name, std::string_view type, const Node *node) {
  // For the same reason we merge configs in reverse order, we want to process
  // maps in reverse order.  That lets the outer config overwrite channels from
  // the inner configs.
  for (auto i = maps->rbegin(); i != maps->rend(); ++i) {
    const Channel *const match = i->match();
    if (!match) {
      continue;
    }
    const flatbuffers::String *const match_name_string = match->name();
    if (!match_name_string) {
      continue;
    }
    const Channel *const rename = i->rename();
    if (!rename) {
      continue;
    }
    const flatbuffers::String *const rename_name_string = rename->name();
    if (!rename_name_string) {
      continue;
    }

    // Handle normal maps (now that we know that match and rename are filled
    // out).
    const std::string_view match_name = match_name_string->string_view();
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
    const flatbuffers::String *const match_type_string = match->type();
    if (match_type_string && match_type_string->string_view() != type) {
      continue;
    }

    // Now handle node specific maps.
    const flatbuffers::String *const match_source_node_string =
        match->source_node();
    if (node && match_source_node_string &&
        match_source_node_string->string_view() !=
            node->name()->string_view()) {
      continue;
    }

    std::string new_name(rename_name_string->string_view());
    if (match_name.back() == '*') {
      new_name += std::string(name->substr(match_name.size() - 1));
    }
    VLOG(1) << "Renamed \"" << *name << "\" to \"" << new_name << "\"";
    *name = std::move(new_name);
  }
}

std::set<std::string> GetChannelAliases(const Configuration *config,
                                        std::string_view name,
                                        std::string_view type,
                                        const std::string_view application_name,
                                        const Node *node) {
  std::set<std::string> names{std::string(name)};
  if (config->has_maps()) {
    HandleReverseMaps(config->maps(), type, node, &names);
  }
  {
    const Application *application =
        GetApplication(config, node, application_name);
    if (application != nullptr && application->has_maps()) {
      HandleReverseMaps(application->maps(), type, node, &names);
    }
  }
  return names;
}

FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config) {
  MutableConfiguration unpacked_config;

  // The act of unpacking a config merges everything.
  UnpackConfiguration(&config.message(), &unpacked_config);

  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  fbb.Finish(PackConfiguration(unpacked_config, &fbb));

  FlatbufferDetachedBuffer<aos::Configuration> result(fbb.Release());

  ValidateConfiguration(result);

  return result;
}

std::optional<FlatbufferDetachedBuffer<Configuration>> MaybeReadConfig(
    const std::string_view path,
    const std::vector<std::string_view> &extra_import_paths) {
  // Add the executable directory to the search path.  That makes it so that
  // tools can be run from any directory without hard-coding an absolute path to
  // the config into all binaries.
  std::vector<std::string_view> extra_import_paths_with_exe =
      extra_import_paths;
  char proc_self_exec_buffer[PATH_MAX + 1];
  std::memset(proc_self_exec_buffer, 0, sizeof(proc_self_exec_buffer));
  ssize_t s = readlink("/proc/self/exe", proc_self_exec_buffer, PATH_MAX);
  if (s > 0) {
    // If the readlink call fails, the worst thing that happens is that we don't
    // automatically find the config next to the binary.  VLOG to make it easier
    // to debug.
    std::string_view proc_self_exec(proc_self_exec_buffer);

    extra_import_paths_with_exe.emplace_back(
        proc_self_exec.substr(0, proc_self_exec.rfind("/")));
  } else {
    VLOG(1) << "Failed to read /proc/self/exe";
  }

  // We only want to read a file once.  So track the visited files in a set.
  absl::btree_set<std::string> visited_paths;
  std::optional<FlatbufferDetachedBuffer<Configuration>> read_config =
      MaybeReadConfig(path, &visited_paths, extra_import_paths_with_exe);

  if (read_config == std::nullopt) {
    return read_config;
  }

  // If we only read one file, and it had a .bfbs extension, it has to be a
  // fully formatted config.  Do a quick verification and return it.
  if (visited_paths.size() == 1 && EndsWith(*visited_paths.begin(), ".bfbs")) {
    ValidateConfiguration(*read_config);
    return read_config;
  }

  return MergeConfiguration(*read_config);
}

FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path,
    const std::vector<std::string_view> &extra_import_paths) {
  auto optional_config = MaybeReadConfig(path, extra_import_paths);
  CHECK(optional_config) << "Could not read config. See above errors";
  return std::move(*optional_config);
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
      VLOG(1) << "Remapped from {\"name\": \"" << original_name
              << "\", \"type\": \"" << type << "\"}, to {\"name\": \"" << name
              << "\", \"type\": \"" << type
              << "\"}, but no channel by that name exists.";
    }
    return nullptr;
  }
}

size_t ChannelIndex(const Configuration *configuration,
                    const Channel *channel) {
  CHECK(configuration->channels() != nullptr) << ": No channels";

  const auto c = std::lower_bound(
      configuration->channels()->cbegin(), configuration->channels()->cend(),
      std::make_pair(channel->name()->string_view(),
                     channel->type()->string_view()),
      CompareChannels);
  CHECK(c != configuration->channels()->cend())
      << ": Channel pointer not found in configuration()->channels()";
  CHECK(*c == channel)
      << ": Channel pointer not found in configuration()->channels()";

  return std::distance(configuration->channels()->cbegin(), c);
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
  MutableConfiguration unpacked_config;
  UnpackConfiguration(&config.message(), &unpacked_config);

  // Now, add the schemas in so they will get packed.
  for (const aos::FlatbufferVector<reflection::Schema> &schema : schemas) {
    CHECK(schema.message().has_root_table());
    CHECK(schema.message().root_table()->has_name());
    unpacked_config.schemas.emplace(
        schema.message().root_table()->name()->string_view(),
        &schema.message());
  }

  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  fbb.Finish(PackConfiguration(unpacked_config, &fbb));

  return aos::FlatbufferDetachedBuffer<aos::Configuration>(fbb.Release());
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

bool IsNodeFromConfiguration(const Configuration *config, const Node *node) {
  if (config == nullptr) {
    return false;
  }

  // Check if is multinode
  if (MultiNode(config)) {
    if (node == nullptr) {
      return false;
    }
    for (const Node *node_from_config : *config->nodes()) {
      if (node_from_config == node) {
        return true;
      }
    }
    return false;
  } else {
    // nullptr is the node for all single node configurations. Return true so
    // this function can be used in the same way for single or multinode
    // configurations.
    return node == nullptr;
  }
}

std::string_view NodeName(const Configuration *config, size_t node_index) {
  if (!configuration::MultiNode(config)) {
    return "(singlenode)";
  }
  return config->nodes()->Get(node_index)->name()->string_view();
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
  if (!MultiNode(config)) {
    if (name.empty()) {
      return nullptr;
    }
    LOG(FATAL) << ": Asking for a named node from a single node configuration.";
  }
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

bool NodeHasTag(const Node *node, std::string_view tag) {
  if (node == nullptr) {
    return true;
  }

  if (!node->has_tags()) {
    return false;
  }

  const auto *const tags = node->tags();
  return std::find_if(tags->begin(), tags->end(),
                      [tag](const flatbuffers::String *candidate) {
                        return candidate->string_view() == tag;
                      }) != tags->end();
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

bool ChannelIsForwardedFromNode(const Channel *channel, const Node *node) {
  if (node == nullptr) {
    return false;
  }
  return ChannelIsSendableOnNode(channel, node) &&
         channel->has_destination_nodes() &&
         channel->destination_nodes()->size() > 0u;
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
      CHECK(channel->has_logger_nodes())
          << "Missing logger nodes on " << StrippedChannelToString(channel);
      CHECK_GT(channel->logger_nodes()->size(), 0u)
          << "Missing logger nodes on " << StrippedChannelToString(channel);

      if (channel->source_node()->string_view() == node_name) {
        return true;
      }

      [[fallthrough]];
    case LoggerConfig::REMOTE_LOGGER:
      CHECK(channel->has_logger_nodes())
          << "Missing logger nodes on " << StrippedChannelToString(channel);
      CHECK_GT(channel->logger_nodes()->size(), 0u)
          << "Missing logger nodes on " << StrippedChannelToString(channel);
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

size_t ConnectionCount(const Channel *channel) {
  if (!channel->has_destination_nodes()) {
    return 0;
  }
  return channel->destination_nodes()->size();
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
  return ConnectionDeliveryTimeIsLoggedOnNode(ConnectionToNode(channel, node),
                                              logger_node);
}

bool ConnectionDeliveryTimeIsLoggedOnNode(const Connection *connection,
                                          const Node *node) {
  if (connection == nullptr) {
    return false;
  }
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

bool ApplicationShouldStart(const Configuration *config, const Node *my_node,
                            const Application *application) {
  if (MultiNode(config)) {
    // Ok, we need
    CHECK(application->has_nodes());
    CHECK(my_node != nullptr);
    for (const flatbuffers::String *str : *application->nodes()) {
      if (str->string_view() == my_node->name()->string_view()) {
        return true;
      }
    }
    return false;
  } else {
    return true;
  }
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
      if (ApplicationShouldStart(config, my_node, *application_iterator)) {
        return *application_iterator;
      }
    }
  }
  return nullptr;
}

const Node *SourceNode(const Configuration *config, const Channel *channel) {
  if (!MultiNode(config)) {
    return nullptr;
  }
  return GetNode(config, channel->source_node()->string_view());
}

std::vector<size_t> SourceNodeIndex(const Configuration *config) {
  CHECK(config->has_channels());
  std::vector<size_t> result;
  result.resize(config->channels()->size(), 0u);
  if (MultiNode(config)) {
    for (size_t i = 0; i < config->channels()->size(); ++i) {
      result[i] = GetNodeIndex(
          config, config->channels()->Get(i)->source_node()->string_view());
    }
  }
  return result;
}

chrono::nanoseconds ChannelStorageDuration(const Configuration *config,
                                           const Channel *channel) {
  CHECK(channel != nullptr);
  if (channel->has_channel_storage_duration()) {
    return chrono::nanoseconds(channel->channel_storage_duration());
  }
  return chrono::nanoseconds(config->channel_storage_duration());
}

size_t QueueSize(const Configuration *config, const Channel *channel) {
  return QueueSize(channel->frequency(),
                   ChannelStorageDuration(config, channel));
}

size_t QueueSize(size_t frequency,
                 chrono::nanoseconds channel_storage_duration) {
  // Use integer arithmetic and round up at all cost.
  return static_cast<int>(
      (999'999'999 +
       static_cast<int64_t>(frequency) *
           static_cast<int64_t>(channel_storage_duration.count())) /
      static_cast<int64_t>(1'000'000'000));
}

int QueueScratchBufferSize(const Channel *channel) {
  return channel->num_readers() + channel->num_senders();
}

// Searches through configurations for schemas that include a certain type
const reflection::Schema *GetSchema(const Configuration *config,
                                    std::string_view schema_type) {
  if (config->has_channels()) {
    std::vector<flatbuffers::Offset<Channel>> channel_offsets;
    for (const Channel *c : *config->channels()) {
      if (schema_type == c->type()->string_view()) {
        return c->schema();
      }
    }
  }
  return nullptr;
}

// Copy schema reflection into detached flatbuffer
std::optional<FlatbufferDetachedBuffer<reflection::Schema>>
GetSchemaDetachedBuffer(const Configuration *config,
                        std::string_view schema_type) {
  const reflection::Schema *found_schema = GetSchema(config, schema_type);
  if (found_schema == nullptr) {
    return std::nullopt;
  }
  return RecursiveCopyFlatBuffer(found_schema);
}

aos::FlatbufferDetachedBuffer<Configuration> AddChannelToConfiguration(
    const Configuration *config, std::string_view name,
    aos::FlatbufferVector<reflection::Schema> schema, const aos::Node *node,
    ChannelT overrides) {
  overrides.name = name;
  CHECK(schema.message().has_root_table());
  overrides.type = schema.message().root_table()->name()->string_view();
  if (node != nullptr) {
    CHECK(node->has_name());
    overrides.source_node = node->name()->string_view();
  }

  // TODO(austin): Use MutableConfiguration to represent this transform more
  // efficiently.
  flatbuffers::FlatBufferBuilder fbb;
  // Don't populate fields from overrides that the user doesn't explicitly
  // override.
  fbb.ForceDefaults(false);
  const flatbuffers::Offset<Channel> channel_offset =
      Channel::Pack(fbb, &overrides);
  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset = fbb.CreateVector({channel_offset});
  Configuration::Builder config_builder(fbb);
  config_builder.add_channels(channels_offset);
  fbb.Finish(config_builder.Finish());
  FlatbufferDetachedBuffer<Configuration> new_channel_config = fbb.Release();
  new_channel_config = MergeConfiguration(new_channel_config, {schema});
  return MergeWithConfig(config, new_channel_config);
}

FlatbufferDetachedBuffer<Configuration> GetPartialConfiguration(
    const Configuration &configuration,
    std::function<bool(const Channel &)> should_include_channel) {
  // TODO(austin): Use MutableConfiguration to represent this better.
  //
  // create new_configuration1, containing everything except the `channels`
  // field.
  FlatbufferDetachedBuffer<Configuration> new_configuration1 =
      RecursiveCopyFlatBuffer(&configuration);
  new_configuration1.mutable_message()->clear_channels();

  // create new_configuration2, containing only the `channels` field.
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<Channel>> new_channels_vec;
  for (const auto &channel : *configuration.channels()) {
    CHECK_NOTNULL(channel);
    if (should_include_channel(*channel)) {
      new_channels_vec.push_back(RecursiveCopyFlatBuffer(channel, &fbb));
    }
  }
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      new_channels_offset = fbb.CreateVector(new_channels_vec);
  Configuration::Builder new_configuration2_builder(fbb);
  new_configuration2_builder.add_channels(new_channels_offset);
  fbb.Finish(new_configuration2_builder.Finish());
  FlatbufferDetachedBuffer<Configuration> new_configuration2 = fbb.Release();

  // Merge the configuration containing channels with the configuration
  // containing everything else, creating a complete configuration.
  const aos::FlatbufferDetachedBuffer<Configuration> raw_subset_configuration =
      MergeFlatBuffers(&new_configuration1.message(),
                       &new_configuration2.message());

  // Use MergeConfiguration to clean up redundant schemas.
  return configuration::MergeConfiguration(raw_subset_configuration);
}
}  // namespace configuration
}  // namespace aos
