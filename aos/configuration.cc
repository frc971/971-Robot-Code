#include "aos/configuration.h"

#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>

#include <set>
#include <string_view>

#include "absl/container/btree_set.h"
#include "aos/configuration_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/unique_malloc_ptr.h"
#include "aos/util/file.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(
    override_hostname, "",
    "If set, this forces the hostname of this node to be the provided "
    "hostname.");

namespace aos {

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
std::string_view ExtractFolder(
    const std::string_view filename) {
  auto last_slash_pos = filename.find_last_of("/\\");

  return last_slash_pos == std::string_view::npos
             ? std::string_view("./")
             : filename.substr(0, last_slash_pos + 1);
}

FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path, absl::btree_set<std::string> *visited_paths) {
  flatbuffers::DetachedBuffer buffer = JsonToFlatbuffer(
      util::ReadFileToStringOrDie(path), ConfigurationTypeTable());

  CHECK_GT(buffer.size(), 0u) << ": Failed to parse JSON file";

  FlatbufferDetachedBuffer<Configuration> config(std::move(buffer));
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

// Maps name for the provided maps.  Modifies name.
void HandleMaps(const flatbuffers::Vector<flatbuffers::Offset<aos::Map>> *maps,
                std::string_view *name, std::string_view type,
                const Node *node) {
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
    if (i->match()->name()->string_view() != *name) {
      continue;
    }

    // Handle type specific maps.
    if (i->match()->has_type() && i->match()->type()->string_view() != type) {
      continue;
    }

    if (node != nullptr && i->match()->has_source_node() &&
        i->match()->source_node()->string_view() !=
            node->name()->string_view()) {
      continue;
    }

    VLOG(1) << "Renamed \"" << *name << "\" to \""
            << i->rename()->name()->string_view() << "\"";
    *name = i->rename()->name()->string_view();
  }
}

}  // namespace

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

  // Now repeat this for the node list.
  absl::btree_set<FlatbufferDetachedBuffer<Node>> nodes;
  if (config.message().has_nodes()) {
    for (const Node *n : *config.message().nodes()) {
      if (!n->has_name()) {
        continue;
      }

      auto result = nodes.insert(CopyFlatBuffer(n));
      if (!result.second) {
        *result.first = MergeFlatBuffers(*result.first, CopyFlatBuffer(n));
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

  // Nodes
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Node>>>
      nodes_offset;
  {
    ::std::vector<flatbuffers::Offset<Node>> node_offsets;
    for (const FlatbufferDetachedBuffer<Node> &n : nodes) {
      node_offsets.emplace_back(CopyFlatBuffer<Node>(&n.message(), &fbb));
    }
    nodes_offset = fbb.CreateVector(node_offsets);
  }

  // And then build a Configuration with them all.
  ConfigurationBuilder configuration_builder(fbb);
  configuration_builder.add_channels(channels_offset);
  if (config.message().has_maps()) {
    configuration_builder.add_maps(maps_offset);
  }
  if (config.message().has_applications()) {
    configuration_builder.add_applications(applications_offset);
  }
  if (config.message().has_nodes()) {
    configuration_builder.add_nodes(nodes_offset);
  }

  fbb.Finish(configuration_builder.Finish());

  // Now, validate that if there is a node list, every channel has a source
  // node.
  FlatbufferDetachedBuffer<Configuration> result(fbb.Release());

  // Check that if there is a node list, all the source nodes are filled out and
  // valid, and all the destination nodes are valid (and not the source).  This
  // is a basic consistency check.
  if (result.message().has_nodes()) {
    for (const Channel *c : *config.message().channels()) {
      CHECK(c->has_source_node()) << ": Channel " << FlatbufferToJson(c)
                                  << " is missing \"source_node\"";
      CHECK(GetNode(&result.message(), c->source_node()->string_view()) !=
            nullptr)
          << ": Channel " << FlatbufferToJson(c)
          << " has an unknown \"source_node\"";

      if (c->has_destination_nodes()) {
        for (const Connection *connection : *c->destination_nodes()) {
          CHECK(connection->has_name());
          CHECK(GetNode(&result.message(), connection->name()->string_view()) !=
                nullptr)
              << ": Channel " << FlatbufferToJson(c)
              << " has an unknown \"destination_nodes\" "
              << connection->name()->string_view();

          switch (connection->timestamp_logger()) {
            case LoggerConfig::LOCAL_LOGGER:
            case LoggerConfig::NOT_LOGGED:
              CHECK(!connection->has_timestamp_logger_node());
              break;
            case LoggerConfig::REMOTE_LOGGER:
            case LoggerConfig::LOCAL_AND_REMOTE_LOGGER:
              CHECK(connection->has_timestamp_logger_node());
              CHECK(
                  GetNode(&result.message(),
                          connection->timestamp_logger_node()->string_view()) !=
                  nullptr)
                  << ": Channel " << FlatbufferToJson(c)
                  << " has an unknown \"timestamp_logger_node\""
                  << connection->name()->string_view();
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

  return result;
}

FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path) {
  // We only want to read a file once.  So track the visited files in a set.
  absl::btree_set<std::string> visited_paths;
  return MergeConfiguration(ReadConfig(path, &visited_paths));
}

const Channel *GetChannel(const Configuration *config, std::string_view name,
                          std::string_view type,
                          std::string_view application_name, const Node *node) {
  const std::string_view original_name = name;
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
        HandleMaps(application_iterator->maps(), &name, type, node);
      }
    }
  }

  // Now do global maps.
  if (config->has_maps()) {
    HandleMaps(config->maps(), &name, type, node);
  }

  if (original_name != name) {
    VLOG(1) << "Remapped to { \"name\": \"" << name << "\", \"type\": \""
            << type << "\" }";
  }

  // Then look for the channel.
  auto channel_iterator =
      std::lower_bound(config->channels()->cbegin(),
                       config->channels()->cend(),
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
    return nullptr;
  }
}

std::string CleanedChannelToString(const Channel *channel) {
  FlatbufferDetachedBuffer<Channel> cleaned_channel = CopyFlatBuffer(channel);
  cleaned_channel.mutable_message()->clear_schema();
  return FlatbufferToJson(cleaned_channel);
}

FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config,
    const std::vector<aos::FlatbufferString<reflection::Schema>> &schemas) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset;
  if (config.message().has_channels()) {
    std::vector<flatbuffers::Offset<Channel>> channel_offsets;
    for (const Channel *c : *config.message().channels()) {
      flatbuffers::FlatBufferBuilder channel_fbb;
      channel_fbb.ForceDefaults(1);

      // Search for a schema with a matching type.
      const aos::FlatbufferString<reflection::Schema> *found_schema = nullptr;
      for (const aos::FlatbufferString<reflection::Schema> &schema: schemas) {
        if (schema.message().root_table() != nullptr) {
          if (schema.message().root_table()->name()->string_view() ==
              c->type()->string_view()) {
            found_schema = &schema;
          }
        }
      }

      CHECK(found_schema != nullptr)
          << ": Failed to find schema for " << FlatbufferToJson(c);

      // The following is wasteful, but works.
      //
      // Copy it into a Channel object by creating an object with only the
      // schema populated and merge that into the current channel.
      flatbuffers::Offset<reflection::Schema> schema_offset =
          CopyFlatBuffer<reflection::Schema>(&found_schema->message(),
                                             &channel_fbb);
      Channel::Builder channel_builder(channel_fbb);
      channel_builder.add_schema(schema_offset);
      channel_fbb.Finish(channel_builder.Finish());
      FlatbufferDetachedBuffer<Channel> channel_schema_flatbuffer(
          channel_fbb.Release());

      FlatbufferDetachedBuffer<Channel> merged_channel(
          MergeFlatBuffers(channel_schema_flatbuffer, CopyFlatBuffer(c)));

      channel_offsets.emplace_back(
          CopyFlatBuffer<Channel>(&merged_channel.message(), &fbb));
    }
    channels_offset = fbb.CreateVector(channel_offsets);
  }

  // Copy the applications and maps unmodified.
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
      applications_offset;
  {
    ::std::vector<flatbuffers::Offset<Application>> applications_offsets;
    if (config.message().has_applications()) {
      for (const Application *a : *config.message().applications()) {
        applications_offsets.emplace_back(CopyFlatBuffer<Application>(a, &fbb));
      }
    }
    applications_offset = fbb.CreateVector(applications_offsets);
  }

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

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Node>>>
      nodes_offset;
  {
    ::std::vector<flatbuffers::Offset<Node>> node_offsets;
    if (config.message().has_nodes()) {
      for (const Node *n : *config.message().nodes()) {
        node_offsets.emplace_back(CopyFlatBuffer<Node>(n, &fbb));
      }
      nodes_offset = fbb.CreateVector(node_offsets);
    }
  }

  // Now insert everything else in unmodified.
  ConfigurationBuilder configuration_builder(fbb);
  if (config.message().has_channels()) {
    configuration_builder.add_channels(channels_offset);
  }
  if (config.message().has_maps()) {
    configuration_builder.add_maps(maps_offset);
  }
  if (config.message().has_applications()) {
    configuration_builder.add_applications(applications_offset);
  }
  if (config.message().has_nodes()) {
    configuration_builder.add_nodes(nodes_offset);
  }

  fbb.Finish(configuration_builder.Finish());
  return fbb.Release();
}

const Node *GetNodeFromHostname(const Configuration *config,
                                std::string_view hostname) {
  for (const Node *node : *config->nodes()) {
    if (node->hostname()->string_view() == hostname) {
      return node;
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

bool ChannelIsSendableOnNode(const Channel *channel, const Node *node) {
  if (node == nullptr) {
    return true;
  }
  return (channel->source_node()->string_view() == node->name()->string_view());
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
  switch(channel->logger()) {
    case LoggerConfig::LOCAL_LOGGER:
      if (node == nullptr) {
        // Single node world.  If there is a local logger, then we want to use
        // it.
        return true;
      }
      return channel->source_node()->string_view() ==
             node->name()->string_view();
    case LoggerConfig::REMOTE_LOGGER:
      CHECK(channel->has_logger_node());

      return channel->logger_node()->string_view() ==
             CHECK_NOTNULL(node)->name()->string_view();
    case LoggerConfig::LOCAL_AND_REMOTE_LOGGER:
      CHECK(channel->has_logger_node());

      if (channel->source_node()->string_view() ==
          CHECK_NOTNULL(node)->name()->string_view()) {
        return true;
      }
      if (channel->logger_node()->string_view() == node->name()->string_view()) {
        return true;
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
      CHECK(connection->has_timestamp_logger_node());
      if (connection->name()->string_view() == node->name()->string_view()) {
        return true;
      }

      if (connection->timestamp_logger_node()->string_view() ==
          node->name()->string_view()) {
        return true;
      }

      return false;
    case LoggerConfig::LOCAL_LOGGER:
      return connection->name()->string_view() == node->name()->string_view();
    case LoggerConfig::REMOTE_LOGGER:
      CHECK(connection->has_timestamp_logger_node());

      return connection->timestamp_logger_node()->string_view() ==
             node->name()->string_view();
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

}  // namespace configuration
}  // namespace aos
