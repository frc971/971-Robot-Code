#include "aos/events/logging/config_remapper.h"

#include <vector>

#include "absl/strings/escaping.h"
#include "flatbuffers/flatbuffers.h"

#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/remote_message_schema.h"
#include "aos/network/team_number.h"
#include "aos/network/timestamp_channel.h"

namespace aos {
using message_bridge::RemoteMessage;

namespace {
// Checks if the specified channel name/type exists in the config and, depending
// on the value of conflict_handling, calls conflict_handler or just dies.
template <typename F>
void CheckAndHandleRemapConflict(
    std::string_view new_name, std::string_view new_type,
    const Configuration *config,
    ConfigRemapper::RemapConflict conflict_handling, F conflict_handler) {
  const Channel *existing_channel =
      configuration::GetChannel(config, new_name, new_type, "", nullptr, true);
  if (existing_channel != nullptr) {
    switch (conflict_handling) {
      case ConfigRemapper::RemapConflict::kDisallow:
        LOG(FATAL)
            << "Channel "
            << configuration::StrippedChannelToString(existing_channel)
            << " is already used--you can't remap an original channel to it.";
        break;
      case ConfigRemapper::RemapConflict::kCascade:
        VLOG(1) << "Automatically remapping "
                << configuration::StrippedChannelToString(existing_channel)
                << " to avoid conflicts.";
        conflict_handler();
        break;
    }
  }
}
}  // namespace

namespace configuration {
// We don't really want to expose this publicly, but log reader doesn't really
// want to re-implement it.
void HandleMaps(const flatbuffers::Vector<flatbuffers::Offset<Map>> *maps,
                std::string *name, std::string_view type, const Node *node);
}  // namespace configuration

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
}

bool EqualsChannels(const Channel *c,
                    ::std::pair<std::string_view, std::string_view> p) {
  return c->name()->string_view() == p.first &&
         c->type()->string_view() == p.second;
}
// Copies the channel, removing the schema as we go.  If new_name is provided,
// it is used instead of the name inside the channel.  If new_type is provided,
// it is used instead of the type in the channel.
flatbuffers::Offset<Channel> CopyChannel(const Channel *c,
                                         std::string_view new_name,
                                         std::string_view new_type,
                                         flatbuffers::FlatBufferBuilder *fbb) {
  CHECK_EQ(Channel::MiniReflectTypeTable()->num_elems, 14u)
      << ": Merging logic needs to be updated when the number of channel "
         "fields changes.";

  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb->CreateSharedString(new_name.empty() ? c->name()->string_view()
                                               : new_name);
  flatbuffers::Offset<flatbuffers::String> type_offset =
      fbb->CreateSharedString(new_type.empty() ? c->type()->str() : new_type);
  flatbuffers::Offset<flatbuffers::String> source_node_offset =
      c->has_source_node() ? fbb->CreateSharedString(c->source_node()->str())
                           : 0;

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Connection>>>
      destination_nodes_offset =
          RecursiveCopyVectorTable(c->destination_nodes(), fbb);

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      logger_nodes_offset = CopyVectorSharedString(c->logger_nodes(), fbb);

  Channel::Builder channel_builder(*fbb);
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
  if (c->has_channel_storage_duration()) {
    channel_builder.add_channel_storage_duration(c->channel_storage_duration());
  }
  return channel_builder.Finish();
}

ConfigRemapper::ConfigRemapper(const Configuration *config,
                               const Configuration *replay_config,
                               const logger::ReplayChannels *replay_channels)
    : remapped_configuration_(config),
      original_configuration_(config),
      replay_configuration_(replay_config),
      replay_channels_(replay_channels) {
  MakeRemappedConfig();

  // If any remote timestamp channel was not marked NOT_LOGGED, then remap that
  // channel to avoid the redundant logged data. Also, this loop handles the
  // MessageHeader to RemoteMessae name change.
  // Note: This path is mainly for backwards compatibility reasons, and should
  // not be necessary for any new logs.
  for (const Node *node : configuration::GetNodes(original_configuration())) {
    message_bridge::ChannelTimestampFinder finder(original_configuration(),
                                                  "log_reader", node);

    absl::btree_set<std::string_view> remote_nodes;

    for (const Channel *channel : *original_configuration()->channels()) {
      if (!configuration::ChannelIsSendableOnNode(channel, node)) {
        continue;
      }
      if (!channel->has_destination_nodes()) {
        continue;
      }
      for (const Connection *connection : *channel->destination_nodes()) {
        if (configuration::ConnectionDeliveryTimeIsLoggedOnNode(connection,
                                                                node)) {
          // Start by seeing if the split timestamp channels are being used for
          // this message.
          const Channel *timestamp_channel = configuration::GetChannel(
              original_configuration(),
              finder.SplitChannelName(channel, connection),
              RemoteMessage::GetFullyQualifiedName(), "", node, true);

          if (timestamp_channel != nullptr) {
            // If for some reason a timestamp channel is not NOT_LOGGED (which
            // is unusual), then remap the channel so that the replayed channel
            // doesn't overlap with the special separate replay we do for
            // timestamps.
            if (timestamp_channel->logger() != LoggerConfig::NOT_LOGGED) {
              RemapOriginalChannel<RemoteMessage>(
                  timestamp_channel->name()->string_view(), node);
            }
            continue;
          }

          // Otherwise collect this one up as a node to look for a combined
          // channel from.  It is more efficient to compare nodes than channels.
          LOG(WARNING) << "Failed to find channel "
                       << finder.SplitChannelName(channel, connection)
                       << " on node " << FlatbufferToJson(node);
          remote_nodes.insert(connection->name()->string_view());
        }
      }
    }

    std::vector<const Node *> timestamp_logger_nodes =
        configuration::TimestampNodes(original_configuration(), node);
    for (const std::string_view remote_node : remote_nodes) {
      const std::string channel = finder.CombinedChannelName(remote_node);

      // See if the log file is an old log with logger::MessageHeader channels
      // in it, or a newer log with RemoteMessage.  If we find an older log,
      // rename the type too along with the name.
      if (HasChannel<logger::MessageHeader>(channel, node)) {
        CHECK(!HasChannel<RemoteMessage>(channel, node))
            << ": Can't have both a logger::MessageHeader and RemoteMessage "
               "remote "
               "timestamp channel.";
        // In theory, we should check NOT_LOGGED like RemoteMessage and be more
        // careful about updating the config, but there are fewer and fewer logs
        // with logger::MessageHeader remote messages, so it isn't worth the
        // effort.
        RemapOriginalChannel<logger::MessageHeader>(
            channel, node, "/original", "aos.message_bridge.RemoteMessage");
      } else {
        CHECK(HasChannel<RemoteMessage>(channel, node))
            << ": Failed to find {\"name\": \"" << channel << "\", \"type\": \""
            << RemoteMessage::GetFullyQualifiedName() << "\"} for node "
            << node->name()->string_view();
        // Only bother to remap if there's something on the channel.  We can
        // tell if the channel was marked NOT_LOGGED or not.  This makes the
        // config not change un-necesarily when we replay a log with NOT_LOGGED
        // messages.
        if (HasOriginalChannel<RemoteMessage>(channel, node)) {
          RemapOriginalChannel<RemoteMessage>(channel, node);
        }
      }
    }
  }
  if (replay_configuration_) {
    CHECK_EQ(configuration::MultiNode(remapped_configuration()),
             configuration::MultiNode(replay_configuration_))
        << ": Log file and replay config need to both be multi or single "
           "node.";
  }
}

ConfigRemapper::~ConfigRemapper() {
  // Zero out some buffers. It's easy to do use-after-frees on these, so make
  // it more obvious.
  if (remapped_configuration_buffer_) {
    remapped_configuration_buffer_->Wipe();
  }
}

const Configuration *ConfigRemapper::original_configuration() const {
  return original_configuration_;
}

const Configuration *ConfigRemapper::remapped_configuration() const {
  return remapped_configuration_;
}

void ConfigRemapper::set_configuration(const Configuration *configuration) {
  remapped_configuration_ = configuration;
}

std::vector<const Channel *> ConfigRemapper::RemappedChannels() const {
  std::vector<const Channel *> result;
  result.reserve(remapped_channels_.size());
  for (auto &pair : remapped_channels_) {
    const Channel *const original_channel =
        CHECK_NOTNULL(original_configuration()->channels()->Get(pair.first));

    auto channel_iterator = std::lower_bound(
        remapped_configuration_->channels()->cbegin(),
        remapped_configuration_->channels()->cend(),
        std::make_pair(std::string_view(pair.second.remapped_name),
                       original_channel->type()->string_view()),
        CompareChannels);

    CHECK(channel_iterator != remapped_configuration_->channels()->cend());
    CHECK(EqualsChannels(
        *channel_iterator,
        std::make_pair(std::string_view(pair.second.remapped_name),
                       original_channel->type()->string_view())));
    result.push_back(*channel_iterator);
  }
  return result;
}

const Channel *ConfigRemapper::RemapChannel(const EventLoop *event_loop,
                                            const Node *node,
                                            const Channel *channel) {
  std::string_view channel_name = channel->name()->string_view();
  std::string_view channel_type = channel->type()->string_view();
  const int channel_index =
      configuration::ChannelIndex(original_configuration(), channel);
  // If the channel is remapped, find the correct channel name to use.
  if (remapped_channels_.count(channel_index) > 0) {
    VLOG(3) << "Got remapped channel on "
            << configuration::CleanedChannelToString(channel);
    channel_name = remapped_channels_[channel_index].remapped_name;
  }

  VLOG(2) << "Going to remap channel " << channel_name << " " << channel_type;
  const Channel *remapped_channel = configuration::GetChannel(
      remapped_configuration(), channel_name, channel_type,
      event_loop ? event_loop->name() : "log_reader", node);

  CHECK(remapped_channel != nullptr)
      << ": Unable to send {\"name\": \"" << channel_name << "\", \"type\": \""
      << channel_type << "\"} because it is not in the provided configuration.";

  return remapped_channel;
}

void ConfigRemapper::RemapOriginalChannel(std::string_view name,
                                          std::string_view type,
                                          std::string_view add_prefix,
                                          std::string_view new_type,
                                          RemapConflict conflict_handling) {
  RemapOriginalChannel(name, type, nullptr, add_prefix, new_type,
                       conflict_handling);
}

void ConfigRemapper::RemapOriginalChannel(std::string_view name,
                                          std::string_view type,
                                          const Node *node,
                                          std::string_view add_prefix,
                                          std::string_view new_type,
                                          RemapConflict conflict_handling) {
  if (node != nullptr) {
    VLOG(1) << "Node is " << FlatbufferToJson(node);
  }
  if (replay_channels_ != nullptr) {
    CHECK(std::find(replay_channels_->begin(), replay_channels_->end(),
                    std::make_pair(std::string{name}, std::string{type})) !=
          replay_channels_->end())
        << "Attempted to remap channel " << name << " " << type
        << " which is not included in the replay channels passed to "
           "ConfigRemapper.";
  }
  const Channel *remapped_channel =
      configuration::GetChannel(original_configuration(), name, type, "", node);
  CHECK(remapped_channel != nullptr) << ": Failed to find {\"name\": \"" << name
                                     << "\", \"type\": \"" << type << "\"}";
  VLOG(1) << "Original {\"name\": \"" << name << "\", \"type\": \"" << type
          << "\"}";
  VLOG(1) << "Remapped "
          << configuration::StrippedChannelToString(remapped_channel);

  // We want to make /spray on node 0 go to /0/spray by snooping the maps.  And
  // we want it to degrade if the heuristics fail to just work.
  //
  // The easiest way to do this is going to be incredibly specific and verbose.
  // Look up /spray, to /0/spray.  Then, prefix the result with /original to get
  // /original/0/spray.  Then, create a map from /original/spray to
  // /original/0/spray for just the type we were asked for.
  if (name != remapped_channel->name()->string_view()) {
    MapT new_map;
    new_map.match = std::make_unique<ChannelT>();
    new_map.match->name = absl::StrCat(add_prefix, name);
    new_map.match->type = type;
    if (node != nullptr) {
      new_map.match->source_node = node->name()->str();
    }
    new_map.rename = std::make_unique<ChannelT>();
    new_map.rename->name =
        absl::StrCat(add_prefix, remapped_channel->name()->string_view());
    maps_.emplace_back(std::move(new_map));
  }

  // Then remap the original channel to the prefixed channel.
  const size_t channel_index =
      configuration::ChannelIndex(original_configuration(), remapped_channel);
  CHECK_EQ(0u, remapped_channels_.count(channel_index))
      << "Already remapped channel "
      << configuration::CleanedChannelToString(remapped_channel);

  RemappedChannel remapped_channel_struct;
  remapped_channel_struct.remapped_name =
      std::string(add_prefix) +
      std::string(remapped_channel->name()->string_view());
  remapped_channel_struct.new_type = new_type;
  const std::string_view remapped_type = new_type.empty() ? type : new_type;
  CheckAndHandleRemapConflict(
      remapped_channel_struct.remapped_name, remapped_type,
      remapped_configuration_, conflict_handling,
      [this, &remapped_channel_struct, remapped_type, node, add_prefix,
       conflict_handling]() {
        RemapOriginalChannel(remapped_channel_struct.remapped_name,
                             remapped_type, node, add_prefix, "",
                             conflict_handling);
      });
  remapped_channels_[channel_index] = std::move(remapped_channel_struct);
  MakeRemappedConfig();
}

void ConfigRemapper::RenameOriginalChannel(const std::string_view name,
                                           const std::string_view type,
                                           const std::string_view new_name,
                                           const std::vector<MapT> &add_maps) {
  RenameOriginalChannel(name, type, nullptr, new_name, add_maps);
}

void ConfigRemapper::RenameOriginalChannel(const std::string_view name,
                                           const std::string_view type,
                                           const Node *const node,
                                           const std::string_view new_name,
                                           const std::vector<MapT> &add_maps) {
  if (node != nullptr) {
    VLOG(1) << "Node is " << FlatbufferToJson(node);
  }
  // First find the channel and rename it.
  const Channel *remapped_channel =
      configuration::GetChannel(original_configuration(), name, type, "", node);
  CHECK(remapped_channel != nullptr) << ": Failed to find {\"name\": \"" << name
                                     << "\", \"type\": \"" << type << "\"}";
  VLOG(1) << "Original {\"name\": \"" << name << "\", \"type\": \"" << type
          << "\"}";
  VLOG(1) << "Remapped "
          << configuration::StrippedChannelToString(remapped_channel);

  const size_t channel_index =
      configuration::ChannelIndex(original_configuration(), remapped_channel);
  CHECK_EQ(0u, remapped_channels_.count(channel_index))
      << "Already remapped channel "
      << configuration::CleanedChannelToString(remapped_channel);

  RemappedChannel remapped_channel_struct;
  remapped_channel_struct.remapped_name = new_name;
  remapped_channel_struct.new_type.clear();
  remapped_channels_[channel_index] = std::move(remapped_channel_struct);

  // Then add any provided maps.
  for (const MapT &map : add_maps) {
    maps_.push_back(map);
  }

  // Finally rewrite the config.
  MakeRemappedConfig();
}

void ConfigRemapper::MakeRemappedConfig() {
  // If no remapping occurred and we are using the original config, then there
  // is nothing interesting to do here.
  if (remapped_channels_.empty() && replay_configuration_ == nullptr) {
    remapped_configuration_ = original_configuration();
    return;
  }
  // Config to copy Channel definitions from. Use the specified
  // replay_configuration_ if it has been provided.
  const Configuration *const base_config = replay_configuration_ == nullptr
                                               ? original_configuration()
                                               : replay_configuration_;

  // Create a config with all the channels, but un-sorted/merged.  Collect up
  // the schemas while we do this.  Call MergeConfiguration to sort everything,
  // and then merge it all in together.

  // This is the builder that we use for the config containing all the new
  // channels.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  std::vector<flatbuffers::Offset<Channel>> channel_offsets;

  CHECK_EQ(Channel::MiniReflectTypeTable()->num_elems, 14u)
      << ": Merging logic needs to be updated when the number of channel "
         "fields changes.";

  // List of schemas.
  std::map<std::string_view, FlatbufferVector<reflection::Schema>> schema_map;
  // Make sure our new RemoteMessage schema is in there for old logs without it.
  schema_map.insert(std::make_pair(
      message_bridge::RemoteMessage::GetFullyQualifiedName(),
      FlatbufferVector<reflection::Schema>(FlatbufferSpan<reflection::Schema>(
          message_bridge::RemoteMessageSchema()))));

  // Reconstruct the remapped channels.
  for (auto &pair : remapped_channels_) {
    const Channel *const c = CHECK_NOTNULL(configuration::GetChannel(
        base_config, original_configuration()->channels()->Get(pair.first), "",
        nullptr));
    channel_offsets.emplace_back(
        CopyChannel(c, pair.second.remapped_name, "", &fbb));

    if (c->has_destination_nodes()) {
      for (const Connection *connection : *c->destination_nodes()) {
        switch (connection->timestamp_logger()) {
          case LoggerConfig::LOCAL_LOGGER:
          case LoggerConfig::NOT_LOGGED:
            // There is no timestamp channel associated with this, so ignore it.
            break;

          case LoggerConfig::REMOTE_LOGGER:
          case LoggerConfig::LOCAL_AND_REMOTE_LOGGER:
            // We want to make a split timestamp channel regardless of what type
            // of log this used to be.  No sense propagating the single
            // timestamp channel.

            CHECK(connection->has_timestamp_logger_nodes());
            for (const flatbuffers::String *timestamp_logger_node :
                 *connection->timestamp_logger_nodes()) {
              const Node *node =
                  configuration::GetNode(original_configuration(),
                                         timestamp_logger_node->string_view());
              message_bridge::ChannelTimestampFinder finder(
                  original_configuration(), "log_reader", node);

              // We are assuming here that all the maps are setup correctly to
              // handle arbitrary timestamps.  Apply the maps for this node to
              // see what name this ends up with.
              std::string name = finder.SplitChannelName(
                  pair.second.remapped_name, c->type()->str(), connection);
              std::string unmapped_name = name;
              configuration::HandleMaps(original_configuration()->maps(), &name,
                                        "aos.message_bridge.RemoteMessage",
                                        node);
              CHECK_NE(name, unmapped_name)
                  << ": Remote timestamp channel was not remapped, this is "
                     "very fishy";
              flatbuffers::Offset<flatbuffers::String> channel_name_offset =
                  fbb.CreateString(name);
              flatbuffers::Offset<flatbuffers::String> channel_type_offset =
                  fbb.CreateString("aos.message_bridge.RemoteMessage");
              flatbuffers::Offset<flatbuffers::String> source_node_offset =
                  fbb.CreateString(timestamp_logger_node->string_view());

              // Now, build a channel.  Don't log it, 2 senders, and match the
              // source frequency.
              Channel::Builder channel_builder(fbb);
              channel_builder.add_name(channel_name_offset);
              channel_builder.add_type(channel_type_offset);
              channel_builder.add_source_node(source_node_offset);
              channel_builder.add_logger(LoggerConfig::NOT_LOGGED);
              channel_builder.add_num_senders(2);
              if (c->has_frequency()) {
                channel_builder.add_frequency(c->frequency());
              }
              if (c->has_channel_storage_duration()) {
                channel_builder.add_channel_storage_duration(
                    c->channel_storage_duration());
              }
              channel_offsets.emplace_back(channel_builder.Finish());
            }
            break;
        }
      }
    }
  }

  // Now reconstruct the original channels, translating types as needed
  for (const Channel *c : *base_config->channels()) {
    // Search for a mapping channel.
    std::string_view new_type = "";
    for (auto &pair : remapped_channels_) {
      const Channel *const remapped_channel =
          original_configuration()->channels()->Get(pair.first);
      if (remapped_channel->name()->string_view() == c->name()->string_view() &&
          remapped_channel->type()->string_view() == c->type()->string_view()) {
        new_type = pair.second.new_type;
        break;
      }
    }

    // Copy everything over.
    channel_offsets.emplace_back(CopyChannel(c, "", new_type, &fbb));

    // Add the schema if it doesn't exist.
    if (schema_map.find(c->type()->string_view()) == schema_map.end()) {
      CHECK(c->has_schema());
      schema_map.insert(std::make_pair(c->type()->string_view(),
                                       RecursiveCopyFlatBuffer(c->schema())));
    }
  }

  // The MergeConfiguration API takes a vector, not a map.  Convert.
  std::vector<FlatbufferVector<reflection::Schema>> schemas;
  while (!schema_map.empty()) {
    schemas.emplace_back(std::move(schema_map.begin()->second));
    schema_map.erase(schema_map.begin());
  }

  // Create the Configuration containing the new channels that we want to add.
  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset =
          channel_offsets.empty() ? 0 : fbb.CreateVector(channel_offsets);

  // Copy over the old maps.
  std::vector<flatbuffers::Offset<Map>> map_offsets;
  if (base_config->maps()) {
    for (const Map *map : *base_config->maps()) {
      map_offsets.emplace_back(RecursiveCopyFlatBuffer(map, &fbb));
    }
  }

  // Now create the new maps.  These are second so they take effect first.
  for (const MapT &map : maps_) {
    CHECK(!map.match->name.empty());
    const flatbuffers::Offset<flatbuffers::String> match_name_offset =
        fbb.CreateString(map.match->name);
    flatbuffers::Offset<flatbuffers::String> match_type_offset;
    if (!map.match->type.empty()) {
      match_type_offset = fbb.CreateString(map.match->type);
    }
    flatbuffers::Offset<flatbuffers::String> match_source_node_offset;
    if (!map.match->source_node.empty()) {
      match_source_node_offset = fbb.CreateString(map.match->source_node);
    }
    CHECK(!map.rename->name.empty());
    const flatbuffers::Offset<flatbuffers::String> rename_name_offset =
        fbb.CreateString(map.rename->name);
    Channel::Builder match_builder(fbb);
    match_builder.add_name(match_name_offset);
    if (!match_type_offset.IsNull()) {
      match_builder.add_type(match_type_offset);
    }
    if (!match_source_node_offset.IsNull()) {
      match_builder.add_source_node(match_source_node_offset);
    }
    const flatbuffers::Offset<Channel> match_offset = match_builder.Finish();

    Channel::Builder rename_builder(fbb);
    rename_builder.add_name(rename_name_offset);
    const flatbuffers::Offset<Channel> rename_offset = rename_builder.Finish();

    Map::Builder map_builder(fbb);
    map_builder.add_match(match_offset);
    map_builder.add_rename(rename_offset);
    map_offsets.emplace_back(map_builder.Finish());
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Map>>>
      maps_offsets = map_offsets.empty() ? 0 : fbb.CreateVector(map_offsets);

  // And copy everything else over.
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Node>>>
      nodes_offset = RecursiveCopyVectorTable(base_config->nodes(), &fbb);

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
      applications_offset =
          RecursiveCopyVectorTable(base_config->applications(), &fbb);

  // Now insert everything else in unmodified.
  ConfigurationBuilder configuration_builder(fbb);
  if (!channels_offset.IsNull()) {
    configuration_builder.add_channels(channels_offset);
  }
  if (!maps_offsets.IsNull()) {
    configuration_builder.add_maps(maps_offsets);
  }
  if (!nodes_offset.IsNull()) {
    configuration_builder.add_nodes(nodes_offset);
  }
  if (!applications_offset.IsNull()) {
    configuration_builder.add_applications(applications_offset);
  }

  if (base_config->has_channel_storage_duration()) {
    configuration_builder.add_channel_storage_duration(
        base_config->channel_storage_duration());
  }

  CHECK_EQ(Configuration::MiniReflectTypeTable()->num_elems, 6u)
      << ": Merging logic needs to be updated when the number of configuration "
         "fields changes.";

  fbb.Finish(configuration_builder.Finish());

  // Clean it up and return it!  By using MergeConfiguration here, we'll
  // actually get a deduplicated config for free too.
  FlatbufferDetachedBuffer<Configuration> new_merged_config =
      configuration::MergeConfiguration(
          FlatbufferDetachedBuffer<Configuration>(fbb.Release()));

  remapped_configuration_buffer_ =
      std::make_unique<FlatbufferDetachedBuffer<Configuration>>(
          configuration::MergeConfiguration(new_merged_config, schemas));

  remapped_configuration_ = &remapped_configuration_buffer_->message();

  // TODO(austin): Lazily re-build to save CPU?
}

}  // namespace aos
