#ifndef AOS_EVENTS_LOGGING_CONFIG_REMAPPER_H_
#define AOS_EVENTS_LOGGING_CONFIG_REMAPPER_H_

#include <map>
#include <string_view>
#include <tuple>
#include <vector>

#include "flatbuffers/flatbuffers.h"

#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/logging/replay_channels.h"

namespace aos {

// This class is used for remapping and renaming channels with the passed in
// configuration to the constructor. Typically, the templated versions of
// RemapOriginalChannel and RenameOriginalChannel are the main functions to use
// for type safety. After remapping and renaming, the remapped configuration can
// be accessed through remapped_configuration(), and the original configuration
// that is not mutated can be accessed through original_configuration.
//
// This class assumes no ownership over any pointers provided to it.
//
// Timestamp channels are automatically remapped on construction
//
// Note: This class does not need logfiles to function unlike LogReader. This
// logic originally lived in LogReader and was refactored out into this class.
// The same API for remapping and renaming still exists in LogReader which now
// just passes along the args to this class.
class ConfigRemapper {
 public:
  ConfigRemapper(const Configuration *config,
                 const Configuration *replay_config = nullptr,
                 const logger::ReplayChannels *replay_channels = nullptr);
  ~ConfigRemapper();

  // Map of channel indices to new name. The channel index will be an index into
  // original_configuration(), and the string key will be the name of the
  // channel to send on instead of the orignal channel name.
  struct RemappedChannel {
    std::string remapped_name;
    std::string new_type;
  };

  // Enum to use for indicating how RemapOriginalChannel behaves when there is
  // already a channel with the remapped name (e.g., as may happen when
  // replaying a logfile that was itself generated from replay).
  enum class RemapConflict {
    // LOG(FATAL) on conflicts in remappings.
    kDisallow,
    // If we run into a conflict, attempt to remap the channel we would be
    // overriding (and continue to do so if remapping *that* channel also
    // generates a conflict).
    // This will mean that if we repeatedly replay a log, we will end up
    // stacking more and more /original's on the start of the oldest version
    // of the channels.
    kCascade
  };

  // Remaps a channel from the original configuration passed to the constructor
  // to the given one. This operates on raw channel names, without any node or
  // application specific mappings.
  void RemapOriginalChannel(
      std::string_view name, std::string_view type,
      std::string_view add_prefix = "/original", std::string_view new_type = "",
      RemapConflict conflict_handling = RemapConflict::kCascade);
  template <typename T>
  void RemapOriginalChannel(
      std::string_view name, std::string_view add_prefix = "/original",
      std::string_view new_type = "",
      RemapConflict conflict_handling = RemapConflict::kCascade) {
    RemapOriginalChannel(name, T::GetFullyQualifiedName(), add_prefix, new_type,
                         conflict_handling);
  }

  // Remaps the provided channel, though this respects node mappings, and
  // preserves them too.  This makes it so if /aos -> /pi1/aos on one node,
  // /original/aos -> /original/pi1/aos on the same node after renaming, just
  // like you would hope.  If new_type is not empty, the new channel will use
  // the provided type instead.  This allows for renaming messages.
  //
  // TODO(austin): If you have 2 nodes remapping something to the same channel,
  // this doesn't handle that.  No use cases exist yet for that, so it isn't
  // being done yet.
  void RemapOriginalChannel(
      std::string_view name, std::string_view type, const Node *node,
      std::string_view add_prefix = "/original", std::string_view new_type = "",
      RemapConflict conflict_handling = RemapConflict::kCascade);

  template <typename T>
  void RemapOriginalChannel(
      std::string_view name, const Node *node,
      std::string_view add_prefix = "/original", std::string_view new_type = "",
      RemapConflict conflict_handling = RemapConflict::kCascade) {
    RemapOriginalChannel(name, T::GetFullyQualifiedName(), node, add_prefix,
                         new_type, conflict_handling);
  }

  // Similar to RemapOriginalChannel(), but lets you specify a name for the new
  // channel without constraints. By default, this will not add any maps for the
  // new channel. Use add_maps to specify any maps you'd like added.
  void RenameOriginalChannel(std::string_view name, std::string_view type,
                             std::string_view new_name,
                             const std::vector<MapT> &add_maps = {});
  template <typename T>
  void RenameOriginalChannel(std::string_view name, std::string_view new_name,
                             const std::vector<MapT> &add_maps = {}) {
    RenameOriginalChannel(name, T::GetFullyQualifiedName(), new_name, add_maps);
  }
  // The following overloads are more suitable for multi-node configurations,
  // and let you rename a channel on a specific node.
  void RenameOriginalChannel(std::string_view name, std::string_view type,
                             const Node *node, std::string_view new_name,
                             const std::vector<MapT> &add_maps = {});
  template <typename T>
  void RenameOriginalChannel(std::string_view name, const Node *node,
                             std::string_view new_name,
                             const std::vector<MapT> &add_maps = {}) {
    RenameOriginalChannel(name, T::GetFullyQualifiedName(), node, new_name,
                          add_maps);
  }

  template <typename T>
  bool HasChannel(std::string_view name, const Node *node = nullptr) {
    return HasChannel(name, T::GetFullyQualifiedName(), node);
  }
  bool HasChannel(std::string_view name, std::string_view type,
                  const Node *node) {
    return configuration::GetChannel(original_configuration(), name, type, "",
                                     node, true) != nullptr;
  }

  // Returns true if the channel exists on the node and was in the original
  // config
  template <typename T>
  bool HasOriginalChannel(std::string_view name, const Node *node = nullptr) {
    const Channel *channel =
        configuration::GetChannel(original_configuration(), name,
                                  T::GetFullyQualifiedName(), "", node, true);
    if (channel == nullptr) return false;
    return channel->logger() != LoggerConfig::NOT_LOGGED;
  }

  template <typename T>
  void MaybeRemapOriginalChannel(std::string_view name,
                                 const Node *node = nullptr) {
    if (HasChannel<T>(name, node)) {
      RemapOriginalChannel<T>(name, node);
    }
  }
  template <typename T>
  void MaybeRenameOriginalChannel(std::string_view name, const Node *node,
                                  std::string_view new_name,
                                  const std::vector<MapT> &add_maps = {}) {
    if (HasChannel<T>(name, node)) {
      RenameOriginalChannel<T>(name, node, new_name, add_maps);
    }
  }

  const Channel *RemapChannel(const EventLoop *event_loop, const Node *node,
                              const Channel *channel);
  // Returns a list of all the original channels from remapping.
  std::vector<const Channel *> RemappedChannels() const;

  void set_configuration(const Configuration *configuration);

  // Returns the configuration that was originally passed to the constructor.
  // This class does not own this pointer.
  const Configuration *original_configuration() const;

  // Returns the configuration that contains the remapping and renamings done on
  // the original configuration. The pointer is invalidated whenever
  // RemapOriginalChannel is called.
  const Configuration *remapped_configuration() const;

 private:
  // Handle constructing a configuration with all the additional remapped
  // channels from calls to RemapOriginalChannel.
  void MakeRemappedConfig();

  std::map<size_t, RemappedChannel> remapped_channels_;
  std::vector<MapT> maps_;
  std::unique_ptr<FlatbufferDetachedBuffer<Configuration>>
      remapped_configuration_buffer_;

  const Configuration *remapped_configuration_ = nullptr;
  const Configuration *original_configuration_ = nullptr;
  const Configuration *replay_configuration_ = nullptr;

  const logger::ReplayChannels *replay_channels_ = nullptr;
};  // class ConfigRemapper

}  // namespace aos
#endif  // AOS_EVENTS_LOGGING_CONFIG_REMAPPER_H_
