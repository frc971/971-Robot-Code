#ifndef AOS_CONFIGURATION_H_
#define AOS_CONFIGURATION_H_

#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string_view>

#include "aos/configuration_generated.h"  // IWYU pragma: export
#include "aos/flatbuffers.h"

namespace aos {

// Holds global configuration data. All of the functions are safe to call
// from wherever.
namespace configuration {

// Reads a json configuration.  This includes all imports and merges.  Note:
// duplicate imports will result in a CHECK.
FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path,
    const std::vector<std::string_view> &extra_import_paths = {});

// Sorts and merges entries in a config.
FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config);

// Adds schema definitions to a sorted and merged config from the provided
// schema list.
FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config,
    const std::vector<aos::FlatbufferVector<reflection::Schema>> &schemas);

// Merges a configuration json snippet into the provided configuration and
// returns the modified config.
FlatbufferDetachedBuffer<Configuration> MergeWithConfig(
    const Configuration *config, std::string_view json);
FlatbufferDetachedBuffer<Configuration> MergeWithConfig(
    const Configuration *config, const Flatbuffer<Configuration> &addition);

// Returns the resolved location for a name, type, and application name. Returns
// nullptr if none is found.
//
// If the application name is empty, it is ignored.  Maps are processed in
// reverse order, and application specific first.
const Channel *GetChannel(const Configuration *config,
                          const std::string_view name,
                          const std::string_view type,
                          const std::string_view application_name,
                          const Node *node, bool quiet = false);
inline const Channel *GetChannel(const Flatbuffer<Configuration> &config,
                                 const std::string_view name,
                                 const std::string_view type,
                                 const std::string_view application_name,
                                 const Node *node) {
  return GetChannel(&config.message(), name, type, application_name, node);
}
template <typename T>
inline const Channel *GetChannel(const Configuration *config,
                                 const std::string_view name,
                                 const std::string_view application_name,
                                 const Node *node) {
  return GetChannel(config, name, T::GetFullyQualifiedName(), application_name,
                    node);
}
// Convenience wrapper for getting a channel from a specified config if you
// already have the name/type in a Channel object--this is useful if you Channel
// object you have does not point to memory within config.
inline const Channel *GetChannel(const Configuration *config,
                                 const Channel *channel,
                                 const std::string_view application_name,
                                 const Node *node) {
  return GetChannel(config, channel->name()->string_view(),
                    channel->type()->string_view(), application_name, node);
}

// Returns the channel index (or dies) of channel in the provided config.
size_t ChannelIndex(const Configuration *config, const Channel *channel);

// Returns the Node out of the config with the matching name, or nullptr if it
// can't be found.
const Node *GetNode(const Configuration *config, std::string_view name);
const Node *GetNode(const Configuration *config, const Node *node);
// Returns the node with the provided index.  This works in both a single and
// multi-node world.
const Node *GetNode(const Configuration *config, size_t node_index);
// Returns a matching node, or nullptr if the provided node is nullptr and we
// are in a single node world.
const Node *GetNodeOrDie(const Configuration *config, const Node *node);
// Returns the Node out of the configuration which matches our hostname.
// CHECKs if it can't be found.
const Node *GetMyNode(const Configuration *config);
const Node *GetNodeFromHostname(const Configuration *config,
                                std::string_view name);

// Returns a vector of the nodes in the config.  (nullptr is considered the node
// in a single node world.)
std::vector<const Node *> GetNodes(const Configuration *config);

// Returns a vector of the nodes in the config with the provided tag.  If this
// is a single-node world, we assume that all tags match.
std::vector<const Node *> GetNodesWithTag(const Configuration *config,
                                          std::string_view tag);

// Returns the node index for a node.  Note: will be faster if node is a pointer
// to a node in config, but is not required.
int GetNodeIndex(const Configuration *config, const Node *node);
int GetNodeIndex(const Configuration *config, std::string_view name);

// Returns true if we are running in a multinode configuration.
bool MultiNode(const Configuration *config);

// Returns true if the provided channel is sendable on the provided node.
bool ChannelIsSendableOnNode(const Channel *channel, const Node *node);
// Returns true if the provided channel is able to be watched or fetched on the
// provided node.
bool ChannelIsReadableOnNode(const Channel *channel, const Node *node);

// Returns true if the message is supposed to be logged on this node.
bool ChannelMessageIsLoggedOnNode(const Channel *channel, const Node *node);

const Connection *ConnectionToNode(const Channel *channel, const Node *node);
// Returns true if the delivery timestamps are supposed to be logged on this
// node.
bool ConnectionDeliveryTimeIsLoggedOnNode(const Channel *channel,
                                          const Node *node,
                                          const Node *logger_node);
bool ConnectionDeliveryTimeIsLoggedOnNode(const Connection *connection,
                                          const Node *node);

// Prints a channel to json, but without the schema.
std::string CleanedChannelToString(const Channel *channel);
// Prints out a channel to json, but only with the name and type.
std::string StrippedChannelToString(const Channel *channel);

// Returns the node names that this node should be forwarding to.
std::vector<std::string_view> DestinationNodeNames(const Configuration *config,
                                                   const Node *my_node);

// Returns the node names that this node should be receiving messages from.
std::vector<std::string_view> SourceNodeNames(const Configuration *config,
                                              const Node *my_node);

// Returns the all nodes that are logging timestamps on our node.
std::vector<const Node *> TimestampNodes(const Configuration *config,
                                         const Node *my_node);

// TODO(austin): GetSchema<T>(const Flatbuffer<Configuration> &config);

}  // namespace configuration

// Compare and equality operators for Channel.  Note: these only check the name
// and type for equality.
bool operator<(const FlatbufferDetachedBuffer<Channel> &lhs,
               const FlatbufferDetachedBuffer<Channel> &rhs);
bool operator==(const FlatbufferDetachedBuffer<Channel> &lhs,
                const FlatbufferDetachedBuffer<Channel> &rhs);
}  // namespace aos

#endif  // AOS_CONFIGURATION_H_
