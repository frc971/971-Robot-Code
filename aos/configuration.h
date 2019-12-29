#ifndef AOS_CONFIGURATION_H_
#define AOS_CONFIGURATION_H_

#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string_view>

#include "aos/configuration_generated.h"
#include "aos/flatbuffers.h"

namespace aos {

// Holds global configuration data. All of the functions are safe to call
// from wherever.
namespace configuration {

// Reads a json configuration.  This includes all imports and merges.  Note:
// duplicate imports will result in a CHECK.
FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path);

// Sorts and merges entries in a config.
FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config);

// Adds schema definitions to a sorted and merged config from the provided
// schema list.
FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config,
    const std::vector<aos::FlatbufferString<reflection::Schema>> &schemas);

// Returns the resolved location for a name, type, and application name. Returns
// nullptr if none is found.
//
// If the application name is empty, it is ignored.  Maps are processed in
// reverse order, and application specific first.
const Channel *GetChannel(const Configuration *config,
                          const std::string_view name,
                          const std::string_view type,
                          const std::string_view application_name,
                          const Node *node);
inline const Channel *GetChannel(const Flatbuffer<Configuration> &config,
                                 const std::string_view name,
                                 const std::string_view type,
                                 const std::string_view application_name,
                                 const Node *node) {
  return GetChannel(&config.message(), name, type, application_name, node);
}

// Returns the Node out of the config with the matching name, or nullptr if it
// can't be found.
const Node *GetNode(const Configuration *config, std::string_view name);
// Returns the Node out of the configuration which matches our hostname.
// CHECKs if it can't be found.
const Node *GetMyNode(const Configuration *config);
const Node *GetNodeFromHostname(const Configuration *config,
                                std::string_view name);

// Returns true if the provided channel is sendable on the provided node.
bool ChannelIsSendableOnNode(const Channel *channel, const Node *node);
// Returns true if the provided channel is able to be watched or fetched on the
// provided node.
bool ChannelIsReadableOnNode(const Channel *channel, const Node *node);

// Prints a channel to json, but without the schema.
std::string CleanedChannelToString(const Channel *channel);

// TODO(austin): GetSchema<T>(const Flatbuffer<Configuration> &config);

// Returns the "root directory" for this run. Under linux, this is the
// directory where the executable is located (from /proc/self/exe)
// The return value will always be to a static string, so no freeing is
// necessary.
const char *GetRootDirectory();
// Returns the directory where logs get written. Relative to GetRootDirectory().
// The return value will always be to a static string, so no freeing is
// necessary.
const char *GetLoggingDirectory();

}  // namespace configuration

// Compare and equality operators for Channel.  Note: these only check the name
// and type for equality.
bool operator<(const FlatbufferDetachedBuffer<Channel> &lhs,
               const FlatbufferDetachedBuffer<Channel> &rhs);
bool operator==(const FlatbufferDetachedBuffer<Channel> &lhs,
                const FlatbufferDetachedBuffer<Channel> &rhs);
}  // namespace aos

#endif  // AOS_CONFIGURATION_H_
