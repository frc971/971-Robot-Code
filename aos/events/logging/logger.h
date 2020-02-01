#ifndef AOS_EVENTS_LOGGER_H_
#define AOS_EVENTS_LOGGER_H_

#include <deque>
#include <vector>
#include <string_view>

#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

// Logs all channels available in the event loop to disk every 100 ms.
// Start by logging one message per channel to capture any state and
// configuration that is sent rately on a channel and would affect execution.
class Logger {
 public:
  Logger(DetachedBufferWriter *writer, EventLoop *event_loop,
         std::chrono::milliseconds polling_period =
             std::chrono::milliseconds(100));

  // Rotates the log file with the new writer.  This writes out the header
  // again, but keeps going as if nothing else happened.
  void Rotate(DetachedBufferWriter *writer);

 private:
  void WriteHeader();

  void DoLogData();

  EventLoop *event_loop_;
  DetachedBufferWriter *writer_;

  // Structure to track both a fetcher, and if the data fetched has been
  // written.  We may want to delay writing data to disk so that we don't let
  // data get too far out of order when written to disk so we can avoid making
  // it too hard to sort when reading.
  struct FetcherStruct {
    std::unique_ptr<RawFetcher> fetcher;
    bool written = false;

    LogType log_type;
  };

  std::vector<FetcherStruct> fetchers_;
  TimerHandler *timer_handler_;

  // Period to poll the channels.
  const std::chrono::milliseconds polling_period_;

  // Last time that data was written for all channels to disk.
  monotonic_clock::time_point last_synchronized_time_;

  monotonic_clock::time_point monotonic_start_time_;
  realtime_clock::time_point realtime_start_time_;

  // Max size that the header has consumed.  This much extra data will be
  // reserved in the builder to avoid reallocating.
  size_t max_header_size_ = 0;
};

// Replays all the channels in the logfile to the event loop.
class LogReader {
 public:
  // If you want to supply a new configuration that will be used for replay
  // (e.g., to change message rates, or to populate an updated schema), then
  // pass it in here. It must provide all the channels that the original logged
  // config did.
  LogReader(std::string_view filename,
            const Configuration *replay_configuration = nullptr);
  LogReader(const std::vector<std::string> &filename,
            const Configuration *replay_configuration = nullptr);
  ~LogReader();

  // Registers all the callbacks to send the log file data out on an event loop
  // created in event_loop_factory.  This also updates time to be at the start
  // of the log file by running until the log file starts.
  // Note: the configuration used in the factory should be configuration()
  // below, but can be anything as long as the locations needed to send
  // everything are available.
  void Register(SimulatedEventLoopFactory *event_loop_factory);
  // Creates an SimulatedEventLoopFactory accessible via event_loop_factory(),
  // and then calls Register.
  void Register();
  // Registers callbacks for all the events after the log file starts.  This is
  // only useful when replaying live.
  void Register(EventLoop *event_loop);

  // Unregisters the senders. You only need to call this if you separately
  // supplied an event loop or event loop factory and the lifetimes are such
  // that they need to be explicitly destroyed before the LogReader destructor
  // gets called.
  void Deregister();

  // Returns the configuration from the log file.
  const Configuration *logged_configuration() const;
  // Returns the configuration being used for replay.
  const Configuration *configuration() const;

  const LogFileHeader *log_file_header() const {
    return sorted_message_reader_.log_file_header();
  }

  // Returns the node that this log file was created on.  This is a pointer to a
  // node in the nodes() list inside configuration().
  const Node *node() const;

  // Returns the starting timestamp for the log file.
  monotonic_clock::time_point monotonic_start_time();
  realtime_clock::time_point realtime_start_time();

  // Causes the logger to publish the provided channel on a different name so
  // that replayed applications can publish on the proper channel name without
  // interference. This operates on raw channel names, without any node or
  // application specific mappings.
  void RemapLoggedChannel(std::string_view name, std::string_view type,
                          std::string_view add_prefix = "/original");
  template <typename T>
  void RemapLoggedChannel(std::string_view name,
                          std::string_view add_prefix = "/original") {
    RemapLoggedChannel(name, T::GetFullyQualifiedName(), add_prefix);
  }

  SimulatedEventLoopFactory *event_loop_factory() {
    return event_loop_factory_;
  }

  // TODO(austin): Add the ability to re-publish the fetched messages.  Add 2
  // options, one which publishes them *now*, and another which publishes them
  // to the simulated event loop factory back in time where they actually
  // happened.

 private:
  // Queues at least max_out_of_order_duration_ messages into channels_.
  void QueueMessages();
  // Handle constructing a configuration with all the additional remapped
  // channels from calls to RemapLoggedChannel.
  void MakeRemappedConfig();

  // Log chunk reader.
  SortedMessageReader sorted_message_reader_;

  std::unique_ptr<FlatbufferDetachedBuffer<Configuration>>
      remapped_configuration_buffer_;

  std::vector<std::unique_ptr<RawSender>> channels_;

  std::unique_ptr<EventLoop> event_loop_unique_ptr_;
  NodeEventLoopFactory *node_event_loop_factory_ = nullptr;
  EventLoop *event_loop_ = nullptr;
  TimerHandler *timer_handler_;

  std::unique_ptr<SimulatedEventLoopFactory> event_loop_factory_unique_ptr_;
  SimulatedEventLoopFactory *event_loop_factory_ = nullptr;

  // Map of channel indices to new name. The channel index will be an index into
  // logged_configuration(), and the string key will be the name of the channel
  // to send on instead of the logged channel name.
  std::map<size_t, std::string> remapped_channels_;

  const Configuration *remapped_configuration_ = nullptr;
  const Configuration *replay_configuration_ = nullptr;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGER_H_
