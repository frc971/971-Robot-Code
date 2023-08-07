#ifndef DOCUMENTATION_AOS_EXAMPLES_CLOCK_OFFSET_READER_H_
#define DOCUMENTATION_AOS_EXAMPLES_CLOCK_OFFSET_READER_H_
#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "documentation/aos/examples/sensor_data_generated.h"

namespace examples {

// This class is a sample that is shown in the markdown documentation.
// If it needs to get updated, the sample should get updated as well.
// TODO(james): Get a convenient way to directly include portions of files in
// markdown so that we don't just manually copy-and-paste the code between
// spots.
class SensorAgeReader {
 public:
  SensorAgeReader(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        clock_offset_fetcher_(
            event_loop->MakeFetcher<aos::message_bridge::ServerStatistics>(
                "/aos")) {
    event_loop_->MakeWatcher(
        "/input", [this](const SensorData &msg) { HandleSensorData(msg); });
  }

  void HandleSensorData(const SensorData &msg) {
    std::chrono::nanoseconds monotonic_offset{0};
    clock_offset_fetcher_.Fetch();
    if (clock_offset_fetcher_.get() != nullptr) {
      for (const auto connection : *clock_offset_fetcher_->connections()) {
        if (connection->has_node() && connection->node()->has_name() &&
            connection->node()->name()->string_view() == "sensor") {
          if (connection->has_monotonic_offset()) {
            monotonic_offset =
                std::chrono::nanoseconds(connection->monotonic_offset());
          } else {
            // If we don't have a monotonic offset, that means we aren't
            // connected, in which case we should just exit early.
            // The ServerStatistics message will always populate statuses for
            // every node, so we don't have to worry about missing the "sensor"
            // node (although it can be good practice to check that the node you
            // are looking for actually exists, to protect against programming
            // errors).
            LOG(WARNING) << "Message bridge disconnected.";
            return;
          }
          break;
        }
      }
    } else {
      LOG(WARNING) << "No message bridge status available.";
      return;
    }
    const aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
    // The monotonic_remote_time will be the time that the message was sent on
    // the source node; by offsetting it by the monotonic_offset, we should get
    // a reasonable estimate of when it was sent. This does not account for any
    // delays between the sensor reading and when it actually got sent.
    const aos::monotonic_clock::time_point send_time(
        event_loop_->context().monotonic_remote_time - monotonic_offset);
    // Many sensors may include some sort of hardware timestamp indicating when
    // the measurement was taken, which is likely before the sent time. This can
    // be populated as a data field inside of the message, and if it is using
    // the same monotonic clock as AOS is then we can do the same offset
    // computation, but get a timestamp for when the data was actually captured.
    const aos::monotonic_clock::time_point capture_time(
        std::chrono::nanoseconds(msg.hardware_capture_time_ns()) -
        monotonic_offset);
    LOG(INFO) << "The sensor data was sent "
              << aos::time::DurationInSeconds(now - send_time)
              << " seconds ago.";
    LOG(INFO) << "The sensor data was read off of the hardware "
              << aos::time::DurationInSeconds(now - capture_time)
              << " seconds ago.";
  }

  aos::EventLoop *event_loop_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> clock_offset_fetcher_;
};
}  // namespace examples
#endif  // DOCUMENTATION_AOS_EXAMPLES_CLOCK_OFFSET_READER_H_
