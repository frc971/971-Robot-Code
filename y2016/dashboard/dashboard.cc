#include "y2016/dashboard/dashboard.h"

#include <chrono>
#include <complex>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "internal/Embedded.h"
#include "seasocks/Server.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"
#include "aos/realtime.h"
#include "aos/seasocks/seasocks_logger.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"
#include "y2016/queues/ball_detector_generated.h"
#include "y2016/vision/vision_generated.h"

namespace chrono = ::std::chrono;

namespace y2016 {
namespace dashboard {
namespace big_indicator {
constexpr int kBlack = 0;
constexpr int kBallIntaked = 1;
constexpr int kAiming = 2;
constexpr int kLockedOn = 3;
}  // namespace big_indicator

namespace superstructure_indicator {
constexpr int kBlack = 0;
constexpr int kNotZeroed = 1;
constexpr int kEstopped = 2;
}  // namespace superstructure_indicator

// Define the following if we want to use a local www directory and feed in
// dummy data.
//#define DASHBOARD_TESTING

// Define the following if we want to read from the vision queue, which has
// caused problems in the past when auto aiming that still need to be addressed.
//#define DASHBOARD_READ_VISION_QUEUE

DataCollector::DataCollector(::aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      vision_status_fetcher_(
          event_loop->MakeFetcher<::y2016::vision::VisionStatus>(
              "/superstructure")),
      ball_detector_fetcher_(
          event_loop->MakeFetcher<::y2016::sensors::BallDetector>(
              "/superstructure")),
      autonomous_mode_fetcher_(
          event_loop->MakeFetcher<::frc971::autonomous::AutonomousMode>(
              "/aos")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2016::control_loops::superstructure::Status>(
                  "/superstructure")),
      cur_raw_data_("no data"),
      sample_id_(0),
      measure_index_(0),
      overflow_id_(1) {}

void DataCollector::RunIteration() {
  ::aos::MutexLocker locker(&mutex_);
  measure_index_ = 0;

// Add recorded data here. /////
#ifdef DASHBOARD_TESTING
  // The following feeds data into the webserver when we do not have a process
  // feeding data to the queues.
  // To test, we are sending three streams holding randomly generated numbers.
  AddPoint("test", ::std::rand() % 4);
  AddPoint("test2", ::std::rand() % 3);
  AddPoint("test3", ::std::rand() % 3 - 1);
  (void)big_indicator::kBlack;
  (void)big_indicator::kBallIntaked;
  (void)big_indicator::kAiming;
  (void)big_indicator::kLockedOn;
  (void)superstructure_indicator::kBlack;
  (void)superstructure_indicator::kNotZeroed;
  (void)superstructure_indicator::kEstopped;
#else
  int big_indicator = big_indicator::kBlack;
  int superstructure_state_indicator = superstructure_indicator::kBlack;
  // We should never have a -1 here, so this is an indicator that somethings
  // gone wrong with reading the auto queue.
  int auto_mode_indicator = -1;

  autonomous_mode_fetcher_.Fetch();
  superstructure_status_fetcher_.Fetch();
  ball_detector_fetcher_.Fetch();
  vision_status_fetcher_.Fetch();

// Caused glitching with auto-aim at NASA, so be cautious with this until
// we find a good fix.
#ifdef DASHBOARD_READ_VISION_QUEUE
  if (vision_status_fetcher_.get() &&
      (vision_status_fetcher_->left_image_valid ||
       vision_status_fetcher_->right_image_valid)) {
    big_indicator = big_indicator::kAiming;
    if (::std::abs(vision_status_fetcher_->horizontal_angle) < 0.002) {
      big_indicator = big_indicator::kLockedOn;
    }
  }
#else
  (void)big_indicator::kAiming;
  (void)big_indicator::kLockedOn;
#endif

  // Ball detector comes after vision because we want to prioritize that
  // indication.
  if (ball_detector_fetcher_.get()) {
    // TODO(comran): Grab detected voltage from joystick_reader. Except this
    // value may not change, so it may be worth it to keep it as it is right
    // now.
    if (ball_detector_fetcher_->voltage() > 2.5) {
      big_indicator = big_indicator::kBallIntaked;
    }
  }

  if (superstructure_status_fetcher_.get()) {
    if (!superstructure_status_fetcher_->zeroed()) {
      superstructure_state_indicator = superstructure_indicator::kNotZeroed;
    }
    if (superstructure_status_fetcher_->estopped()) {
      superstructure_state_indicator = superstructure_indicator::kEstopped;
    }
  }

  if (autonomous_mode_fetcher_.get()) {
    auto_mode_indicator = autonomous_mode_fetcher_->mode();
  }

  AddPoint("big indicator", big_indicator);
  AddPoint("superstructure state indicator", superstructure_state_indicator);
  if (auto_mode_indicator != 15) {
    AddPoint("auto mode indicator", auto_mode_indicator);
  }
#endif

  // Get ready for next iteration. /////
  sample_id_++;
}

void DataCollector::AddPoint(const ::std::string &name, double value) {
  // Mutex should be locked when this method is called to synchronize packets.
  AOS_CHECK(mutex_.OwnedBySelf());

  size_t index = GetIndex(sample_id_);

  ItemDatapoint datapoint{value, ::aos::monotonic_clock::now()};
  if (measure_index_ >= sample_items_.size()) {
    // New item in our data table.
    ::std::vector<ItemDatapoint> datapoints;
    SampleItem item{name, datapoints};
    sample_items_.emplace_back(item);
  } else if (index >= sample_items_.at(measure_index_).datapoints.size()) {
    // New data point for an already existing item.
    sample_items_.at(measure_index_).datapoints.emplace_back(datapoint);
  } else {
    // Overwrite an already existing data point for an already existing item.
    sample_items_.at(measure_index_).datapoints.at(index) = datapoint;
  }

  measure_index_++;
}

::std::string DataCollector::Fetch(int32_t from_sample) {
  ::aos::MutexLocker locker(&mutex_);

  ::std::stringstream message;
  message.precision(10);

  // Send out the names of each item when requested by the client.
  // Example: *item_one_name,item_two_name,item_three_name
  if (from_sample == 0) {
    message << "*";  // Begin name packet.

    // Add comma-separated list of names.
    for (size_t cur_data_name = 0; cur_data_name < sample_items_.size();
         cur_data_name++) {
      if (cur_data_name > 0) {
        message << ",";
      }
      message << sample_items_.at(cur_data_name).name;
    }
    return message.str();
  }

  // Send out one sample containing the data.
  // Samples are split with dollar signs, info with percent signs, and
  // measurements with commas.
  // Example of data with two samples: $289%2803.13%10,67$290%2803.14%12,68

  // Note that we are ignoring the from_sample being sent to keep up with the
  // live data without worrying about client lag.
  int32_t cur_sample = sample_id_;
  int32_t adjusted_index = GetIndex(cur_sample);
  message << "$";  // Begin data packet.

  // Make sure we are not out of range.
  if (sample_items_.size() > 0) {
    if (static_cast<size_t>(adjusted_index) <
        sample_items_.at(0).datapoints.size()) {
      message << cur_sample << "%"
              << ::aos::time::DurationInSeconds(
                     sample_items_.at(0)
                         .datapoints.at(adjusted_index)
                         .time.time_since_epoch())
              << "%";  // Send time.
      // Add comma-separated list of data points.
      for (size_t cur_measure = 0; cur_measure < sample_items_.size();
           cur_measure++) {
        if (cur_measure > 0) {
          message << ",";
        }
        message << sample_items_.at(cur_measure)
                       .datapoints.at(adjusted_index)
                       .value;
      }
    }
  }

  return message.str();
}

size_t DataCollector::GetIndex(size_t sample_id) {
  return sample_id % overflow_id_;
}

void DataCollector::operator()() {
  ::aos::SetCurrentThreadName("DashboardData");

  ::aos::time::PhasedLoop phased_loop(chrono::milliseconds(100),
                                      event_loop_->monotonic_now(),
                                      chrono::seconds(0));
  while (run_) {
    phased_loop.SleepUntilNext();
    RunIteration();
  }
}

SocketHandler::SocketHandler(::aos::EventLoop *event_loop)
    : data_collector_(event_loop),
      data_collector_thread_(::std::ref(data_collector_)) {}

void SocketHandler::onConnect(seasocks::WebSocket *connection) {
  connections_.insert(connection);
  AOS_LOG(INFO, "Connected: %s : %s\n", connection->getRequestUri().c_str(),
          seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void SocketHandler::onData(seasocks::WebSocket *connection, const char *data) {
  int32_t from_sample = atoi(data);

  ::std::string send_data = data_collector_.Fetch(from_sample);
  connection->send(send_data.c_str());
}

void SocketHandler::onDisconnect(seasocks::WebSocket *connection) {
  connections_.erase(connection);
  AOS_LOG(INFO, "Disconnected: %s : %s\n", connection->getRequestUri().c_str(),
          seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void SocketHandler::Quit() {
  data_collector_.Quit();
  data_collector_thread_.join();
}

}  // namespace dashboard
}  // namespace y2016

int main(int, char *[]) {
  // Make sure to reference this to force the linker to include it.
  findEmbeddedContent("");

  ::aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  ::seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
      new ::aos::seasocks::SeasocksLogger(::seasocks::Logger::Level::Info)));
  ::y2016::dashboard::SocketHandler socket_handler(&event_loop);

  server.addWebSocketHandler(
      "/ws",
      ::std::shared_ptr<::y2016::dashboard::SocketHandler>(&socket_handler));
#ifdef DASHBOARD_TESTING
  server.serve("www", 1180);
#else
  // Absolute directory of www folder on the robot.
  server.serve("/home/admin/robot_code/www", 1180);
#endif

  socket_handler.Quit();

  ::aos::Cleanup();
  return 0;
}
