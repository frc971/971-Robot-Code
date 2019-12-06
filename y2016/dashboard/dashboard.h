#ifndef Y2016_DASHBOARD_DASHBOARD_H_
#define Y2016_DASHBOARD_DASHBOARD_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

#include "seasocks/PageHandler.h"
#include "seasocks/PrintfLogger.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"

#include "aos/events/event_loop.h"
#include "aos/mutex/mutex.h"
#include "aos/time/time.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"
#include "y2016/queues/ball_detector_generated.h"
#include "y2016/vision/vision_generated.h"

namespace y2016 {
namespace dashboard {

// Dashboard is a webserver that opens a socket and stream data from the robot
// to the client. It is divided between the DataCollector, which polls
// RunIteration to determine what to send to the client, and an instance of a
// Seasocks server, which initiates a webserver on a port and opens a socket
// for streaming data.

// It is an adaption of http_status, which was a 2015 project
// that plotted live position data from the robot queues on a webpage for
// debugging.

class DataCollector {
 public:
  DataCollector(::aos::EventLoop *event_loop);
  void RunIteration();

  // Store a datapoint. In this case, we are reading data points to determine
  // what color to display on the webpage indicators. Traditionally, this would
  // be used to plot live data on a graph on the page.
  void AddPoint(const ::std::string &name, double value);

  // Method called by the websocket to get a JSON-packaged string containing,
  // at most, a constant number of samples, starting at from_sample.
  ::std::string Fetch(int32_t from_sample);

  void operator()();
  void Quit() { run_ = false; }

 private:
  // Returns a wrapped index based on the overflow size.
  size_t GetIndex(size_t sample_id);

  struct ItemDatapoint {
    double value;
    ::aos::monotonic_clock::time_point time;
  };

  struct SampleItem {
    ::std::string name;
    ::std::vector<ItemDatapoint> datapoints;
  };

  ::aos::EventLoop *event_loop_;

  ::aos::Fetcher<::y2016::vision::VisionStatus> vision_status_fetcher_;
  ::aos::Fetcher<::y2016::sensors::BallDetector> ball_detector_fetcher_;
  ::aos::Fetcher<::frc971::autonomous::AutonomousMode> autonomous_mode_fetcher_;
  ::aos::Fetcher<::y2016::control_loops::superstructure::Status>
      superstructure_status_fetcher_;

  // Storage vector that is written and overwritten with data in a FIFO fashion.
  ::std::vector<SampleItem> sample_items_;

  ::std::string cur_raw_data_;
  int32_t sample_id_;          // Last sample id used.
  size_t measure_index_;      // Last measure index used.
  const int32_t overflow_id_;  // Vector wrapping size.

  ::std::atomic<bool> run_{true};
  ::aos::Mutex mutex_;
};

class SocketHandler : public seasocks::WebSocket::Handler {
 public:
  SocketHandler(::aos::EventLoop *event_loop);
  void onConnect(seasocks::WebSocket* connection) override;
  void onData(seasocks::WebSocket* connection, const char* data) override;
  void onDisconnect(seasocks::WebSocket* connection) override;
  void Quit();

 private:
  ::std::set<seasocks::WebSocket*> connections_;
  DataCollector data_collector_;
  ::std::thread data_collector_thread_;
};

class SeasocksLogger : public seasocks::PrintfLogger {
 public:
  SeasocksLogger(Level min_level_to_log);
  void log(Level level, const char* message) override;
};

}  // namespace dashboard
}  // namespace y2016

#endif  // Y2016_DASHBOARD_DASHBOARD_H_
