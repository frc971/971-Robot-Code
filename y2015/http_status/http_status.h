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

#include "aos/linux_code/init.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/mutex.h"

namespace frc971 {
namespace http_status {

// A class for storing data from DataCollector and packaging it as a custom
// message for the websocket.
// Samples are stored in a vector that wraps around at a certain point to avoid
// clogging up memory. These samples should be already on all clients before
// they are overwritten. To avoid losing samples, there must be a balance
// between the rate samples are being recorded at and the speed of the link
// between the robot and client.

class HTTPStatusMessage {
 public:
  HTTPStatusMessage();

  // Stores an individual measurement in the current sample.
  void AddMeasure(::std::string name, double value);

  // Starts a new sample that contains measurements for all the states at a
  // timestep, and lock mutex to synchronize measures.
  void NextSample();

  // Unlock mutex.
  void EndSample();

  // Method called by the websocket to get a JSON-packaged string containing,
  // at most, a constant number of samples, starting at "from_sample".
  // "from_sample" is a specific index for a sample that is not wrapped.
  ::std::string Fetch(size_t from_sample);

 private:
  // Returns the vector index of the sample given.
  // Since the vectors wrap, multiple sample_ids may refer to the same vector
  // index.
  int32_t GetIndex(int32_t sample_id);

  // Vectors of vectors to store samples at indexes determined by GetIndex.
  ::std::vector<::std::string> data_names_;
  ::std::vector<::std::vector<double>> data_values_;
  ::std::vector<::aos::time::Time> sample_times_;

  int32_t sample_id_;          // Last sample id used.
  int32_t measure_index_;      // Last measure index used.
  const int32_t overflow_id_;  // Vector wrapping size.
  // Number of samples to include in each JSON packet.
  const int32_t num_samples_per_packet_;

  // Mutex used to synchronize data.
  ::aos::Mutex mutex_;
};

class DataCollector {
 public:
  DataCollector();
  void RunIteration();
  ::std::string GetData(int32_t from);

  void operator()();  // Will be called by ::std::thread internally.
  void Quit() { run_ = false; }

 private:
  ::std::string cur_raw_data_;
  HTTPStatusMessage message_;
  ::std::atomic<bool> run_{true};
};

class SocketHandler : public seasocks::WebSocket::Handler {
 public:
  SocketHandler();
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

}  // namespace http_status
}  // namespace frc971
