#include "y2015/http_status/http_status.h"

#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "seasocks/Server.h"

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/mutex.h"

#include "y2015/control_loops/claw/claw.q.h"
#include "y2015/control_loops/fridge/fridge.q.h"

#include "y2015/http_status/embedded.h"

namespace frc971 {
namespace http_status {

// TODO(comran): Make some of these separate libraries & document them better.

HTTPStatusMessage::HTTPStatusMessage()
    : sample_id_(0),
      measure_index_(0),
      overflow_id_(200),
      num_samples_per_packet_(50) {}

void HTTPStatusMessage::NextSample() {
  int32_t adjusted_index = GetIndex(sample_id_);

  ::aos::time::Time time_now = ::aos::time::Time::Now();

  if (sample_id_ < overflow_id_) {
    sample_times_.emplace_back(time_now);
    data_values_.emplace_back(::std::vector<double>());
  } else {
    sample_times_[adjusted_index] = time_now;
  }

  sample_id_++;
  measure_index_ = 0;

  CHECK(!mutex_.Lock());  // Lock the mutex so measures can be added.
}

void HTTPStatusMessage::EndSample() { mutex_.Unlock(); }

int32_t HTTPStatusMessage::GetIndex(int32_t sample_id) {
  return sample_id % overflow_id_;
}

void HTTPStatusMessage::AddMeasure(::std::string name, double value) {
  // Mutex should be locked when this method is called to synchronize packets.
  assert(mutex_.OwnedBySelf());

  int32_t index = GetIndex(sample_id_ - 1);

  if (measure_index_ >= static_cast<int32_t>(data_names_.size())) {
    data_names_.emplace_back(name);
  }

  if (measure_index_ >= static_cast<int32_t>(data_values_.at(index).size())) {
    data_values_.at(index).emplace_back(value);
  } else {
    data_values_.at(index).at(measure_index_) = value;
  }
  measure_index_++;
}

::std::string HTTPStatusMessage::Fetch(size_t from_sample) {
  ::aos::MutexLocker locker(&mutex_);

  ::std::stringstream message;
  message.precision(10);  // Cap how precise the time/measurement data is.

  // To save space, data is being sent with a custom protocol over to the
  // client.
  // Initially, a message containing all the names of the measurements is sent
  // and is preceeded with a *.
  // Names begin with a star and are split with commas.

  // Example: *test,test2
  if (static_cast<int32_t>(from_sample) == -1) {
    message << "*";
    for (int32_t cur_data_name = 0;
         cur_data_name < static_cast<int32_t>(data_names_.size());
         cur_data_name++) {
      if (cur_data_name > 0) {
        message << ",";
      }
      message << data_names_.at(cur_data_name);
    }
    return message.str();
  }

  // TODO(comran): Use from_sample to determine the speed packets should be sent
  // out to avoid skipping packets.
  from_sample = sample_id_ - num_samples_per_packet_;

  // Data packets are sent, with raw data being placed at the same index as the
  // original index of the measurement name sent in the initial packet.
  // Samples are split with dollar signs, info with percent signs, and
  // measurements with commas.
  // This special format system is helpful for debugging issues and looping
  // through the data on the client side.

  // Example of two samples that correspond with the initialized example:
  // 289%2803.135127%10,67$290%2803.140109%12,68
  for (int32_t cur_sample = from_sample;
       cur_sample <
               static_cast<int32_t>(from_sample + num_samples_per_packet_) &&
           GetIndex(cur_sample) < static_cast<int32_t>(data_values_.size());
       cur_sample++) {
    if (cur_sample != static_cast<int32_t>(from_sample)) {
      message << "$";
    }

    int32_t adjusted_index = GetIndex(cur_sample);

    message << cur_sample << "%" << sample_times_.at(adjusted_index).ToSeconds()
            << "%";
    for (int32_t cur_measure = 0;
         cur_measure < static_cast<int32_t>(data_names_.size());
         cur_measure++) {
      if (cur_measure > 0) {
        message << ",";
      }
      message << data_values_.at(adjusted_index).at(cur_measure);
    }
  }
  return message.str();
}

DataCollector::DataCollector() : cur_raw_data_("no data") {}

void DataCollector::RunIteration() {
  auto& fridge_queue = control_loops::fridge_queue;
  auto& claw_queue = control_loops::claw_queue;

  fridge_queue.status.FetchAnother();
  claw_queue.status.FetchAnother();

  message_.NextSample();
  // Add recorded data here. /////
  // NOTE: Try to use fewer than 30 measures, or the whole thing will lag.
  // Abbreviate names if long, otherwise just use the command to get the value
  // from the queue.

  // TODO(comran): Make it so that the name doesn't have to be copied as a
  // string.

  // //// Fridge
  // Positions
  message_.AddMeasure("(fridge position left arm encoder)",
                      fridge_queue.position->arm.left.encoder);
  message_.AddMeasure("(fridge position right arm encoder)",
                      fridge_queue.position->arm.right.encoder);
  message_.AddMeasure("(fridge position left elev encoder)",
                      fridge_queue.position->elevator.left.encoder);
  message_.AddMeasure("(fridge position right elev encoder)",
                      fridge_queue.position->elevator.right.encoder);
  // Goals
  message_.AddMeasure("fridge_queue.goal->profiling_type",
                      fridge_queue.goal->profiling_type);
  message_.AddMeasure("fridge_queue.goal->angle", fridge_queue.goal->angle);
  message_.AddMeasure("fridge_queue.goal->angular_velocity",
                      fridge_queue.goal->angular_velocity);
  message_.AddMeasure("fridge_queue.goal->height", fridge_queue.goal->height);
  message_.AddMeasure("fridge_queue.goal->velocity",
                      fridge_queue.goal->velocity);
  message_.AddMeasure("fridge_queue.x", fridge_queue.goal->x);
  message_.AddMeasure("fridge_queue.x_velocity", fridge_queue.goal->x_velocity);
  message_.AddMeasure("fridge_queue.y", fridge_queue.goal->y);
  message_.AddMeasure("fridge_queue.y_velocity", fridge_queue.goal->y_velocity);
  // Statuses
  message_.AddMeasure("fridge_queue.status->height",
                      fridge_queue.status->height);
  message_.AddMeasure("fridge_queue.status->velocity",
                      fridge_queue.status->velocity);
  message_.AddMeasure("fridge_queue.status->angle", fridge_queue.status->angle);
  message_.AddMeasure("fridge_queue.status->angular_velocity",
                      fridge_queue.status->angular_velocity);
  message_.AddMeasure("fridge_queue.status->x", fridge_queue.status->x);
  message_.AddMeasure("fridge_queue.status->x_velocity",
                      fridge_queue.status->x_velocity);
  message_.AddMeasure("fridge_queue.status->y", fridge_queue.status->y);
  message_.AddMeasure("fridge_queue.status->y_velocity",
                      fridge_queue.status->y_velocity);
  message_.AddMeasure("fridge_queue.status->state", fridge_queue.status->state);
  message_.AddMeasure("fridge_queue.status->zeroed",
                      fridge_queue.status->zeroed);
  message_.AddMeasure("fridge_queue.status->estopped",
                      fridge_queue.status->estopped);
  // Outputs
  message_.AddMeasure("fridge_queue.output->left_arm",
                      fridge_queue.output->left_arm);
  message_.AddMeasure("fridge_queue.output->right_arm",
                      fridge_queue.output->right_arm);
  message_.AddMeasure("fridge_queue.output->left_elevator",
                      fridge_queue.output->left_elevator);
  message_.AddMeasure("fridge_queue.output->right_elevator",
                      fridge_queue.output->right_elevator);
  // End recorded data. /////
  message_.EndSample();
}

::std::string DataCollector::GetData(int32_t from_sample) {
  return message_.Fetch(from_sample);
}

void DataCollector::operator()() {
  ::aos::SetCurrentThreadName("HTTPStatusData");

  while (run_) {
    ::aos::time::PhasedLoopXMS(5, 0);
    RunIteration();
  }
}

SocketHandler::SocketHandler()
    : data_collector_thread_(::std::ref(data_collector_)) {}

void SocketHandler::onConnect(seasocks::WebSocket* connection) {
  connections_.insert(connection);
  LOG(INFO, "Connected: %s : %s\n", connection->getRequestUri().c_str(),
      seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void SocketHandler::onData(seasocks::WebSocket* connection, const char* data) {
  int32_t from_sample = atoi(data);

  ::std::string send_data = data_collector_.GetData(from_sample);
  connection->send(send_data.c_str());
}

void SocketHandler::onDisconnect(seasocks::WebSocket* connection) {
  connections_.erase(connection);
  LOG(INFO, "Disconnected: %s : %s\n", connection->getRequestUri().c_str(),
      seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void SocketHandler::Quit() {
  data_collector_.Quit();
  data_collector_thread_.join();
}

SeasocksLogger::SeasocksLogger(Level min_level_to_log)
    : PrintfLogger(min_level_to_log) {}

void SeasocksLogger::log(Level level, const char* message) {
  log_level aos_level;
  switch (level) {
    case seasocks::Logger::INFO:
      aos_level = INFO;
      break;
    case seasocks::Logger::WARNING:
      aos_level = WARNING;
      break;
    case seasocks::Logger::ERROR:
    case seasocks::Logger::SEVERE:
      aos_level = ERROR;
      break;
    case seasocks::Logger::DEBUG:
    case seasocks::Logger::ACCESS:
    default:
      aos_level = DEBUG;
      break;
  }
  LOG(aos_level, "Seasocks: %s\n", message);
}

}  // namespace http_status
}  // namespace frc971

int main(int, char* []) {
  ::aos::InitNRT();

  seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
      new frc971::http_status::SeasocksLogger(seasocks::Logger::INFO)));
  frc971::http_status::SocketHandler socket_handler;

  server.addWebSocketHandler(
      "/ws",
      ::std::shared_ptr<frc971::http_status::SocketHandler>(&socket_handler));
  server.serve("www", 8080);

  socket_handler.Quit();

  ::aos::Cleanup();
  return 0;
}
