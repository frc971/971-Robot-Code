#include <map>

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"
#include "aos/starter/starterd_lib.h"

namespace aos {
namespace starter {

// Simple mock of starterd that updates the starter status message to act as
// though applications are started and stopped when requested.
// TODO(james.kuszmaul): Consider integrating with SimulatedEventLoopFactory.
class MockStarter {
 public:
  struct ApplicationStatus {
    int id;
    bool running;
    aos::monotonic_clock::time_point start_time;
  };

  MockStarter(aos::EventLoop *event_loop);

  const aos::Node *node() const { return event_loop_->node(); }

  const std::map<std::string, ApplicationStatus> &statuses() const {
    return statuses_;
  }

 private:
  void SendStatus();

  aos::EventLoop *event_loop_;
  aos::Sender<aos::starter::Status> status_sender_;
  std::map<std::string, ApplicationStatus> statuses_;
  int next_id_ = 0;
};

// Spins up MockStarter's for each node.
class MockStarters {
 public:
  MockStarters(aos::SimulatedEventLoopFactory *event_loop_factory);
  const std::vector<std::unique_ptr<MockStarter>> &starters() const {
    return mock_starters_;
  }

 private:
  std::vector<std::unique_ptr<aos::EventLoop>> event_loops_;
  std::vector<std::unique_ptr<MockStarter>> mock_starters_;
};

}  // namespace starter
}  // namespace aos
