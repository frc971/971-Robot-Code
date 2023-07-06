#include <iomanip>
#include <iostream>

#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "gflags/gflags.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"

DEFINE_string(skip, "", "Applications to skip, seperated by ;");

struct ChannelState {
  const aos::Channel *channel = nullptr;
  double frequency_sum = 0.0;
  size_t frequency_count = 0;
};

// List of channels for an application.
struct Application {
  std::vector<ChannelState> watchers;
  std::vector<ChannelState> fetchers;
  std::vector<ChannelState> senders;
};

// List of all applications connected to a channel.
struct ChannelConnections {
  std::vector<std::pair<std::string, double>> senders;
  std::vector<std::string> watchers;
  std::vector<std::string> fetchers;
};

int main(int argc, char **argv) {
  gflags::SetUsageMessage(
      "Usage: \n"
      "  aos_graph_channels [args] logfile1 logfile2 ...\n"
      "\n"
      "The output is in dot format.  Typical usage will be to pipe the results "
      "to dot\n"
      "\n"
      "  aos_graph_channels ./log/ | dot -Tx11");

  aos::InitGoogle(&argc, &argv);

  if (argc < 2) {
    LOG(FATAL) << "Expected at least 1 logfile as an argument.";
  }

  const std::vector<std::string> skip_list = absl::StrSplit(FLAGS_skip, ";");
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));

  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.Register(&factory);

  // Now: hook everything up to grab all the timing reports and extract them
  // into the Application data structure.
  std::map<std::string, Application> applications;

  std::vector<std::unique_ptr<aos::EventLoop>> loops;
  for (const aos::Node *node :
       aos::configuration::GetNodes(factory.configuration())) {
    std::unique_ptr<aos::EventLoop> event_loop =
        factory.MakeEventLoop("timing_reports", node);
    event_loop->SkipTimingReport();
    event_loop->SkipAosLog();

    event_loop->MakeWatcher("/aos", [&](const aos::timing::Report
                                            &timing_report) {
      if (std::find(skip_list.begin(), skip_list.end(),
                    timing_report.name()->str()) != skip_list.end()) {
        return;
      }
      // Make an application if one doesn't exist.
      auto it = applications.find(timing_report.name()->str());
      if (it == applications.end()) {
        it = applications.emplace(timing_report.name()->str(), Application())
                 .first;
      }

      // Add watcher state.
      if (timing_report.has_watchers()) {
        for (const aos::timing::Watcher *watcher : *timing_report.watchers()) {
          const aos::Channel *channel =
              factory.configuration()->channels()->Get(
                  watcher->channel_index());
          auto watcher_it = std::find_if(
              it->second.watchers.begin(), it->second.watchers.end(),
              [&](const ChannelState &c) { return c.channel == channel; });
          if (watcher_it == it->second.watchers.end()) {
            it->second.watchers.push_back(ChannelState{.channel = channel,
                                                       .frequency_sum = 0.0,
                                                       .frequency_count = 0});
            watcher_it = it->second.watchers.end() - 1;
          }
          watcher_it->frequency_sum += watcher->count();
          ++watcher_it->frequency_count;
        }
      }

      // Add sender state.
      if (timing_report.has_senders()) {
        for (const aos::timing::Sender *sender : *timing_report.senders()) {
          const aos::Channel *channel =
              factory.configuration()->channels()->Get(sender->channel_index());
          auto sender_it = std::find_if(
              it->second.senders.begin(), it->second.senders.end(),
              [&](const ChannelState &c) { return c.channel == channel; });
          if (sender_it == it->second.senders.end()) {
            it->second.senders.push_back(ChannelState{.channel = channel,
                                                      .frequency_sum = 0.0,
                                                      .frequency_count = 0});
            sender_it = it->second.senders.end() - 1;
          }
          sender_it->frequency_sum += sender->count();
          ++sender_it->frequency_count;
        }
      }

      // Add fetcher state.
      if (timing_report.has_fetchers()) {
        for (const aos::timing::Fetcher *fetcher : *timing_report.fetchers()) {
          const aos::Channel *channel =
              factory.configuration()->channels()->Get(
                  fetcher->channel_index());
          auto fetcher_it = std::find_if(
              it->second.fetchers.begin(), it->second.fetchers.end(),
              [&](const ChannelState &c) { return c.channel == channel; });
          if (fetcher_it == it->second.fetchers.end()) {
            it->second.fetchers.push_back(ChannelState{.channel = channel,
                                                       .frequency_sum = 0.0,
                                                       .frequency_count = 0});
            fetcher_it = it->second.fetchers.end() - 1;
          }
          fetcher_it->frequency_sum += fetcher->count();
          ++fetcher_it->frequency_count;
        }
      }
    });
    loops.emplace_back(std::move(event_loop));
  }

  factory.Run();

  reader.Deregister();

  // Now, we need to flip this graph on it's head to deduplicate and draw the
  // correct graph.  Build it all up as a list of applications per channel.
  std::map<const aos::Channel *, ChannelConnections> connections;
  for (const std::pair<const std::string, Application> &app : applications) {
    for (const ChannelState &state : app.second.senders) {
      auto it = connections.find(state.channel);
      if (it == connections.end()) {
        it = connections.emplace(state.channel, ChannelConnections()).first;
      }

      it->second.senders.emplace_back(std::make_pair(
          app.first, state.frequency_count == 0
                         ? 0.0
                         : state.frequency_sum / state.frequency_count));
    }
    for (const ChannelState &state : app.second.watchers) {
      auto it = connections.find(state.channel);
      if (it == connections.end()) {
        it = connections.emplace(state.channel, ChannelConnections()).first;
      }

      it->second.watchers.emplace_back(app.first);
    }
    for (const ChannelState &state : app.second.fetchers) {
      auto it = connections.find(state.channel);
      if (it == connections.end()) {
        it = connections.emplace(state.channel, ChannelConnections()).first;
      }

      it->second.fetchers.emplace_back(app.first);
    }
  }

  const std::vector<std::string> color_list = {
      "red", "blue", "orange", "green", "violet", "gold3", "magenta"};

  // Now generate graphvis compatible output.
  std::stringstream graph_out;
  graph_out << "digraph g {" << std::endl;
  for (const std::pair<const aos::Channel *const, ChannelConnections> &c :
       connections) {
    const std::string channel = absl::StrCat(
        c.first->name()->string_view(), "\n", c.first->type()->string_view());
    for (const std::pair<std::string, double> &sender : c.second.senders) {
      graph_out << "\t\"" << sender.first << "\" -> \"" << channel
                << "\" [label=\"" << sender.second << "\" color=\""
                << color_list[0]
                << "\" weight=" << static_cast<int>(sender.second) << "];"
                << std::endl;
    }
    for (const std::string &watcher : c.second.watchers) {
      graph_out << "\t\"" << channel << "\" -> \"" << watcher << "\" [color=\""
                << color_list[1] << "\"];" << std::endl;
    }
    for (const std::string &watcher : c.second.fetchers) {
      graph_out << "\t\"" << channel << "\" -> \"" << watcher << "\" [color=\""
                << color_list[2] << "\"];" << std::endl;
    }
  }

  size_t index = 0;
  for (const std::pair<const std::string, Application> &app : applications) {
    graph_out << "\t\"" << app.first << "\" [color=\"" << color_list[index]
              << "\" shape=box style=filled];" << std::endl;
    ++index;
    if (index >= color_list.size()) {
      index = 0;
    }
  }
  graph_out << "}" << std::endl;

  std::cout << graph_out.str();

  return 0;
}
