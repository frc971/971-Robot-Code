#include "aos/events/logging/log_reader_utils.h"

#include "absl/strings/str_join.h"

namespace {

struct ChannelsExtractorAccumulator {
  // A set of senders, watchers and fetchers to have unique channels
  std::set<int> senders;
  std::set<int> watchers;
  std::set<int> fetchers;

  // observed_applications are all the applications for which timing reports
  // have been found
  std::vector<std::set<std::string>> observed_applications;

  // remaining_applications are the vector of applications that have not been
  // found on the nodes specified
  std::vector<std::set<std::string>> remaining_applications;
};

void HandleChannelsInApplications(
    const aos::timing::Report &report, const size_t nodes_index,
    aos::SimulatedEventLoopFactory *factory,
    const aos::Configuration *logged_configuration,
    const aos::logger::ChannelsInLogOptions &options,
    ChannelsExtractorAccumulator *results) {
  std::string name = report.name()->str();
  if (results->observed_applications[nodes_index].count(name) > 0) {
    if (!results->remaining_applications[nodes_index].empty()) {
      LOG(FATAL) << "Didn't see timing reports for every application! "
                 << absl::StrJoin(results->remaining_applications[nodes_index],
                                  ", ");
    } else {
      factory->Exit();
    }
  }
  if (results->remaining_applications[nodes_index].count(name) == 0) {
    return;
  }
  results->observed_applications[nodes_index].insert(name);
  results->remaining_applications[nodes_index].erase(name);

  if (options.get_senders) {
    if (report.has_senders()) {
      for (const aos::timing::Sender *sender : *report.senders()) {
        CHECK_LT(0, sender->channel_index());
        CHECK_LT(static_cast<size_t>(sender->channel_index()),
                 logged_configuration->channels()->size());
        results->senders.insert(sender->channel_index());
      }
    }
  }

  if (options.get_watchers) {
    if (report.has_watchers()) {
      for (const aos::timing::Watcher *watcher : *report.watchers()) {
        CHECK_LT(0, watcher->channel_index());
        CHECK_LT(static_cast<size_t>(watcher->channel_index()),
                 factory->configuration()->channels()->size());
        results->watchers.insert(watcher->channel_index());
      }
    }
  }

  if (options.get_fetchers) {
    if (report.has_fetchers()) {
      for (const aos::timing::Fetcher *fetcher : *report.fetchers()) {
        CHECK_LT(0, fetcher->channel_index());
        CHECK_LT(static_cast<size_t>(fetcher->channel_index()),
                 factory->configuration()->channels()->size());
        results->fetchers.insert(fetcher->channel_index());
      }
    }
  }
}

class ChannelsExtractor {
 public:
  ChannelsExtractor(aos::EventLoop *node_event_loop,
                    ChannelsExtractorAccumulator *results,
                    const size_t node_index,
                    aos::SimulatedEventLoopFactory *factory,
                    const aos::Configuration *logged_configuration,
                    const aos::logger::ChannelsInLogOptions options) {
    // skip timing report because we don't want the reader to generate a timing
    // report
    node_event_loop->SkipTimingReport();
    node_event_loop->SkipAosLog();

    // This is the watcher which looks for applications and then records the
    // respective channels
    node_event_loop->MakeWatcher(
        "/aos", [results, node_index, factory, logged_configuration,
                 options](const aos::timing::Report &report) {
          HandleChannelsInApplications(report, node_index, factory,
                                       logged_configuration, options, results);
        });
  }
};

}  // namespace
namespace aos::logger {

ChannelsInLogResult ChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications,
    const aos::logger::ChannelsInLogOptions options) {
  // Make a log_reader object to make event loop and look into channels and
  // configuration
  LogReader reader(log_files);
  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.RegisterWithoutStarting(&factory);

  ChannelsExtractorAccumulator results;

  // Make watchers for every node in "nodes" and for all applications in
  // "applications" that ran for that node
  for (size_t ii = 0; ii < nodes.size(); ++ii) {
    results.observed_applications.push_back({});
    results.remaining_applications.push_back(
        std::set<std::string>(applications.begin(), applications.end()));

    const aos::Node *const node = nodes.at(ii);

    aos::NodeEventLoopFactory *node_factory = factory.GetNodeEventLoopFactory(
        aos::configuration::GetNode(factory.configuration(), node));

    reader.OnStart(node,
                   [node_factory, &results, ii, &factory, &reader, &options]() {
                     node_factory->AlwaysStart<ChannelsExtractor>(
                         "channels_extractor", &results, ii, &factory,
                         reader.logged_configuration(), options);
                   });
  }

  factory.Run();
  reader.Deregister();

  for (size_t ii = 0; ii < nodes.size(); ++ii) {
    if (!results.remaining_applications[ii].empty()) {
      LOG(INFO) << "Didn't find all applications requested on "
                << nodes[ii]->name()->string_view()
                << ": remaining applications: "
                << absl::StrJoin(results.remaining_applications[ii], ", ");
    }
  }

  ChannelsInLogResult channels;

  if (options.get_senders) {
    channels.senders = std::make_optional<std::vector<aos::ChannelT>>({});
    for (const int index : results.senders) {
      channels.senders.value().push_back({});
      reader.logged_configuration()->channels()->Get(index)->UnPackTo(
          &channels.senders.value().back());
    }
  }

  if (options.get_watchers) {
    channels.watchers = std::make_optional<std::vector<aos::ChannelT>>({});
    for (const int index : results.watchers) {
      channels.watchers.value().push_back({});
      reader.configuration()->channels()->Get(index)->UnPackTo(
          &channels.watchers.value().back());
    }
  }

  if (options.get_fetchers) {
    channels.fetchers = std::make_optional<std::vector<aos::ChannelT>>({});
    for (const int index : results.fetchers) {
      channels.fetchers.value().push_back({});
      reader.configuration()->channels()->Get(index)->UnPackTo(
          &channels.fetchers.value().back());
    }
  }

  if (options.get_senders && options.get_watchers && options.get_fetchers) {
    channels.watchers_and_fetchers_without_senders =
        std::make_optional<std::vector<aos::ChannelT>>({});
    std::set<int> watchers_and_fetchers_without_senders;
    // TODO(EricS) probably a better way to optimize this symmetric diff algo
    for (const int watcher : results.watchers) {
      if (!std::binary_search(results.senders.begin(), results.senders.end(),
                              watcher)) {
        watchers_and_fetchers_without_senders.insert(watcher);
      }
    }

    for (const int fetcher : results.fetchers) {
      if (!std::binary_search(results.senders.begin(), results.senders.end(),
                              fetcher)) {
        watchers_and_fetchers_without_senders.insert(fetcher);
      }
    }

    for (const int index : watchers_and_fetchers_without_senders) {
      channels.watchers_and_fetchers_without_senders.value().push_back({});
      reader.configuration()->channels()->Get(index)->UnPackTo(
          &channels.watchers_and_fetchers_without_senders.value().back());
    }
  }

  return channels;
}

std::vector<aos::ChannelT> SenderChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications) {
  return ChannelsInLog(log_files, nodes, applications, {true, false, false})
      .senders.value();
}

std::vector<aos::ChannelT> WatcherChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications) {
  return ChannelsInLog(log_files, nodes, applications, {false, true, false})
      .watchers.value();
}

std::vector<aos::ChannelT> FetcherChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications) {
  return ChannelsInLog(log_files, nodes, applications, {false, false, true})
      .fetchers.value();
}

}  // namespace aos::logger
