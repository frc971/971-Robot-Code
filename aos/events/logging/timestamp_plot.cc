#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "aos/init.h"
#include "aos/util/file.h"
#include "frc971/analysis/in_process_plotter.h"

using frc971::analysis::Plotter;

// Simple C++ application to read the CSV files and use the in process plotter
// to plot them.  This smokes the pants off gnuplot in terms of interactivity.

namespace aos {

std::pair<std::vector<double>, std::vector<double>> ReadSamples(
    std::string_view node1, std::string_view node2, bool flip) {
  std::vector<double> samplefile12_t;
  std::vector<double> samplefile12_o;

  const std::string file = aos::util::ReadFileToStringOrDie(absl::StrCat(
      "/tmp/timestamp_noncausal_", node1, "_", node2, "_samples.csv"));
  bool first = true;
  std::vector<std::string_view> lines = absl::StrSplit(file, '\n');
  samplefile12_t.reserve(lines.size());
  for (const std::string_view n : lines) {
    if (first) {
      first = false;
      continue;
    }
    if (n == "") {
      continue;
    }

    std::vector<std::string_view> l = absl::StrSplit(n, ", ");
    CHECK_EQ(l.size(), 4u);
    double t;
    double o;
    CHECK(absl::SimpleAtod(l[0], &t));
    CHECK(absl::SimpleAtod(l[1], &o));
    samplefile12_t.emplace_back(t);
    samplefile12_o.emplace_back(flip ? -o : o);
  }
  return std::make_pair(samplefile12_t, samplefile12_o);
}

std::pair<std::vector<double>, std::vector<double>> ReadLines(
    std::string_view node1, std::string_view node2, bool flip) {
  std::vector<double> samplefile12_t;
  std::vector<double> samplefile12_o;

  const std::string file = aos::util::ReadFileToStringOrDie(
      absl::StrCat("/tmp/timestamp_noncausal_", node1, "_", node2, ".csv"));
  bool first = true;
  std::vector<std::string_view> lines = absl::StrSplit(file, '\n');
  samplefile12_t.reserve(lines.size());
  for (const std::string_view n : lines) {
    if (first) {
      first = false;
      continue;
    }
    if (n == "") {
      continue;
    }

    std::vector<std::string_view> l = absl::StrSplit(n, ", ");
    CHECK_EQ(l.size(), 3u);
    double t;
    double o;
    CHECK(absl::SimpleAtod(l[0], &t));
    CHECK(absl::SimpleAtod(l[2], &o));
    samplefile12_t.emplace_back(t);
    samplefile12_o.emplace_back(flip ? -o : o);
  }
  return std::make_pair(samplefile12_t, samplefile12_o);
}

std::pair<std::vector<double>, std::vector<double>> ReadOffset(
    std::string_view node1, std::string_view node2) {
  int node1_index = -1;
  int node2_index = -1;

  {
    const std::string start_time_file = aos::util::ReadFileToStringOrDie(
        "/tmp/timestamp_noncausal_starttime.csv");
    std::vector<std::string_view> nodes = absl::StrSplit(start_time_file, '\n');

    int index = 0;
    for (const std::string_view n : nodes) {
      if (n == "") {
        continue;
      }

      std::vector<std::string_view> l = absl::StrSplit(n, ", ");
      CHECK_EQ(l.size(), 2u) << "'" << n << "'";
      if (l[0] == node1) {
        node1_index = index;
      }
      if (l[0] == node2) {
        node2_index = index;
      }
      ++index;
    }
  }
  CHECK_NE(node1_index, -1) << ": Unknown node " << node1;
  CHECK_NE(node2_index, -1) << ": Unknown node " << node2;
  std::vector<double> offsetfile_t;
  std::vector<double> offsetfile_o;

  const std::string file =
      aos::util::ReadFileToStringOrDie("/tmp/timestamp_noncausal_offsets.csv");
  bool first = true;
  std::vector<std::string_view> lines = absl::StrSplit(file, '\n');
  offsetfile_t.reserve(lines.size());
  for (const std::string_view n : lines) {
    if (first) {
      first = false;
      continue;
    }
    if (n == "") {
      continue;
    }

    std::vector<std::string_view> l = absl::StrSplit(n, ", ");
    CHECK_LT(static_cast<size_t>(node1_index + 1), l.size());
    CHECK_LT(static_cast<size_t>(node2_index + 1), l.size());
    double t;
    double o1;
    double o2;
    CHECK(absl::SimpleAtod(l[0], &t));
    CHECK(absl::SimpleAtod(l[1 + node1_index], &o1));
    CHECK(absl::SimpleAtod(l[1 + node2_index], &o2));
    offsetfile_t.emplace_back(t);
    offsetfile_o.emplace_back(o2 - o1);
  }
  return std::make_pair(offsetfile_t, offsetfile_o);
}

void AddNodes(Plotter *plotter, std::string_view node1,
              std::string_view node2) {
  const std::pair<std::vector<double>, std::vector<double>> samplefile12 =
      ReadSamples(node1, node2, false);
  const std::pair<std::vector<double>, std::vector<double>> samplefile21 =
      ReadSamples(node2, node1, true);

  const std::pair<std::vector<double>, std::vector<double>> noncausalfile12 =
      ReadLines(node1, node2, false);
  const std::pair<std::vector<double>, std::vector<double>> noncausalfile21 =
      ReadLines(node2, node1, true);

  const std::pair<std::vector<double>, std::vector<double>> offsetfile =
      ReadOffset(node1, node2);

  CHECK_EQ(samplefile12.first.size(), samplefile12.second.size());
  CHECK_EQ(samplefile21.first.size(), samplefile21.second.size());
  CHECK_EQ(noncausalfile12.first.size(), noncausalfile12.second.size());
  CHECK_EQ(noncausalfile21.first.size(), noncausalfile21.second.size());

  LOG(INFO) << samplefile12.first.size() + samplefile21.first.size() +
                   noncausalfile12.first.size() + noncausalfile21.first.size()
            << " points";
  plotter->AddLine(
      samplefile12.first, samplefile12.second,
      Plotter::LineOptions{.label = absl::StrCat("sample ", node1, " ", node2),
                           .line_style = "*",
                           .color = "purple"});
  plotter->AddLine(
      samplefile21.first, samplefile21.second,
      Plotter::LineOptions{.label = absl::StrCat("sample ", node2, " ", node1),
                           .line_style = "*",
                           .color = "green"});

  plotter->AddLine(
      noncausalfile12.first, noncausalfile12.second,
      Plotter::LineOptions{.label = absl::StrCat("nc ", node1, " ", node2),
                           .line_style = "-",
                           .color = "blue"});
  plotter->AddLine(
      noncausalfile21.first, noncausalfile21.second,
      Plotter::LineOptions{.label = absl::StrCat("nc ", node2, " ", node1),
                           .line_style = "-",
                           .color = "orange"});

  plotter->AddLine(offsetfile.first, offsetfile.second,
                   Plotter::LineOptions{
                       .label = absl::StrCat("filter ", node2, " ", node1),
                       // TODO(austin): roboRIO compiler wants all the fields
                       // filled out, but other compilers don't...  Sigh.
                       .line_style = "*-",
                       .color = "yellow"});
}

int Main(int argc, const char *const *argv) {
  CHECK_EQ(argc, 3);

  LOG(INFO) << argv[1];
  LOG(INFO) << argv[2];

  // TODO(austin): Find all node pairs and plot them...

  const std::string_view node1 = argv[1];
  const std::string_view node2 = argv[2];

  Plotter plotter;
  plotter.AddFigure("Time");

  AddNodes(&plotter, node1, node2);

  plotter.Publish();

  plotter.Spin();

  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::Main(argc, argv);
}
