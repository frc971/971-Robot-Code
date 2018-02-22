#include "y2018/control_loops/superstructure/arm/graph.h"

#include "gtest/gtest.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {
namespace testing {

class HeapTest {
 public:
  explicit HeapTest(size_t id, double cost) : id_(id), cost_(cost) {}

  void SetCost(double cost) { cost_ = cost; }

  bool operator<(const HeapTest &o) { return cost_ > o.cost_; }

 private:
  size_t id_;
  double cost_;

  IntrusivePriorityQueue<HeapTest>::QueueEntry *entry_ = nullptr;
  friend class IntrusivePriorityQueue<HeapTest>;
};

TEST(IntrusivePriorityQueue, HeapTest) {
  std::vector<HeapTest> items;
  IntrusivePriorityQueue<HeapTest> queue;
  size_t i = 0;
  for (double value :
       {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0}) {
    items.push_back(HeapTest(i, value));
    ++i;
  }
  for (auto &item : items) {
    queue.InsertOrDecrease(&item);
    queue.VerifyCorrectness();
  }

  items[6].SetCost(-1.0);
  queue.InsertOrDecrease(&items[6]);
  queue.VerifyCorrectness();

  while (!queue.Empty()) {
    queue.PopMin();
    queue.VerifyCorrectness();
  }
}

TEST(GraphTest, Distances) {
  SearchGraph graph(6, {{0, 1, 4.0},
                        {1, 5, 2.0},
                        {2, 0, 7.0},
                        {2, 3, 8.0},
                        {3, 1, 2.0},
                        {4, 0, 1.0},
                        {5, 2, 9.0},
                        {5, 4, 8.0}});

  graph.SetGoal(2);
  size_t i = 0;
  for (double expected_cost : {15, 11, 0, 13, 16, 9}) {
    EXPECT_EQ(graph.GetCostToGoal(i), expected_cost);
    ++i;
  }
}

}  // namespace testing
}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
