#ifndef FRC971_SHOOTER_INTERPOLATION_INTERPOLATION_H_
#define FRC971_SHOOTER_INTERPOLATION_INTERPOLATION_H_

#include <algorithm>
#include <utility>
#include <vector>

namespace frc971 {
namespace shooter_interpolation {

double Blend(double coefficient, double a1, double a2);

template <typename YValue>
class InterpolationTable {
 public:
  using Point = ::std::pair<double, YValue>;
  InterpolationTable() = default;
  InterpolationTable(const ::std::vector<Point> &table);

  // Uses the interpolation table to calculate the optimal shooter angle and
  // power for a shot
  YValue Get(double x) const;

  bool GetInRange(double x, YValue* type) const;

 private:
  // Contains the list of angle entries in the interpolation table
  ::std::vector<Point> table_;
};

template <typename YValue>
InterpolationTable<YValue>::InterpolationTable(const ::std::vector<Point> &table)
  : table_(table) {
    ::std::sort(table_.begin(), table_.end(),
                [](const Point &a, const Point &b) {
                return a.first < b.first;
                });
  }

template <typename YValue>
YValue InterpolationTable<YValue>::Get(double x) const {
  // Points to to the smallest item such that it->first >= dist, or end() if no
  // such item exists.
  auto it = ::std::lower_bound(table_.begin(), table_.end(), x,
                               [](const Point &a,
                                  double dist) { return a.first < dist; });
  if (it == table_.begin()) {
    return it->second;
  } else if (it == table_.end()) {
    return table_.back().second;
  } else {
    auto x_a2 = it;
    auto x_a1 = it - 1;
    double x1 = x_a1->first;
    double x2 = x_a2->first;
    double coefficient = (x - x1) / (x2 - x1);
    return YValue::BlendY(coefficient, x_a1->second, x_a2->second);
  }
}

template <typename YValue>
bool InterpolationTable<YValue>::GetInRange(double x, YValue* result) const {
  // Points to to the smallest item such that it->first >= dist, or end() if no
  // such item exists.
  auto it = ::std::lower_bound(table_.begin(), table_.end(), x,
                               [](const Point &a,
                                  double dist) { return a.first < dist; });
  if (it == table_.begin()) {
    return false;
  } else if (it == table_.end()) {
    return false;
  } else {
    auto x_a2 = it;
    auto x_a1 = it - 1;
    double x1 = x_a1->first;
    double x2 = x_a2->first;
    double coefficient = (x - x1) / (x2 - x1);
    *result = YValue::BlendY(coefficient, x_a1->second, x_a2->second);
    return true;
  }
}

}  // namespace shooter_interpolation
}  // namespace frc971

#endif  // FRC971_SHOOTER_INTERPOLATION_INTERPOLATION_H_
