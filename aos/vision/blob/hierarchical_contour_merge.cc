#include "aos/vision/blob/hierarchical_contour_merge.h"

#include <math.h>
#include <queue>

#include "aos/vision/blob/disjoint_set.h"

namespace aos {
namespace vision {

namespace {

int Mod(int a, int n) { return a - n * (a / n); }

}  // namespace

template <typename T>
class IntegralArray {
 public:
  IntegralArray() {}
  IntegralArray(int size) { items_.reserve(size); }

  // This is an exclusive range lookup into a modulo ring.
  // The integral is precomputed in items_ and is inclusive even though
  // the input is [a, b).
  T Get(int a, int b) {
    a = Mod(a, items_.size());
    b = Mod(b, items_.size());
    if (a == b) return 0;
    if (b < a) {
      if (b == 0) {
        return items_[items_.size() - 1] - items_[a - 1];
      }
      return items_[items_.size() - 1] + items_[b - 1] - items_[a - 1];
    }
    if (a == 0) {
      return items_[b - 1];
    } else {
      return items_[b - 1] - items_[a - 1];
    }
  }
  void Add(T t) {
    if (items_.size() == 0) {
      items_.push_back(t);
    } else {
      items_.push_back(t + items_[items_.size() - 1]);
    }
  }

 private:
  std::vector<T> items_;
};

class IntegralLineFit {
 public:
  IntegralLineFit(int number_of_points, int min_line_length)
      : xx_(number_of_points),
        xy_(number_of_points),
        yy_(number_of_points),
        x_(number_of_points),
        y_(number_of_points),
        // These are not IntegralArrays.
        n_(number_of_points),
        min_len_(min_line_length) {}

  void AddPt(Point pt) {
    xx_.Add(pt.x * pt.x);
    xy_.Add(pt.x * pt.y);
    yy_.Add(pt.y * pt.y);
    x_.Add(pt.x);
    y_.Add(pt.y);
  }

  int GetNForRange(int st, int ed) {
    int nv = (ed + 1) - st;
    if (ed < st) {
      nv += n_;
    }
    return nv;
  }

  float GetLineErrorRate(int st, int ed) {
    int64_t nv = GetNForRange(st, ed);

    int64_t px = x_.Get(st, ed);
    int64_t py = y_.Get(st, ed);
    int64_t pxx = xx_.Get(st, ed);
    int64_t pxy = xy_.Get(st, ed);
    int64_t pyy = yy_.Get(st, ed);

    double nvsq = nv * nv;
    double m_xx = (pxx * nv - px * px) / nvsq;
    double m_xy = (pxy * nv - px * py) / nvsq;
    double m_yy = (pyy * nv - py * py) / nvsq;

    double b = m_xx + m_yy;
    double c = m_xx * m_yy - m_xy * m_xy;
    return ((b - sqrt(b * b - 4 * c)) / 2.0);
  }

  float GetErrorLineRange(int st, int ed) {
    int nv = GetNForRange(st, ed);
    int j = std::max(min_len_ - nv, 0) / 2;
    return GetLineErrorRate((st - j + n_) % n_, (ed + 1 + j + n_) % n_);
  }

  FittedLine FitLine(int st, int ed, Point pst, Point ped) {
    int nv = GetNForRange(st, ed);
    // Adjust line out to be at least min_len_.
    int j = std::max(min_len_ - nv, 0) / 2;

    st = Mod(st - j, n_);
    ed = Mod(ed + 1 + j, n_);
    if (nv <= min_len_) {
      return FittedLine{pst, pst};
    }

    int64_t px = x_.Get(st, ed);
    int64_t py = y_.Get(st, ed);
    int64_t pxx = xx_.Get(st, ed);
    int64_t pxy = xy_.Get(st, ed);
    int64_t pyy = yy_.Get(st, ed);

    double nvsq = nv * nv;
    double m_xx = (pxx * nv - px * px) / nvsq;
    double m_xy = (pxy * nv - px * py) / nvsq;
    double m_yy = (pyy * nv - py * py) / nvsq;
    double m_x = px / ((double)nv);
    double m_y = py / ((double)nv);

    double b = (m_xx + m_yy) / 2.0;
    double c = m_xx * m_yy - m_xy * m_xy;

    double eiggen = sqrt(b * b - c);
    double eigv = b - eiggen;

    double vx = m_xx - eigv;
    double vy = m_xy;
    double mag = sqrt(vx * vx + vy * vy);
    vx /= mag;
    vy /= mag;

    double av = vx * (pst.x - m_x) + vy * (pst.y - m_y);
    double bv = vx * (ped.x - m_x) + vy * (ped.y - m_y);

    Point apt = {(int)(m_x + vx * av), (int)(m_y + vy * av)};
    Point bpt = {(int)(m_x + vx * bv), (int)(m_y + vy * bv)};

    return FittedLine{apt, bpt};
  }

 private:
  IntegralArray<int> xx_;
  IntegralArray<int> xy_;
  IntegralArray<int> yy_;
  IntegralArray<int> x_;
  IntegralArray<int> y_;

  // Number of points in contour.
  int n_;

  // Minimum line length we will look for.
  int min_len_;
};

struct JoinEvent {
  int st;
  int ed;
  // All joins defined to be equal in priority.
  // To be used in std::pair<float, JoinEvent> so need a comparator
  // event though it isn't used.
  bool operator<(const JoinEvent & /*o*/) const { return false; }
};

void HierarchicalMerge(ContourNode *stval, std::vector<FittedLine> *fit_lines,
                       float merge_rate, int min_len) {
  ContourNode *c = stval;
  // count the number of points in the contour.
  int n = 0;
  do {
    n++;
    c = c->next;
  } while (c != stval);
  IntegralLineFit fit(n, min_len);
  c = stval;
  std::vector<Point> pts;
  do {
    fit.AddPt(c->pt);
    pts.push_back(c->pt);
    c = c->next;
  } while (c != stval);

  DisjointSet ids(n);

  std::vector<int> sts;
  sts.reserve(n);
  std::vector<int> eds;
  eds.reserve(n);
  for (int i = 0; i < n; i++) {
    sts.push_back(i);
    eds.push_back(i);
  }

  // Note priority queue takes a pair, so float is used as the priority.
  std::priority_queue<std::pair<float, JoinEvent>> events;
  for (int i = 0; i < n; i++) {
    float err = fit.GetErrorLineRange(i - 1, i);
    events.push(
        std::pair<float, JoinEvent>(err, JoinEvent{(i - 1 + n) % n, i}));
  }

  while (events.size() > 0) {
    auto event = events.top().second;
    // Merge the lines that are most like a line.
    events.pop();
    int pi1 = ids.Find(event.st);
    int pi2 = ids.Find(event.ed);
    int st = sts[pi1];
    int ed = eds[pi2];
    if (st == event.st && ed == event.ed && pi1 != pi2) {
      ids[pi2] = pi1;
      int pi = sts[ids.Find((st - 1 + n) % n)];
      int ni = eds[ids.Find((ed + 1 + n) % n)];
      eds[pi1] = ed;
      if (pi != st) {
        float err = fit.GetErrorLineRange(pi, ed);
        if (err < merge_rate) {
          events.push(std::pair<float, JoinEvent>(err, JoinEvent{pi, ed}));
        }
      }
      if (ni != ed) {
        float err = fit.GetErrorLineRange(st, ni);
        if (err < merge_rate) {
          events.push(std::pair<float, JoinEvent>(err, JoinEvent{st, ni}));
        }
      }
    }
  }
  for (int i = 0; i < n; i++) {
    if (ids[i] == -1) {
      int sti = sts[i];
      int edi = eds[i];
      if ((edi - sti + n) % n > min_len) {
        auto line_fit = fit.FitLine(sti, edi, pts[sti], pts[edi]);
        fit_lines->emplace_back(line_fit);
      }
    }
  }
}

}  // namespace vision
}  // namespace aos
