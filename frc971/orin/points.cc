#include "frc971/orin/points.h"

#include <iomanip>
#include <ostream>

namespace frc971 {
namespace apriltag {

std::ostream &operator<<(std::ostream &os, const QuadBoundaryPoint &point) {
  std::ios_base::fmtflags original_flags = os.flags();

  os << "key:" << std::hex << std::setw(16) << std::setfill('0') << point.key
     << " rep01:" << std::setw(10) << point.rep01() << " pt:" << std::setw(6)
     << point.point_bits();
  os.flags(original_flags);
  return os;
}

static_assert(sizeof(QuadBoundaryPoint) == 8,
              "QuadBoundaryPoint didn't pack right.");

std::ostream &operator<<(std::ostream &os, const IndexPoint &point) {
  std::ios_base::fmtflags original_flags = os.flags();

  os << "key:" << std::hex << std::setw(16) << std::setfill('0') << point.key
     << " i:" << std::setw(3) << point.blob_index() << " t:" << std::setw(7)
     << point.theta() << " p:" << std::setw(6) << point.point_bits();
  os.flags(original_flags);
  return os;
}

static_assert(sizeof(IndexPoint) == 8, "IndexPoint didn't pack right.");

}  // namespace apriltag
}  // namespace frc971
