#ifndef FRC971_ZEROING_WRAP_H_
#define FRC971_ZEROING_WRAP_H_

namespace frc971 {
namespace zeroing {

// Returns a modified value which has been wrapped such that it is +- period/2
// away from nearest.
double Wrap(double nearest, double value, double period);

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_WRAP_H_
