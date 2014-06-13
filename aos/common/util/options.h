#ifndef AOS_COMMON_UTIL_OPTIONS_H_
#define AOS_COMMON_UTIL_OPTIONS_H_

#include <sys/types.h>

namespace aos {

template <class Owner>
class Options;

// An "option" that can be combined with other options and passed as one
// argument. This class is designed to emulate integral constants (except be
// type-safe), so its instances can be combined with operator| (creating an
// Options), and an Options can be tested for membership with operator&.
// Owner is the only class that can construct Option instances and is used to
// differentiate between options for different classes. It is safe to only have
// a forward declaration for Owner in scope when instantiating either of these
// template classes.
template <class Owner>
class Options {
 public:
  // Represents a single options. Instances of this should be created as
  // constants by Owner.
  class Option {
   public:
    constexpr Options operator|(Option option) const {
      return Options(bit_ | option.bit_);
    }

    constexpr bool operator==(Option other) const { return bit_ == other.bit_; }

    constexpr unsigned int printable() const { return bit_; }

   private:
    // Bit must have exactly 1 bit set and that must be a different one for each
    // instance.
    explicit constexpr Option(unsigned int bit) : bit_(bit) {}

    unsigned int bit_;

    friend class Options;
    friend Owner;
  };

  constexpr Options(Option option) : bits_(option.bit_) {}

  constexpr bool operator&(Option option) const {
    return (bits_ & option.bit_) != 0;
  }

  constexpr Options operator|(Option option) const {
    return Options(bits_ | option.bit_);
  }
  constexpr Options operator|(Options options) const {
    return Options(bits_ | options.bits_);
  }

  constexpr bool operator==(Options other) const {
    return bits_ == other.bits_;
  }

  constexpr unsigned int printable() const { return bits_; }

  // Returns true if no Options other than the ones in options are set.
  // Useful for validating that no illegal options are passed.
  constexpr bool NoOthersSet(Options options) const {
    return (bits_ & ~options.bits_) == 0;
  }

  // Returns true if exactly 1 of the Options in options is set.
  // Useful for validating that one of a group of mutually exclusive options has
  // been passed.
  constexpr bool ExactlyOneSet(Options options) const {
    return __builtin_popcount(bits_ & options.bits_) == 1;
  }

  // Returns true if all of the Options in options are set.
  constexpr bool AllSet(Options options) const {
    return (bits_ & options.bits_) == options.bits_;
  }

 private:
  explicit constexpr Options(unsigned int bits) : bits_(bits) {}

  unsigned int bits_;

  friend class Option;
};

}  // namespace options

#endif  // AOS_COMMON_UTIL_OPTIONS_H_
