#ifndef AOS_TESTING_RANDOM_SEED_H_
#define AOS_TESTING_RANDOM_SEED_H_

namespace aos::testing {

// Returns the random seed to use for testing.
//
// This is ${TEST_RANDOM_SEED} if it is set or 1.
int RandomSeed();

}  // namespace aos::testing

#endif  // AOS_TESTING_RANDOM_SEED_H_
