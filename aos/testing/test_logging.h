#ifndef AOS_TESTING_TEST_LOGGING_H_
#define AOS_TESTING_TEST_LOGGING_H_

namespace aos {
namespace testing {

// Enables the logging framework for use during a gtest test.
// It will print out all WARNING and above messages all of the time. It will
// also print out all log messages when a test fails.
// This function only needs to be called once in each process (after gtest is
// initialized), however it can be called more than that.
void EnableTestLogging();

// Redirect the messages enabled by EnableTestLogging() function to a file.
// By default the messages are printed to standard output.
void SetLogFileName(const char* filename);

// Force the messages to be printed as they are handled by the logging
// framework. This can be useful for tests that hang where no messages would
// otherwise be printed. This is also useful for tests that do pass, but where
// we want to use graphing tools to verify what's happening.
void ForcePrintLogsDuringTests();

}  // namespace testing
}  // namespace aos

#endif  // AOS_TESTING_TEST_LOGGING_H_
