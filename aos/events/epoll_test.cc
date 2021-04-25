#include "aos/events/epoll.h"

#include <fcntl.h>
#include <unistd.h>

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace aos {
namespace internal {
namespace testing {

// A simple wrapper around both ends of a pipe along with some helpers to easily
// read/write data through it.
class Pipe {
 public:
  Pipe() { PCHECK(pipe2(fds_, O_NONBLOCK) == 0); }
  ~Pipe() {
    if (fds_[0] >= 0) {
      PCHECK(close(fds_[0]) == 0);
    }
    if (fds_[1] >= 0) {
      PCHECK(close(fds_[1]) == 0);
    }
  }

  int read_fd() { return fds_[0]; }
  int write_fd() { return fds_[1]; }
  void close_read_fd() {
    PCHECK(close(fds_[0]) == 0);
    fds_[0] = -1;
  }

  void Write(const std::string &data) {
    CHECK_EQ(write(write_fd(), data.data(), data.size()),
             static_cast<ssize_t>(data.size()));
  }

  std::string Read(size_t size) {
    std::string result;
    result.resize(size);
    CHECK_EQ(read(read_fd(), result.data(), size), static_cast<ssize_t>(size));
    return result;
  }

 private:
  int fds_[2];
};

class EPollTest : public ::testing::Test {
  public:
   void RunFor(std::chrono::nanoseconds duration) {
     TimerFd timerfd;
     bool did_quit = false;
     epoll_.OnReadable(timerfd.fd(), [this, &timerfd, &did_quit]() {
       CHECK(!did_quit);
       epoll_.Quit();
       did_quit = true;
       timerfd.Read();
     });
     timerfd.SetTime(monotonic_clock::now() + duration,
                     monotonic_clock::duration::zero());
     epoll_.Run();
     CHECK(did_quit);
     epoll_.DeleteFd(timerfd.fd());
   }

  // Tests should avoid relying on ordering for events closer in time than this,
  // or waiting for longer than this to ensure events happen in order.
  static constexpr std::chrono::nanoseconds tick_duration() {
    return std::chrono::milliseconds(50);
  }

   EPoll epoll_;
};

// Test that the basics of OnReadable work.
TEST_F(EPollTest, BasicReadable) {
  Pipe pipe;
  bool got_data = false;
  epoll_.OnReadable(pipe.read_fd(), [&]() {
    ASSERT_FALSE(got_data);
    ASSERT_EQ("some", pipe.Read(4));
    got_data = true;
  });
  RunFor(tick_duration());
  EXPECT_FALSE(got_data);

  pipe.Write("some");
  RunFor(tick_duration());
  EXPECT_TRUE(got_data);

  epoll_.DeleteFd(pipe.read_fd());
}

// Test that the basics of OnWriteable work.
TEST_F(EPollTest, BasicWriteable) {
  Pipe pipe;
  int number_writes = 0;
  epoll_.OnWriteable(pipe.write_fd(), [&]() {
    pipe.Write(" ");
    ++number_writes;
  });

  // First, fill up the pipe's write buffer.
  RunFor(tick_duration());
  EXPECT_GT(number_writes, 0);

  // Now, if we try again, we shouldn't do anything.
  const int bytes_in_pipe = number_writes;
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, 0);

  // Empty the pipe, then fill it up again.
  for (int i = 0; i < bytes_in_pipe; ++i) {
    ASSERT_EQ(" ", pipe.Read(1));
  }
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, bytes_in_pipe);

  epoll_.DeleteFd(pipe.write_fd());
}

// Test that the basics of OnError work.
TEST_F(EPollTest, BasicError) {
  // In order to trigger an error, close the read file descriptor (per the
  // epoll_ctl manpage, this should trigger an error).
  Pipe pipe;
  int number_errors = 0;
  epoll_.OnError(pipe.write_fd(), [&]() {
    ++number_errors;
  });

  // Sanity check that we *don't* get any errors before anything interesting has
  // happened.
  RunFor(tick_duration());
  EXPECT_EQ(number_errors, 0);

  pipe.close_read_fd();

  // For some reason, OnError doesn't seem to play nice with the timer setup we
  // have in this test, so just poll for a single event.
  epoll_.Poll(false);

  EXPECT_EQ(number_errors, 1);

  epoll_.DeleteFd(pipe.write_fd());
}

TEST(EPollDeathTest, InvalidFd) {
  EPoll epoll;
  Pipe pipe;
  epoll.OnReadable(pipe.read_fd(), []() {});
  EXPECT_DEATH(epoll.OnReadable(pipe.read_fd(), []() {}),
               "Duplicate in functions");
  epoll.OnWriteable(pipe.read_fd(), []() {});
  EXPECT_DEATH(epoll.OnWriteable(pipe.read_fd(), []() {}),
               "Duplicate out functions");

  epoll.DeleteFd(pipe.read_fd());
  EXPECT_DEATH(epoll.DeleteFd(pipe.read_fd()), "fd [0-9]+ not found");
  EXPECT_DEATH(epoll.DeleteFd(pipe.write_fd()), "fd [0-9]+ not found");
}

// Tests that enabling/disabling a writeable FD works.
TEST_F(EPollTest, WriteableEnableDisable) {
  Pipe pipe;
  int number_writes = 0;
  epoll_.OnWriteable(pipe.write_fd(), [&]() {
    pipe.Write(" ");
    ++number_writes;
  });

  // First, fill up the pipe's write buffer.
  RunFor(tick_duration());
  EXPECT_GT(number_writes, 0);

  // Empty the pipe.
  const int bytes_in_pipe = number_writes;
  for (int i = 0; i < bytes_in_pipe; ++i) {
    ASSERT_EQ(" ", pipe.Read(1));
  }

  // If we disable writeable checking, then nothing should happen.
  epoll_.DisableWriteable(pipe.write_fd());
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, 0);

  // Disabling it again should be a NOP.
  epoll_.DisableWriteable(pipe.write_fd());

  // And then when we re-enable, it should fill the pipe up again.
  epoll_.EnableWriteable(pipe.write_fd());
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, bytes_in_pipe);

  epoll_.DeleteFd(pipe.write_fd());
}

}  // namespace testing
}  // namespace internal
}  // namespace aos
