#include <memory>
#include <thread>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <atomic>
#include <mutex>
#include <sched.h>
#include <assert.h>
#include <WPILib.h>
#include "dma.h"
#include <signal.h>

::std::atomic<double> last_time;

class priority_mutex {
 public:
  typedef pthread_mutex_t *native_handle_type;

  // TODO(austin): Write a test case for the mutex, and make the constructor
  // constexpr.
  priority_mutex() {
    pthread_mutexattr_t attr;
    // Turn on priority inheritance.
    assert_perror(pthread_mutexattr_init(&attr));
    assert_perror(pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL));
    assert_perror(pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT));

    assert_perror(pthread_mutex_init(native_handle(), &attr));

    assert_perror(pthread_mutexattr_destroy(&attr));
  }

  ~priority_mutex() { pthread_mutex_destroy(&handle_); }

  void lock() { assert_perror(pthread_mutex_lock(&handle_)); }
  bool try_lock() {
    int ret = pthread_mutex_trylock(&handle_);
    if (ret == 0) {
      return true;
    } else if (ret == EBUSY) {
      return false;
    } else {
      assert_perror(ret);
    }
  }
  void unlock() { assert_perror(pthread_mutex_unlock(&handle_)); }

  native_handle_type native_handle() { return &handle_; }

 private:
  DISALLOW_COPY_AND_ASSIGN(priority_mutex);
  pthread_mutex_t handle_;
};

class EdgePrinter {
 public:
  EdgePrinter(DigitalInput *sensor)
      : quit_(false),
        sensor_(sensor),
        interrupt_count_(0) {
  }

  void Start() {
    printf("Creating thread %d\n", sensor_->GetChannel());
    thread_.reset(new ::std::thread(::std::ref(*this)));
  }

  void operator ()() {
    struct sched_param param;
    param.sched_priority = 55;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
      perror("sched_setscheduler failed");
      exit(-1);
    }

    printf("Started thread %d\n", sensor_->GetChannel());

    sensor_->RequestInterrupts();
    sensor_->SetUpSourceEdge(true, false);

    InterruptableSensorBase::WaitResult result = InterruptableSensorBase::kBoth;
    while (!quit_) {
      result = sensor_->WaitForInterrupt(
          0.1, result != InterruptableSensorBase::kTimeout);
      if (result != InterruptableSensorBase::kTimeout) {
        ++interrupt_count_;
        printf("Got %d edges on %d\n", interrupt_count_.load(),
               sensor_->GetChannel());
      }
    }
  }

  int interrupt_count() const { return interrupt_count_; }

  void quit() {
    quit_ = true;
    thread_->join();
  }

  DigitalInput *sensor() { return sensor_.get(); }

 private:
  ::std::atomic<bool> quit_;
  ::std::unique_ptr<DigitalInput> sensor_;
  ::std::atomic<int> interrupt_count_;
  ::std::unique_ptr<::std::thread> thread_;
};

class TestRobot;
static TestRobot *my_robot;

class TestRobot : public RobotBase {
 public:
  static void HandleSigIntStatic(int signal) { my_robot->HandleSigInt(signal); }
  void HandleSigInt(int /*signal*/) { quit_ = true; }

  ::std::unique_ptr<Encoder> MakeEncoder(int index) {
    return ::std::unique_ptr<Encoder>(
        new Encoder(sensor(10 + 2 * index), sensor(11 + 2 * index)));
  }

  ::std::vector<::std::unique_ptr<EdgePrinter>> printers;
  ::std::vector<::std::unique_ptr<DigitalInput>> dio;

  DigitalInput *sensor(int i) {
    if (i < 8) {
      return printers[i]->sensor();
    } else {
      return dio[i - 8].get();
    }
  }

  void AllEdgeTests() {
    my_robot = this;
    quit_ = false;
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    // Setup the sighub handler
    sa.sa_handler = &TestRobot::HandleSigIntStatic;

    // Restart the system call, if at all possible
    sa.sa_flags = SA_RESTART;

    // Block every signal during the handler
    sigfillset(&sa.sa_mask);

    for (int i = 0; i < 8; ++i) {
      printers.emplace_back(new EdgePrinter(new DigitalInput(i)));
    }
    printf("Created all objects\n");
    for (auto &printer : printers) {
      printer->Start();
    }

    for (int i = 8; i < 26; ++i) {
      dio.emplace_back(new DigitalInput(i));
    }

    ::std::unique_ptr<Encoder> e0 = MakeEncoder(0);
    ::std::unique_ptr<Encoder> e1 = MakeEncoder(1);
    ::std::unique_ptr<Encoder> e2 = MakeEncoder(2);
    ::std::unique_ptr<Encoder> e3 = MakeEncoder(3);

    DMA dma;

    dma.Add(sensor(6));
    dma.Add(e0.get());
    dma.SetExternalTrigger(sensor(6), true, true);
    dma.Start();
    while (!quit_) {
      printf("Battery voltage %f\n",
             DriverStation::GetInstance()->GetBatteryVoltage());

      DMASample dma_sample;
      size_t left;
      DMA::ReadStatus dma_read_return = dma.Read(&dma_sample, 1000, &left);
      printf("dma_read %d, items left %d\n", dma_read_return, left);

      if (left >= 0) {
        uint32_t sensor_value = 0;
        uint32_t dma_sensor_value = 0;
        for (int i = 0; i < 26; ++i) {
          int j = i;
          if (j >= 10) j += 6;
          sensor_value |= (static_cast<uint32_t>(sensor(i)->Get()) << j);
          dma_sensor_value |= (static_cast<uint32_t>(dma_sample.Get(sensor(i)) << j));
        }

        printf("dio 0x%x\n", sensor_value);
        printf("dma 0x%x\n", dma_sensor_value);
        printf("e0 %d, e0_dma %d\n", e0->GetRaw(), dma_sample.GetRaw(e0.get()));
        printf("e1 %d, e1_dma %d\n", e1->GetRaw(), dma_sample.GetRaw(e1.get()));
        printf("e2 %d, e2_dma %d\n", e2->GetRaw(), dma_sample.GetRaw(e2.get()));
        printf("e3 %d, e3_dma %d\n", e3->GetRaw(), dma_sample.GetRaw(e3.get()));
        printf("timestamp %f %f\n", dma_sample.GetTimestamp(),
               static_cast<double>(GetFPGATime()) * 0.000001);

        printf("Remaining is %d\n", left);
      }
    }
    // Wait(0.5);
    for (auto &printer : printers) {
      printer->quit();
    }
  }

  virtual void StartCompetition() {
    AllEdgeTests();
  }

 private:
  ::std::unique_ptr<Encoder> encoder_;
  ::std::unique_ptr<Encoder> test_encoder_;
  ::std::unique_ptr<Talon> talon_;
  ::std::unique_ptr<DigitalInput> hall_;

  ::std::atomic<bool> quit_;
  ::std::mutex encoder_mutex_;
};

START_ROBOT_CLASS(TestRobot);
