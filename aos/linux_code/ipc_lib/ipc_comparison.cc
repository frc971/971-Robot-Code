#include "third_party/gflags/include/gflags/gflags.h"

#include <stdint.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/eventfd.h>
#include <semaphore.h>
#include <mqueue.h>

#include <thread>
#include <memory>
#include <string>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/implementations.h"
#include "aos/common/time.h"
#include "aos/common/mutex.h"
#include "aos/common/event.h"
#include "aos/common/condition.h"
#include "aos/linux_code/init.h"
#include "aos/linux_code/ipc_lib/queue.h"

DEFINE_string(method, "", "Which IPC method to use");
DEFINE_int32(messages, 1000000, "How many messages to send back and forth");
DEFINE_int32(client_cpu, 0, "CPU to pin client to");
DEFINE_int32(server_cpu, 0, "CPU to pin server to");
DEFINE_int32(client_priority, 1,
             "Realtime priority for client. Negative for don't change");
DEFINE_int32(server_priority, 1,
             "Realtime priority for server. Negative for don't change");

namespace aos {

// A generic interface for an object which can send some data to another thread
// and back.
//
// One side is called the "server". It should constantly Wait, do something with
// the result, and then call Pong.
// The other side is called the "client". It should repeatedly call Ping.
class PingPongerInterface {
 public:
  // A chunk of memory definitely on its own cache line anywhere sane.
  typedef uint8_t Data[1024] __attribute__((aligned(128)));

  virtual ~PingPongerInterface() {}

  // Returns where the "client" side should write data in preparation to send to
  // the server.
  // The result is valid until the next Ping call.
  virtual Data *PingData() = 0;

  // Sends the data returned from the most recent PingData call to the "server"
  // side and returns its response.
  // PingData must be called exactly once before each call of this method.
  // The result is valid until the next PingData call.
  virtual const Data *Ping() = 0;

  // Waits for a Ping call and then returns the associated data.
  // The result is valid until the beginning of the next Pong call.
  virtual const Data *Wait() = 0;

  // Returns where the "server" side should write data in preparation to send
  // back to the "client".
  // The result is valid until the next Pong call.
  virtual Data *PongData() = 0;

  // Sends data back to an in-progress Ping.
  // Sends the data returned from the most recent PongData call back to an
  // in-progress Ping.
  // PongData must be called exactly once before each call of this method.
  virtual void Pong() = 0;
};

// Base class for implementations which simple use a pair of Data objects for
// all Pings and Pongs.
class StaticPingPonger : public PingPongerInterface {
 public:
  Data *PingData() override { return &ping_data_; }
  Data *PongData() override { return &pong_data_; }

 private:
  Data ping_data_, pong_data_;
};

// Implements ping-pong by sending the data over file descriptors.
class FDPingPonger : public StaticPingPonger {
 protected:
  // Subclasses must override and call Init.
  FDPingPonger() {}

  // Subclasses must call this in their constructor.
  // Does not take ownership of any of the file descriptors, any/all of which
  // may be the same.
  // {server,client}_read must be open for reading and {server,client}_write
  // must be open for writing.
  void Init(int server_read, int server_write, int client_read,
            int client_write) {
    server_read_ = server_read;
    server_write_ = server_write;
    client_read_ = client_read;
    client_write_ = client_write;
  }

 private:
  const Data *Ping() override {
    WriteFully(client_write_, *PingData());
    ReadFully(client_read_, &read_by_client_);
    return &read_by_client_;
  }

  const Data *Wait() override {
    ReadFully(server_read_, &read_by_server_);
    return &read_by_server_;
  }

  void Pong() override { WriteFully(server_write_, *PongData()); }

  void ReadFully(int fd, Data *data) {
    size_t remaining = sizeof(*data);
    uint8_t *current = &(*data)[0];
    while (remaining > 0) {
      const ssize_t result = PCHECK(read(fd, current, remaining));
      CHECK_LE(static_cast<size_t>(result), remaining);
      remaining -= result;
      current += result;
    }
  }

  void WriteFully(int fd, const Data &data) {
    size_t remaining = sizeof(data);
    const uint8_t *current = &data[0];
    while (remaining > 0) {
      const ssize_t result = PCHECK(write(fd, current, remaining));
      CHECK_LE(static_cast<size_t>(result), remaining);
      remaining -= result;
      current += result;
    }
  }

  Data read_by_client_, read_by_server_;
  int server_read_ = -1, server_write_ = -1, client_read_ = -1,
      client_write_ = -1;
};

class PipePingPonger : public FDPingPonger {
 public:
  PipePingPonger() {
    PCHECK(pipe(to_server));
    PCHECK(pipe(from_server));
    Init(to_server[0], from_server[1], from_server[0], to_server[1]);
  }
  ~PipePingPonger() {
    PCHECK(close(to_server[0]));
    PCHECK(close(to_server[1]));
    PCHECK(close(from_server[0]));
    PCHECK(close(from_server[1]));
  }

 private:
  int to_server[2], from_server[2];
};

class NamedPipePingPonger : public FDPingPonger {
 public:
  NamedPipePingPonger() {
    OpenFifo("/tmp/to_server", &client_write_, &server_read_);
    OpenFifo("/tmp/from_server", &server_write_, &client_read_);

    Init(server_read_, server_write_, client_read_, client_write_);
  }
  ~NamedPipePingPonger() {
    PCHECK(close(server_read_));
    PCHECK(close(client_write_));
    PCHECK(close(client_read_));
    PCHECK(close(server_write_));
  }

 private:
  void OpenFifo(const char *name, int *write, int *read) {
    {
      const int ret = unlink(name);
      if (ret == -1 && errno != ENOENT) {
        PLOG(FATAL, "unlink(%s)", name);
      }
      PCHECK(mkfifo(name, S_IWUSR | S_IRUSR));
      // Have to open it nonblocking because the other end isn't open yet...
      *read = PCHECK(open(name, O_RDONLY | O_NONBLOCK));
      *write = PCHECK(open(name, O_WRONLY));
      {
        const int flags = PCHECK(fcntl(*read, F_GETFL));
        PCHECK(fcntl(*read, F_SETFL, flags & ~O_NONBLOCK));
      }
    }
  }

  int server_read_, server_write_, client_read_, client_write_;
};

class UnixPingPonger : public FDPingPonger {
 public:
  UnixPingPonger(int type) {
    PCHECK(socketpair(AF_UNIX, type, 0, to_server));
    PCHECK(socketpair(AF_UNIX, type, 0, from_server));
    Init(to_server[0], from_server[1], from_server[0], to_server[1]);
  }
  ~UnixPingPonger() {
    PCHECK(close(to_server[0]));
    PCHECK(close(to_server[1]));
    PCHECK(close(from_server[0]));
    PCHECK(close(from_server[1]));
  }

 private:
  int to_server[2], from_server[2];
};

class TCPPingPonger : public FDPingPonger {
 public:
  TCPPingPonger() {
    server_ = PCHECK(socket(AF_INET, SOCK_STREAM, 0));
    {
      sockaddr_in server_address;
      memset(&server_address, 0, sizeof(server_address));
      server_address.sin_family = AF_INET;
      server_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      PCHECK(bind(server_, reinterpret_cast<sockaddr *>(&server_address),
                  sizeof(server_address)));
    }
    PCHECK(listen(server_, 1));

    client_ = PCHECK(socket(AF_INET, SOCK_STREAM, 0));
    {
      sockaddr_in client_address;
      unsigned int length = sizeof(client_address);
      PCHECK(getsockname(server_, reinterpret_cast<sockaddr *>(&client_address),
                         &length));
      PCHECK(connect(client_, reinterpret_cast<sockaddr *>(&client_address),
                     length));
    }
    server_connection_ = PCHECK(accept(server_, nullptr, 0));

    Init(server_connection_, server_connection_, client_, client_);
  }
  ~TCPPingPonger() {
    PCHECK(close(client_));
    PCHECK(close(server_connection_));
    PCHECK(close(server_));
  }

 private:
  int server_, client_, server_connection_;
};

class UDPPingPonger : public FDPingPonger {
 public:
  UDPPingPonger() {
    CreatePair(&server_read_, &client_write_);
    CreatePair(&client_read_, &server_write_);

    Init(server_read_, server_write_, client_read_, client_write_);
  }
  ~UDPPingPonger() {
    PCHECK(close(server_read_));
    PCHECK(close(client_write_));
    PCHECK(close(client_read_));
    PCHECK(close(server_write_));
  }

 private:
  void CreatePair(int *server, int *client) {
    *server = PCHECK(socket(AF_INET, SOCK_DGRAM, 0));
    {
      sockaddr_in server_address;
      memset(&server_address, 0, sizeof(server_address));
      server_address.sin_family = AF_INET;
      server_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      // server_address.sin_port = htons(server_ + 3000);
      PCHECK(bind(*server, reinterpret_cast<sockaddr *>(&server_address),
                  sizeof(server_address)));
    }
    *client = PCHECK(socket(AF_INET, SOCK_DGRAM, 0));
    {
      sockaddr_in client_address;
      unsigned int length = sizeof(client_address);
      PCHECK(getsockname(*server, reinterpret_cast<sockaddr *>(&client_address),
                         &length));
      PCHECK(connect(*client, reinterpret_cast<sockaddr *>(&client_address),
                     length));
    }
  }

  int server_read_, server_write_, client_read_, client_write_;
};

// Implements ping-pong without copying the data using a condition variable-like
// interface.
class ConditionVariablePingPonger : public StaticPingPonger {
 protected:
  // Represents a condition variable bundled with a mutex.
  //
  // Wait may return spuriously.
  class ConditionVariableInterface {
   public:
    virtual ~ConditionVariableInterface() {}

    // Locks the mutex.
    virtual void Lock() = 0;

    // Unlocks the mutex.
    virtual void Unlock() = 0;

    // Waits on the condition variable.
    //
    // The mutex must be locked when this is called.
    virtual void Wait() = 0;

    // Signals the condition variable.
    //
    // The mutex does not have to be locked during this.
    virtual void Signal() = 0;
  };

  ConditionVariablePingPonger(
      ::std::unique_ptr<ConditionVariableInterface> ping,
      ::std::unique_ptr<ConditionVariableInterface> pong)
      : ping_(::std::move(ping)), pong_(::std::move(pong)) {}

 private:
  const Data *Ping() override {
    ping_->Lock();
    to_server_ = PingData();
    ping_->Unlock();
    ping_->Signal();
    pong_->Lock();
    while (from_server_ == nullptr) {
      pong_->Wait();
    }
    const Data *r = from_server_;
    from_server_ = nullptr;
    pong_->Unlock();
    return r;
  }

  const Data *Wait() override {
    ping_->Lock();
    while (to_server_ == nullptr) {
      ping_->Wait();
    }
    const Data *r = to_server_;
    to_server_ = nullptr;
    ping_->Unlock();
    return r;
  }

  void Pong() override {
    pong_->Lock();
    from_server_ = PongData();
    pong_->Unlock();
    pong_->Signal();
  }

  const Data *to_server_ = nullptr, *from_server_ = nullptr;
  const ::std::unique_ptr<ConditionVariableInterface> ping_, pong_;
};

// Implements ping-pong without copying the data using a semaphore-like
// interface.
class SemaphorePingPonger : public StaticPingPonger {
 protected:
  // Represents a semaphore, which need only count to 1.
  //
  // The behavior when calling Get/Put in anything other than alternating order
  // is undefined.
  //
  // Wait may NOT return spuriously.
  class SemaphoreInterface {
   public:
    virtual ~SemaphoreInterface() {}

    virtual void Get() = 0;
    virtual void Put() = 0;
  };

  SemaphorePingPonger(::std::unique_ptr<SemaphoreInterface> ping,
                      ::std::unique_ptr<SemaphoreInterface> pong)
      : ping_(::std::move(ping)), pong_(::std::move(pong)) {}

 private:
  const Data *Ping() override {
    to_server_ = PingData();
    ping_->Put();
    pong_->Get();
    return from_server_;
  }

  const Data *Wait() override {
    ping_->Get();
    return to_server_;
  }

  void Pong() override {
    from_server_ = PongData();
    pong_->Put();
  }

  const Data *to_server_ = nullptr, *from_server_ = nullptr;
  const ::std::unique_ptr<SemaphoreInterface> ping_, pong_;
};


class AOSMutexPingPonger : public ConditionVariablePingPonger {
 public:
  AOSMutexPingPonger()
      : ConditionVariablePingPonger(
            ::std::unique_ptr<ConditionVariableInterface>(
                new AOSConditionVariable()),
            ::std::unique_ptr<ConditionVariableInterface>(
                new AOSConditionVariable())) {}

 private:
  class AOSConditionVariable : public ConditionVariableInterface {
   public:
    AOSConditionVariable() : condition_(&mutex_) {}

   private:
    void Lock() override { CHECK(!mutex_.Lock()); }
    void Unlock() override { mutex_.Unlock(); }
    void Wait() override { CHECK(!condition_.Wait()); }
    void Signal() override { condition_.Broadcast(); }

    Mutex mutex_;
    Condition condition_;
  };
};

class AOSEventPingPonger : public SemaphorePingPonger {
 public:
  AOSEventPingPonger()
      : SemaphorePingPonger(
            ::std::unique_ptr<SemaphoreInterface>(
                new AOSEventSemaphore()),
            ::std::unique_ptr<SemaphoreInterface>(
                new AOSEventSemaphore())) {}

 private:
  class AOSEventSemaphore : public SemaphoreInterface {
   private:
    void Get() override {
      event_.Wait();
      event_.Clear();
    }
    void Put() override { event_.Set(); }

    Event event_;
  };
};

class PthreadMutexPingPonger : public ConditionVariablePingPonger {
 public:
  PthreadMutexPingPonger()
      : ConditionVariablePingPonger(
            ::std::unique_ptr<ConditionVariableInterface>(
                new PthreadConditionVariable()),
            ::std::unique_ptr<ConditionVariableInterface>(
                new PthreadConditionVariable())) {}

 private:
  class PthreadConditionVariable : public ConditionVariableInterface {
   public:
    PthreadConditionVariable() {
      PRCHECK(pthread_cond_init(&condition_, nullptr));
      PRCHECK(pthread_mutex_init(&mutex_, nullptr));
    }
    ~PthreadConditionVariable() {
      PRCHECK(pthread_mutex_destroy(&mutex_));
      PRCHECK(pthread_cond_destroy(&condition_));
    }

   private:
    void Lock() override { PRCHECK(pthread_mutex_lock(&mutex_)); }
    void Unlock() override { PRCHECK(pthread_mutex_unlock(&mutex_)); }
    void Wait() override { PRCHECK(pthread_cond_wait(&condition_, &mutex_)); }
    void Signal() override { PRCHECK(pthread_cond_broadcast(&condition_)); }

    pthread_cond_t condition_;
    pthread_mutex_t mutex_;
  };
};

class EventFDPingPonger : public SemaphorePingPonger {
 public:
  EventFDPingPonger()
      : SemaphorePingPonger(
            ::std::unique_ptr<SemaphoreInterface>(new EventFDSemaphore()),
            ::std::unique_ptr<SemaphoreInterface>(new EventFDSemaphore())) {}

 private:
  class EventFDSemaphore : public SemaphoreInterface {
   public:
    EventFDSemaphore() : fd_(PCHECK(eventfd(0, 0))) {}
    ~EventFDSemaphore() { PCHECK(close(fd_)); }

   private:
    void Get() override {
      uint64_t value;
      if (read(fd_, &value, sizeof(value)) != sizeof(value)) {
        PLOG(FATAL, "reading from eventfd %d failed\n", fd_);
      }
      CHECK_EQ(1u, value);
    }
    void Put() override {
      uint64_t value = 1;
      if (write(fd_, &value, sizeof(value)) != sizeof(value)) {
        PLOG(FATAL, "writing to eventfd %d failed\n", fd_);
      }
    }

    const int fd_;
  };
};

class SysvSemaphorePingPonger : public SemaphorePingPonger {
 public:
  SysvSemaphorePingPonger()
      : SemaphorePingPonger(
            ::std::unique_ptr<SemaphoreInterface>(new SysvSemaphore()),
            ::std::unique_ptr<SemaphoreInterface>(new SysvSemaphore())) {}

 private:
  class SysvSemaphore : public SemaphoreInterface {
   public:
    SysvSemaphore()
        : sem_id_(PCHECK(semget(IPC_PRIVATE, 1, 0600))) {}

   private:
    void Get() override {
      struct sembuf op;
      op.sem_num = 0;
      op.sem_op = -1;
      op.sem_flg = 0;
      PCHECK(semop(sem_id_, &op, 1));
    }
    void Put() override {
      struct sembuf op;
      op.sem_num = 0;
      op.sem_op = 1;
      op.sem_flg = 0;
      PCHECK(semop(sem_id_, &op, 1));
    }

    const int sem_id_;
  };
};

class PosixSemaphorePingPonger : public SemaphorePingPonger {
 protected:
  PosixSemaphorePingPonger(sem_t *ping_sem, sem_t *pong_sem)
      : SemaphorePingPonger(
            ::std::unique_ptr<SemaphoreInterface>(new PosixSemaphore(ping_sem)),
            ::std::unique_ptr<SemaphoreInterface>(
                new PosixSemaphore(pong_sem))) {}

 private:
  class PosixSemaphore : public SemaphoreInterface {
   public:
    PosixSemaphore(sem_t *sem)
        : sem_(sem) {}

   private:
    void Get() override { PCHECK(sem_wait(sem_)); }
    void Put() override { PCHECK(sem_post(sem_)); }

    sem_t *const sem_;
  };
};

class SysvQueuePingPonger : public StaticPingPonger {
 public:
  SysvQueuePingPonger()
      : ping_(PCHECK(msgget(IPC_PRIVATE, 0600))),
        pong_(PCHECK(msgget(IPC_PRIVATE, 0600))) {}

  const Data *Ping() override {
    {
      Message to_send;
      memcpy(&to_send.data, PingData(), sizeof(Data));
      PCHECK(msgsnd(ping_, &to_send, sizeof(Data), 0));
    }
    {
      Message received;
      PCHECK(msgrcv(pong_, &received, sizeof(Data), 1, 0));
      memcpy(&pong_received_, &received.data, sizeof(Data));
    }
    return &pong_received_;
  }

  const Data *Wait() override {
    {
      Message received;
      PCHECK(msgrcv(ping_, &received, sizeof(Data), 1, 0));
      memcpy(&ping_received_, &received.data, sizeof(Data));
    }
    return &ping_received_;
  }

  virtual void Pong() override {
    Message to_send;
    memcpy(&to_send.data, PongData(), sizeof(Data));
    PCHECK(msgsnd(pong_, &to_send, sizeof(Data), 0));
  }

 private:
  struct Message {
    long mtype = 1;
    char data[sizeof(Data)];
  };

  Data ping_received_, pong_received_;

  const int ping_, pong_;
};

class PosixQueuePingPonger : public StaticPingPonger {
 public:
  PosixQueuePingPonger() : ping_(Open("/ping")), pong_(Open("/pong")) {}
  ~PosixQueuePingPonger() {
    PCHECK(mq_close(ping_));
    PCHECK(mq_close(pong_));
  }

  const Data *Ping() override {
    PCHECK(mq_send(ping_, static_cast<char *>(static_cast<void *>(PingData())),
                   sizeof(Data), 1));
    PCHECK(mq_receive(pong_,
                      static_cast<char *>(static_cast<void *>(&pong_received_)),
                      sizeof(Data), nullptr));
    return &pong_received_;
  }

  const Data *Wait() override {
    PCHECK(mq_receive(ping_,
                      static_cast<char *>(static_cast<void *>(&ping_received_)),
                      sizeof(Data), nullptr));
    return &ping_received_;
  }

  virtual void Pong() override {
    PCHECK(mq_send(pong_, static_cast<char *>(static_cast<void *>(PongData())),
                   sizeof(Data), 1));
  }

 private:
  mqd_t Open(const char *name) {
    if (mq_unlink(name) == -1 && errno != ENOENT) {
      PLOG(FATAL, "mq_unlink(%s) failed", name);
    }
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 1;
    attr.mq_msgsize = sizeof(Data);
    attr.mq_curmsgs = 0;
    const mqd_t r = mq_open(name, O_CREAT | O_RDWR | O_EXCL, 0600, &attr);
    if (r == reinterpret_cast<mqd_t>(-1)) {
      PLOG(FATAL, "mq_open(%s, O_CREAT | O_RDWR | O_EXCL) failed", name);
    }
    return r;
  }

  const mqd_t ping_, pong_;
  Data ping_received_, pong_received_;
};

class PosixUnnamedSemaphorePingPonger : public PosixSemaphorePingPonger {
 public:
  PosixUnnamedSemaphorePingPonger(int pshared)
      : PosixSemaphorePingPonger(&ping_sem_, &pong_sem_) {
    PCHECK(sem_init(&ping_sem_, pshared, 0));
    PCHECK(sem_init(&pong_sem_, pshared, 0));
  }
  ~PosixUnnamedSemaphorePingPonger() {
    PCHECK(sem_destroy(&ping_sem_));
    PCHECK(sem_destroy(&pong_sem_));
  }

 private:
  sem_t ping_sem_, pong_sem_;
};

class PosixNamedSemaphorePingPonger : public PosixSemaphorePingPonger {
 public:
  PosixNamedSemaphorePingPonger()
      : PosixSemaphorePingPonger(ping_sem_ = Open("/ping"),
                                 pong_sem_ = Open("/pong")) {}
  ~PosixNamedSemaphorePingPonger() {
    PCHECK(sem_close(ping_sem_));
    PCHECK(sem_close(pong_sem_));
  }

 private:
  sem_t *Open(const char *name) {
    if (sem_unlink(name) == -1 && errno != ENOENT) {
      PLOG(FATAL, "shm_unlink(%s) failed", name);
    }
    sem_t *const r = sem_open(name, O_CREAT | O_RDWR | O_EXCL, 0600, 0);
    if (r == SEM_FAILED) {
      PLOG(FATAL, "sem_open(%s, O_CREAT | O_RDWR | O_EXCL) failed", name);
    }
    return r;
  }

  sem_t *ping_sem_, *pong_sem_;
};

class AOSQueuePingPonger : public PingPongerInterface {
 public:
  AOSQueuePingPonger()
      : ping_queue_(RawQueue::Fetch("ping", sizeof(Data), 0, 1)),
        pong_queue_(RawQueue::Fetch("pong", sizeof(Data), 0, 1)) {}

  Data *PingData() override {
    CHECK_EQ(nullptr, ping_to_send_);
    ping_to_send_ = static_cast<Data *>(ping_queue_->GetMessage());
    return ping_to_send_;
  }

  const Data *Ping() override {
    CHECK_NE(nullptr, ping_to_send_);
    CHECK(ping_queue_->WriteMessage(ping_to_send_, RawQueue::kBlock));
    ping_to_send_ = nullptr;
    pong_queue_->FreeMessage(pong_received_);
    pong_received_ =
        static_cast<const Data *>(pong_queue_->ReadMessage(RawQueue::kBlock));
    return pong_received_;
  }

  const Data *Wait() override {
    ping_queue_->FreeMessage(ping_received_);
    ping_received_ =
        static_cast<const Data *>(ping_queue_->ReadMessage(RawQueue::kBlock));
    return ping_received_;
  }

  Data *PongData() override {
    CHECK_EQ(nullptr, pong_to_send_);
    pong_to_send_ = static_cast<Data *>(pong_queue_->GetMessage());
    return pong_to_send_;
  }

  void Pong() override {
    CHECK_NE(nullptr, pong_to_send_);
    CHECK(pong_queue_->WriteMessage(pong_to_send_, RawQueue::kBlock));
    pong_to_send_ = nullptr;
  }

 private:
  RawQueue *const ping_queue_;
  RawQueue *const pong_queue_;

  Data *ping_to_send_ = nullptr, *pong_to_send_ = nullptr;
  const Data *ping_received_ = nullptr, *pong_received_ = nullptr;
};

int Main(int /*argc*/, char **argv) {
  ::std::unique_ptr<PingPongerInterface> ping_ponger;
  if (FLAGS_method == "pipe") {
    ping_ponger.reset(new PipePingPonger());
  } else if (FLAGS_method == "named_pipe") {
    ping_ponger.reset(new NamedPipePingPonger());
  } else if (FLAGS_method == "aos_mutex") {
    ping_ponger.reset(new AOSMutexPingPonger());
  } else if (FLAGS_method == "aos_event") {
    ping_ponger.reset(new AOSEventPingPonger());
  } else if (FLAGS_method == "pthread_mutex") {
    ping_ponger.reset(new PthreadMutexPingPonger());
  } else if (FLAGS_method == "aos_queue") {
    ping_ponger.reset(new AOSQueuePingPonger());
  } else if (FLAGS_method == "eventfd") {
    ping_ponger.reset(new EventFDPingPonger());
  } else if (FLAGS_method == "sysv_semaphore") {
    ping_ponger.reset(new SysvSemaphorePingPonger());
  } else if (FLAGS_method == "sysv_queue") {
    ping_ponger.reset(new SysvQueuePingPonger());
  } else if (FLAGS_method == "posix_semaphore_unnamed_shared") {
    ping_ponger.reset(new PosixUnnamedSemaphorePingPonger(1));
  } else if (FLAGS_method == "posix_semaphore_unnamed_unshared") {
    ping_ponger.reset(new PosixUnnamedSemaphorePingPonger(0));
  } else if (FLAGS_method == "posix_semaphore_named") {
    ping_ponger.reset(new PosixNamedSemaphorePingPonger());
  } else if (FLAGS_method == "posix_queue") {
    ping_ponger.reset(new PosixQueuePingPonger());
  } else if (FLAGS_method == "unix_stream") {
    ping_ponger.reset(new UnixPingPonger(SOCK_STREAM));
  } else if (FLAGS_method == "unix_datagram") {
    ping_ponger.reset(new UnixPingPonger(SOCK_DGRAM));
  } else if (FLAGS_method == "unix_seqpacket") {
    ping_ponger.reset(new UnixPingPonger(SOCK_SEQPACKET));
  } else if (FLAGS_method == "tcp") {
    ping_ponger.reset(new TCPPingPonger());
  } else if (FLAGS_method == "udp") {
    ping_ponger.reset(new UDPPingPonger());
  } else {
    fprintf(stderr, "Unknown IPC method to test '%s'\n", FLAGS_method.c_str());
    ::gflags::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  bool done = false;

  ::std::thread server([&ping_ponger, &done]() {
    if (FLAGS_server_priority > 0) {
      SetCurrentThreadRealtimePriority(FLAGS_server_priority);
    }
    PinCurrentThreadToCPU(FLAGS_server_cpu);

    while (!done) {
      const PingPongerInterface::Data *data = ping_ponger->Wait();
      PingPongerInterface::Data *response = ping_ponger->PongData();
      for (size_t i = 0; i < sizeof(*data); ++i) {
        (*response)[i] = (*data)[i] + 1;
      }
      ping_ponger->Pong();
    }
  });

  if (FLAGS_client_priority > 0) {
    SetCurrentThreadRealtimePriority(FLAGS_client_priority);
  }
  PinCurrentThreadToCPU(FLAGS_client_cpu);

  // Warm everything up.
  for (int i = 0; i < 1000; ++i) {
    PingPongerInterface::Data *warmup_data = ping_ponger->PingData();
    memset(*warmup_data, i % 255, sizeof(*warmup_data));
    ping_ponger->Ping();
  }

  const time::Time start = time::Time::Now();

  for (int32_t i = 0; i < FLAGS_messages; ++i) {
    PingPongerInterface::Data *to_send = ping_ponger->PingData();
    memset(*to_send, i % 123, sizeof(*to_send));
    const PingPongerInterface::Data *received = ping_ponger->Ping();
    for (size_t ii = 0; ii < sizeof(*received); ++ii) {
      CHECK_EQ(((i % 123) + 1) % 255, (*received)[ii]);
    }
  }

  const time::Time end = time::Time::Now();

  done = true;
  ping_ponger->PingData();
  ping_ponger->Ping();
  server.join();

  LOG(INFO, "Took %f seconds to send %" PRId32 " messages\n",
      (end - start).ToSeconds(), FLAGS_messages);
  const time::Time per_message = (end - start) / FLAGS_messages;
  if (per_message.sec() > 0) {
    LOG(INFO, "More than 1 second per message ?!?\n");
  } else {
    LOG(INFO, "That is %" PRId32 " nanoseconds per message\n",
        per_message.nsec());
  }

  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  ::gflags::SetUsageMessage(
      ::std::string("Compares various forms of IPC. Usage:\n") + argv[0] +
      " --method=METHOD\n"
      "METHOD can be one of the following:\n"
      "\tpipe\n"
      "\tnamed_pipe\n"
      "\taos_mutex\n"
      "\taos_event\n"
      "\tpthread_mutex\n"
      "\taos_queue\n"
      "\teventfd\n"
      "\tsysv_semaphore\n"
      "\tsysv_queue\n"
      "\tposix_semaphore_unnamed_shared\n"
      "\tposix_semaphore_unnamed_unshared\n"
      "\tposix_semaphore_named\n"
      "\tposix_queue\n"
      "\tunix_stream\n"
      "\tunix_datagram\n"
      "\tunix_seqpacket\n"
      "\ttcp\n"
      "\tudp\n");
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  ::aos::InitNRT();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  return ::aos::Main(argc, argv);
}
