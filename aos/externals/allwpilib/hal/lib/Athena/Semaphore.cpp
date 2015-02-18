#include "HAL/Semaphore.hpp"
#include "HAL/cpp/Synchronized.hpp"

#include "Log.hpp"

// set the logging level
TLogLevel semaphoreLogLevel = logDEBUG;

#define SEMAPHORE_LOG(level) \
    if (level > semaphoreLogLevel) ; \
    else Log().Get(level)

// See: http://www.vxdev.com/docs/vx55man/vxworks/ref/semMLib.html
const uint32_t SEMAPHORE_Q_FIFO= 0x01; // TODO: Support
const uint32_t SEMAPHORE_Q_PRIORITY = 0x01; // TODO: Support
const uint32_t SEMAPHORE_DELETE_SAFE = 0x04; // TODO: Support
const uint32_t SEMAPHORE_INVERSION_SAFE = 0x08; // TODO: Support

const int32_t SEMAPHORE_NO_WAIT = 0;
const int32_t SEMAPHORE_WAIT_FOREVER = -1;

const uint32_t SEMAPHORE_EMPTY = 0;
const uint32_t SEMAPHORE_FULL = 1;

class MutexInterface {
 public:
  virtual void lock() = 0;
  virtual void unlock() = 0;
  virtual bool try_lock() = 0;
  virtual pthread_mutex_t *native_handle() = 0;
  virtual ~MutexInterface() {}
};

template <typename MutexType>
class TemplatedMutexWrapper : public MutexInterface {
 public:
  virtual void lock() { m_.lock(); }
  virtual void unlock() { m_.unlock(); }
  virtual bool try_lock() { return m_.try_lock(); }
  pthread_mutex_t *native_handle() { return m_.native_handle(); }

 private:
  MutexType m_;
};

MUTEX_ID initializeMutexRecursive()
{
  return new TemplatedMutexWrapper<ReentrantMutex>();
}

MUTEX_ID initializeMutexNormal()
{
  return new TemplatedMutexWrapper<Mutex>();
}

void deleteMutex(MUTEX_ID mutex)
{
	delete mutex;
}

/**
 * Lock the semaphore, blocking until it's available.
 * @return 0 for success, -1 for error. If -1, the error will be in errno.
 */
int8_t lockMutex(MUTEX_ID mutex)
{
    mutex->lock();
    return 0;
}

/**
 * Tries to lock the mutex.
 * @return 1 if we got the lock, 0 otherwise.
 */
int8_t tryLockMutex(MUTEX_ID mutex)
{
    return mutex->try_lock();
}

/**
 * Unlock the semaphore.
 * @return 0 for success, -1 for error. If -1, the error will be in errno.
 */
int8_t unlockMutex(MUTEX_ID mutex)
{
	mutex->unlock();
  return 0;
}

SEMAPHORE_ID initializeSemaphore(uint32_t initial_value) {
  SEMAPHORE_ID sem = new sem_t;
  sem_init(sem, 0, initial_value);
  return sem;
}

void deleteSemaphore(SEMAPHORE_ID sem) {
  sem_destroy(sem);
  delete sem;
}

/**
 * Lock the semaphore, blocking until it's available.
 * @return 0 for success, -1 for error. If -1, the error will be in errno.
 */
int8_t takeSemaphore(SEMAPHORE_ID sem)
{
    return sem_wait(sem);
}

int8_t tryTakeSemaphore(SEMAPHORE_ID sem)
{
    return sem_trywait(sem);
}

/**
 * Unlock the semaphore.
 * @return 0 for success, -1 for error. If -1, the error will be in errno.
 */
int8_t giveSemaphore(SEMAPHORE_ID sem)
{
  return sem_post(sem);
}


MULTIWAIT_ID initializeMultiWait() {
  pthread_condattr_t attr;
  pthread_condattr_init(&attr);
  MULTIWAIT_ID cond = new pthread_cond_t();
  pthread_cond_init(cond, &attr);
  pthread_condattr_destroy(&attr);
  return cond;
}

void deleteMultiWait(MULTIWAIT_ID sem) {
  pthread_cond_destroy(sem);
  delete sem;
}

int8_t takeMultiWait(MULTIWAIT_ID sem, MUTEX_ID m, int32_t timeout) {
  lockMutex(m);
  int8_t val = pthread_cond_wait(sem, m->native_handle());
  unlockMutex(m);
  return val;
}

int8_t giveMultiWait(MULTIWAIT_ID sem) {
  return pthread_cond_broadcast(sem);
}


