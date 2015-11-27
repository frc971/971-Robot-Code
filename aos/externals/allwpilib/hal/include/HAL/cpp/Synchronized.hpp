/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#pragma once

#include <mutex>
#include "HAL/Semaphore.hpp"

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete;      \
  void operator=(const TypeName&) = delete

class Synchronized;

/**
 * Wrap a pthead mutex for easier use in C++. For a static
 * instance, the constructor runs at program load time before main() can spawn
 * any tasks. Use that to fix race conditions in setup code.
 *
 * This uses a pthread mutex which has PTHREAD_MUTEX_RECURSIVE set, making it
 * "reentrant" in the sense that the owning task can lock it more than once. The
 * task will need to unlock the mutex the same number of times to make it
 * available for other threads to acquire.
 *
 * This class is safe to use in static variables because it does not depend on
 * any other C++ static constructors or destructors.
 */
class ReentrantMutex {
 public:
  typedef pthread_mutex_t *native_handle_type;

  constexpr ReentrantMutex() noexcept = default;
  ReentrantMutex(const ReentrantMutex &) = delete;
  ReentrantMutex &operator=(const ReentrantMutex &) = delete;

  /**
   * Lock the mutex, blocking until it's available.
   */
  void lock() { pthread_mutex_lock(&mutex_); }

  /**
   * Unlock the mutex.
   */
  void unlock() { pthread_mutex_unlock(&mutex_); }

  /**
   * Tries to lock the mutex.
   */
  bool try_lock() noexcept { return !pthread_mutex_trylock(&mutex_); }

  pthread_mutex_t *native_handle() { return &mutex_; }

 private:
  // Do the equivalent of setting PTHREAD_PRIO_INHERIT and
  // PTHREAD_MUTEX_RECURSIVE_NP.
#ifndef __PTHREAD_SPINS
// This is what it should be for old pthreads which doesn't know about lock
// elision at all.
#define __PTHREAD_SPINS 0
#endif
#if __WORDSIZE == 64
  pthread_mutex_t mutex_ = {
      {0, 0, 0, 0, 0x20 | PTHREAD_MUTEX_RECURSIVE_NP, __PTHREAD_SPINS, {0, 0}}};
#else
  pthread_mutex_t mutex_ = {
      {0, 0, 0, 0x20 | PTHREAD_MUTEX_RECURSIVE_NP, 0, {__PTHREAD_SPINS}}};
#endif
};

/**
 * Wrap a pthead mutex for easier use in C++. For a static
 * instance, the constructor runs at program load time before main() can spawn
 * any tasks. Use that to fix race conditions in setup code.
 *
 * This class is safe to use in static variables because it does not depend on
 * any other C++ static constructors or destructors.
 */
class Mutex {
 public:
  typedef pthread_mutex_t *native_handle_type;

  constexpr Mutex() noexcept = default;
  Mutex(const Mutex &) = delete;
  Mutex &operator=(const Mutex &) = delete;

  /**
   * Lock the mutex, blocking until it's available.
   */
  void lock() { pthread_mutex_lock(&mutex_); }

  /**
   * Unlock the mutex.
   */
  void unlock() { pthread_mutex_unlock(&mutex_); }

  /**
   * Tries to lock the mutex.
   */
  bool try_lock() noexcept { return !pthread_mutex_trylock(&mutex_); }

  native_handle_type native_handle() { return &mutex_; }

 private:
  // Do the equivalent of setting PTHREAD_PRIO_INHERIT.
#if __WORDSIZE == 64
  pthread_mutex_t mutex_ = {{0, 0, 0, 0, 0x20, __PTHREAD_SPINS, {0, 0}}};
#else
  pthread_mutex_t mutex_ = {{0, 0, 0, 0x20, 0, {__PTHREAD_SPINS}}};
#endif
};


/**
 * Provide easy support for critical regions.
 *
 * A critical region is an area of code that is always executed under mutual exclusion. Only
 * one task can be executing this code at any time. The idea is that code that manipulates data
 * that is shared between two or more tasks has to be prevented from executing at the same time
 * otherwise a race condition is possible when both tasks try to update the data. Typically
 * semaphores are used to ensure only single task access to the data.
 *
 * Synchronized objects are a simple wrapper around semaphores to help ensure
 * that semaphores are always unlocked (semGive) after locking (semTake).
 *
 * You allocate a Synchronized as a local variable, *not* on the heap. That
 * makes it a "stack object" whose destructor runs automatically when it goes
 * out of scope. E.g.
 *
 *   { Synchronized _sync(aReentrantSemaphore); ... critical region ... }
 */
class Synchronized
{
public:
#ifndef __vxworks
	explicit Synchronized(SEMAPHORE_ID);
#endif
	virtual ~Synchronized();
private:
	SEMAPHORE_ID m_semaphore;

	DISALLOW_COPY_AND_ASSIGN(Synchronized);
};

