#include "aos/ipc_lib/lockless_queue.h"

#include <linux/futex.h>
#include <sys/types.h>
#include <syscall.h>
#include <unistd.h>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "aos/ipc_lib/lockless_queue_memory.h"
#include "aos/realtime.h"
#include "aos/util/compiler_memory_barrier.h"
#include "glog/logging.h"

namespace aos {
namespace ipc_lib {
namespace {

class GrabQueueSetupLockOrDie {
 public:
  GrabQueueSetupLockOrDie(LocklessQueueMemory *memory) : memory_(memory) {
    const int result = mutex_grab(&(memory->queue_setup_lock));
    CHECK(result == 0 || result == 1) << ": " << result;
  }

  ~GrabQueueSetupLockOrDie() { mutex_unlock(&(memory_->queue_setup_lock)); }

  GrabQueueSetupLockOrDie(const GrabQueueSetupLockOrDie &) = delete;
  GrabQueueSetupLockOrDie &operator=(const GrabQueueSetupLockOrDie &) = delete;

 private:
  LocklessQueueMemory *const memory_;
};

void Cleanup(LocklessQueueMemory *memory, const GrabQueueSetupLockOrDie &) {
  // Make sure we start looking at shared memory fresh right now. We'll handle
  // people dying partway through by either cleaning up after them or not, but
  // we want to ensure we clean up after anybody who has already died when we
  // start.
  aos_compiler_memory_barrier();

  const size_t num_senders = memory->num_senders();
  const size_t queue_size = memory->queue_size();
  const size_t num_messages = memory->num_messages();

  // There are a large number of crazy cases here for how things can go wrong
  // and how we have to recover.  They either require us to keep extra track of
  // what is going on, slowing down the send path, or require a large number of
  // cases.
  //
  // The solution here is to not over-think it.  This is running while not real
  // time during construction.  It is allowed to be slow.  It will also very
  // rarely trigger.  There is a small uS window where process death is
  // ambiguous.
  //
  // So, build up a list N long, where N is the number of messages.  Search
  // through the entire queue and the sender list (ignoring any dead senders),
  // and mark down which ones we have seen.  Once we have seen all the messages
  // except the N dead senders, we know which messages are dead.  Because the
  // queue is active while we do this, it may take a couple of go arounds to see
  // everything.

  // Do the easy case.  Find all senders who have died.  See if they are either
  // consistent already, or if they have copied over to_replace to the scratch
  // index, but haven't cleared to_replace.  Count them.
  size_t valid_senders = 0;
  for (size_t i = 0; i < num_senders; ++i) {
    Sender *sender = memory->GetSender(i);
    const uint32_t tid =
        __atomic_load_n(&(sender->tid.futex), __ATOMIC_ACQUIRE);
    if (tid & FUTEX_OWNER_DIED) {
      VLOG(3) << "Found an easy death for sender " << i;
      // We can do a relaxed load here because we're the only person touching
      // this sender at this point.
      const Index to_replace = sender->to_replace.RelaxedLoad();
      const Index scratch_index = sender->scratch_index.Load();

      // I find it easiest to think about this in terms of the set of observable
      // states.  The main code progresses through the following states:

      // 1) scratch_index = xxx
      //    to_replace = invalid
      // This is unambiguous.  Already good.

      // 2) scratch_index = xxx
      //    to_replace = yyy
      // Very ambiguous.  Is xxx or yyy the correct one?  Need to either roll
      // this forwards or backwards.

      // 3) scratch_index = yyy
      //    to_replace = yyy
      // We are in the act of moving to_replace to scratch_index, but didn't
      // finish.  Easy.

      // 4) scratch_index = yyy
      //    to_replace = invalid
      // Finished, but died.  Looks like 1)

      // Any cleanup code needs to follow the same set of states to be robust to
      // death, so death can be restarted.

      // Could be 2) or 3).
      if (to_replace.valid()) {
        // 3)
        if (to_replace == scratch_index) {
          // Just need to invalidate to_replace to finish.
          sender->to_replace.Invalidate();

          // And mark that we succeeded.
          __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);
          ++valid_senders;
        }
      } else {
        // 1) or 4).  Make sure we aren't corrupted and declare victory.
        CHECK(scratch_index.valid());

        __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);
        ++valid_senders;
      }
    } else {
      // Not dead.
      ++valid_senders;
    }
  }

  // If all the senders are (or were made) good, there is no need to do the hard
  // case.
  if (valid_senders == num_senders) {
    return;
  }

  VLOG(3) << "Starting hard cleanup";

  size_t num_accounted_for = 0;
  size_t num_missing = 0;
  ::std::vector<bool> accounted_for(num_messages, false);

  while ((num_accounted_for + num_missing) != num_messages) {
    num_missing = 0;
    for (size_t i = 0; i < num_senders; ++i) {
      Sender *const sender = memory->GetSender(i);
      const uint32_t tid =
          __atomic_load_n(&(sender->tid.futex), __ATOMIC_ACQUIRE);
      if (tid & FUTEX_OWNER_DIED) {
        ++num_missing;
      } else {
        // We can do a relaxed load here because we're the only person touching
        // this sender at this point, if it matters. If it's not a dead sender,
        // then any message it every has will already be accounted for, so this
        // will always be a NOP.
        const Index scratch_index = sender->scratch_index.RelaxedLoad();
        if (!accounted_for[scratch_index.message_index()]) {
          ++num_accounted_for;
        }
        accounted_for[scratch_index.message_index()] = true;
      }
    }

    for (size_t i = 0; i < queue_size; ++i) {
      // Same logic as above for scratch_index applies here too.
      const Index index = memory->GetQueue(i)->RelaxedLoad();
      if (!accounted_for[index.message_index()]) {
        ++num_accounted_for;
      }
      accounted_for[index.message_index()] = true;
    }
  }

  while (num_missing != 0) {
    const size_t starting_num_missing = num_missing;
    for (size_t i = 0; i < num_senders; ++i) {
      Sender *sender = memory->GetSender(i);
      const uint32_t tid =
          __atomic_load_n(&(sender->tid.futex), __ATOMIC_ACQUIRE);
      if (tid & FUTEX_OWNER_DIED) {
        // We can do relaxed loads here because we're the only person touching
        // this sender at this point.
        const Index scratch_index = sender->scratch_index.RelaxedLoad();
        const Index to_replace = sender->to_replace.RelaxedLoad();

        // Candidate.
        CHECK_LE(to_replace.message_index(), accounted_for.size());
        if (accounted_for[to_replace.message_index()]) {
          VLOG(3) << "Sender " << i
                  << " died, to_replace is already accounted for";
          // If both are accounted for, we are corrupt...
          CHECK(!accounted_for[scratch_index.message_index()]);

          // to_replace is already accounted for.  This means that we didn't
          // atomically insert scratch_index into the queue yet.  So
          // invalidate to_replace.
          sender->to_replace.Invalidate();

          // And then mark this sender clean.
          __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);

          // And account for scratch_index.
          accounted_for[scratch_index.message_index()] = true;
          --num_missing;
          ++num_accounted_for;
        } else if (accounted_for[scratch_index.message_index()]) {
          VLOG(3) << "Sender " << i
                  << " died, scratch_index is already accounted for";
          // scratch_index is accounted for.  That means we did the insert,
          // but didn't record it.
          CHECK(to_replace.valid());
          // Finish the transaction.  Copy to_replace, then clear it.

          sender->scratch_index.Store(to_replace);
          sender->to_replace.Invalidate();

          // And then mark this sender clean.
          __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);

          // And account for to_replace.
          accounted_for[to_replace.message_index()] = true;
          --num_missing;
          ++num_accounted_for;
        } else {
          VLOG(3) << "Sender " << i << " died, neither is accounted for";
          // Ambiguous.  There will be an unambiguous one somewhere that we
          // can do first.
        }
      }
    }
    // CHECK that we are making progress.
    CHECK_NE(num_missing, starting_num_missing);
  }
}

// Exposes rt_tgsigqueueinfo so we can send the signal *just* to the target
// thread.
// TODO(Brian): Do directly in assembly for armhf at least for efficiency.
int rt_tgsigqueueinfo(pid_t tgid, pid_t tid, int sig, siginfo_t *si) {
  return syscall(SYS_rt_tgsigqueueinfo, tgid, tid, sig, si);
}

}  // namespace

size_t LocklessQueueConfiguration::message_size() const {
  // Round up the message size so following data is aligned appropriately.
  return LocklessQueueMemory::AlignmentRoundUp(message_data_size) +
         sizeof(Message);
}

size_t LocklessQueueMemorySize(LocklessQueueConfiguration config) {
  // Round up the message size so following data is aligned appropriately.
  config.message_data_size =
      LocklessQueueMemory::AlignmentRoundUp(config.message_data_size);

  // As we build up the size, confirm that everything is aligned to the
  // alignment requirements of the type.
  size_t size = sizeof(LocklessQueueMemory);
  CHECK_EQ(size % alignof(LocklessQueueMemory), 0u);

  CHECK_EQ(size % alignof(AtomicIndex), 0u);
  size += LocklessQueueMemory::SizeOfQueue(config);

  CHECK_EQ(size % alignof(Message), 0u);
  size += LocklessQueueMemory::SizeOfMessages(config);

  CHECK_EQ(size % alignof(Watcher), 0u);
  size += LocklessQueueMemory::SizeOfWatchers(config);

  CHECK_EQ(size % alignof(Sender), 0u);
  size += LocklessQueueMemory::SizeOfSenders(config);

  return size;
}

LocklessQueueMemory *InitializeLocklessQueueMemory(
    LocklessQueueMemory *memory, LocklessQueueConfiguration config) {
  // Everything should be zero initialized already.  So we just need to fill
  // everything out properly.

  // Grab the mutex.  We don't care if the previous reader died.  We are going
  // to check everything anyways.
  GrabQueueSetupLockOrDie grab_queue_setup_lock(memory);

  if (!memory->initialized) {
    // TODO(austin): Check these for out of bounds.
    memory->config.num_watchers = config.num_watchers;
    memory->config.num_senders = config.num_senders;
    memory->config.queue_size = config.queue_size;
    memory->config.message_data_size = config.message_data_size;

    const size_t num_messages = memory->num_messages();
    // There need to be at most MaxMessages() messages allocated.
    CHECK_LE(num_messages, Index::MaxMessages());

    for (size_t i = 0; i < num_messages; ++i) {
      memory->GetMessage(Index(QueueIndex::Zero(memory->queue_size()), i))
          ->header.queue_index.Invalidate();
    }

    for (size_t i = 0; i < memory->queue_size(); ++i) {
      // Make the initial counter be the furthest away number.  That means that
      // index 0 should be 0xffff, 1 should be 0, etc.
      memory->GetQueue(i)->Store(Index(QueueIndex::Zero(memory->queue_size())
                                           .IncrementBy(i)
                                           .DecrementBy(memory->queue_size()),
                                       i));
    }

    memory->next_queue_index.Invalidate();

    for (size_t i = 0; i < memory->num_senders(); ++i) {
      ::aos::ipc_lib::Sender *s = memory->GetSender(i);
      // Nobody else can possibly be touching these because we haven't set
      // initialized to true yet.
      s->scratch_index.RelaxedStore(Index(0xffff, i + memory->queue_size()));
      s->to_replace.RelaxedInvalidate();
    }

    aos_compiler_memory_barrier();
    // Signal everything is done.  This needs to be done last, so if we die, we
    // redo initialization.
    memory->initialized = true;
  }

  return memory;
}

LocklessQueue::LocklessQueue(LocklessQueueMemory *memory,
                             LocklessQueueConfiguration config)
    : memory_(InitializeLocklessQueueMemory(memory, config)),
      watcher_copy_(memory_->num_watchers()),
      pid_(getpid()),
      uid_(getuid()) {}

LocklessQueue::~LocklessQueue() {
  CHECK_EQ(watcher_index_, -1);

  GrabQueueSetupLockOrDie grab_queue_setup_lock(memory_);
  const int num_watchers = memory_->num_watchers();
  // Cleanup is cheap. The next user will do it anyways, so no need for us to do
  // anything right now.

  // And confirm that nothing is owned by us.
  for (int i = 0; i < num_watchers; ++i) {
    CHECK(!death_notification_is_held(&(memory_->GetWatcher(i)->tid)));
  }
}

size_t LocklessQueue::QueueSize() const { return memory_->queue_size(); }

bool LocklessQueue::RegisterWakeup(int priority) {
  // TODO(austin): Make sure signal coalescing is turned on.  We don't need
  // duplicates.  That will improve performance under high load.

  // Since everything is self consistent, all we need to do is make sure nobody
  // else is running.  Someone dying will get caught in the generic consistency
  // check.
  GrabQueueSetupLockOrDie grab_queue_setup_lock(memory_);
  const int num_watchers = memory_->num_watchers();

  // Now, find the first empty watcher and grab it.
  CHECK_EQ(watcher_index_, -1);
  for (int i = 0; i < num_watchers; ++i) {
    // If we see a slot the kernel has marked as dead, everything we do reusing
    // it needs to happen-after whatever that process did before dying.
    auto *const futex = &(memory_->GetWatcher(i)->tid.futex);
    const uint32_t tid = __atomic_load_n(futex, __ATOMIC_ACQUIRE);
    if (tid == 0 || (tid & FUTEX_OWNER_DIED)) {
      watcher_index_ = i;
      // Relaxed is OK here because we're the only task going to touch it
      // between here and the write in death_notification_init below (other
      // recovery is blocked by us holding the setup lock).
      __atomic_store_n(futex, 0, __ATOMIC_RELAXED);
      break;
    }
  }

  // Bail if we failed to find an open slot.
  if (watcher_index_ == -1) {
    return false;
  }

  Watcher *w = memory_->GetWatcher(watcher_index_);

  w->pid = getpid();
  w->priority = priority;

  // Grabbing a mutex is a compiler and memory barrier, so nothing before will
  // get rearranged afterwords.
  death_notification_init(&(w->tid));
  return true;
}

void LocklessQueue::UnregisterWakeup() {
  // Since everything is self consistent, all we need to do is make sure nobody
  // else is running.  Someone dying will get caught in the generic consistency
  // check.
  GrabQueueSetupLockOrDie grab_queue_setup_lock(memory_);

  // Make sure we are registered.
  CHECK_NE(watcher_index_, -1);

  // Make sure we still own the slot we are supposed to.
  CHECK(
      death_notification_is_held(&(memory_->GetWatcher(watcher_index_)->tid)));

  // The act of unlocking invalidates the entry.  Invalidate it.
  death_notification_release(&(memory_->GetWatcher(watcher_index_)->tid));
  // And internally forget the slot.
  watcher_index_ = -1;
}

int LocklessQueue::Wakeup(const int current_priority) {
  const size_t num_watchers = memory_->num_watchers();

  CHECK_EQ(watcher_copy_.size(), num_watchers);

  // Grab a copy so it won't change out from underneath us, and we can sort it
  // nicely in C++.
  // Do note that there is still a window where the process can die *after* we
  // read everything.  We will still PI boost and send a signal to the thread in
  // question.  There is no way without pidfd's to close this window, and
  // creating a pidfd is likely not RT.
  for (size_t i = 0; i < num_watchers; ++i) {
    Watcher *w = memory_->GetWatcher(i);
    watcher_copy_[i].tid = __atomic_load_n(&(w->tid.futex), __ATOMIC_RELAXED);
    // Force the load of the TID to come first.
    aos_compiler_memory_barrier();
    watcher_copy_[i].pid = w->pid.load(std::memory_order_relaxed);
    watcher_copy_[i].priority = w->priority.load(std::memory_order_relaxed);

    // Use a priority of -1 to mean an invalid entry to make sorting easier.
    if (watcher_copy_[i].tid & FUTEX_OWNER_DIED || watcher_copy_[i].tid == 0) {
      watcher_copy_[i].priority = -1;
    } else {
      // Ensure all of this happens after we're done looking at the pid+priority
      // in shared memory.
      aos_compiler_memory_barrier();
      if (watcher_copy_[i].tid != static_cast<pid_t>(__atomic_load_n(
                                      &(w->tid.futex), __ATOMIC_RELAXED))) {
        // Confirm that the watcher hasn't been re-used and modified while we
        // read it.  If it has, mark it invalid again.
        watcher_copy_[i].priority = -1;
        watcher_copy_[i].tid = 0;
      }
    }
  }

  // Now sort.
  ::std::sort(watcher_copy_.begin(), watcher_copy_.end(),
              [](const WatcherCopy &a, const WatcherCopy &b) {
                return a.priority > b.priority;
              });

  int count = 0;
  if (watcher_copy_[0].priority != -1) {
    const int max_priority =
        ::std::max(current_priority, watcher_copy_[0].priority);
    // Boost if we are RT and there is a higher priority sender out there.
    // Otherwise we might run into priority inversions.
    if (max_priority > current_priority && current_priority > 0) {
      SetCurrentThreadRealtimePriority(max_priority);
    }

    // Build up the siginfo to send.
    siginfo_t uinfo;
    memset(&uinfo, 0, sizeof(uinfo));

    uinfo.si_code = SI_QUEUE;
    uinfo.si_pid = pid_;
    uinfo.si_uid = uid_;
    uinfo.si_value.sival_int = 0;

    for (const WatcherCopy &watcher_copy : watcher_copy_) {
      // The first -1 priority means we are at the end of the valid list.
      if (watcher_copy.priority == -1) {
        break;
      }

      // Send the signal.  Target just the thread that sent it so that we can
      // support multiple watchers in a process (when someone creates multiple
      // event loops in different threads).
      rt_tgsigqueueinfo(watcher_copy.pid, watcher_copy.tid, kWakeupSignal,
                        &uinfo);

      ++count;
    }

    // Drop back down if we were boosted.
    if (max_priority > current_priority && current_priority > 0) {
      SetCurrentThreadRealtimePriority(current_priority);
    }
  }

  return count;
}

LocklessQueue::Sender::Sender(LocklessQueueMemory *memory) : memory_(memory) {
  GrabQueueSetupLockOrDie grab_queue_setup_lock(memory_);

  // Since we already have the lock, go ahead and try cleaning up.
  Cleanup(memory_, grab_queue_setup_lock);

  const int num_senders = memory_->num_senders();

  for (int i = 0; i < num_senders; ++i) {
    ::aos::ipc_lib::Sender *s = memory->GetSender(i);
    // This doesn't need synchronization because we're the only process doing
    // initialization right now, and nobody else will be touching senders which
    // we're interested in.
    const uint32_t tid = __atomic_load_n(&(s->tid.futex), __ATOMIC_RELAXED);
    if (tid == 0) {
      sender_index_ = i;
      break;
    }
  }

  if (sender_index_ == -1) {
    LOG(FATAL) << "Too many senders";
  }

  ::aos::ipc_lib::Sender *s = memory_->GetSender(sender_index_);

  // Indicate that we are now alive by taking over the slot. If the previous
  // owner died, we still want to do this.
  death_notification_init(&(s->tid));
}

LocklessQueue::Sender::~Sender() {
  if (memory_ != nullptr) {
    death_notification_release(&(memory_->GetSender(sender_index_)->tid));
  }
}

LocklessQueue::Sender LocklessQueue::MakeSender() {
  return LocklessQueue::Sender(memory_);
}

QueueIndex ZeroOrValid(QueueIndex index) {
  if (!index.valid()) {
    return index.Clear();
  }
  return index;
}

size_t LocklessQueue::Sender::size() { return memory_->message_data_size(); }

void *LocklessQueue::Sender::Data() {
  ::aos::ipc_lib::Sender *sender = memory_->GetSender(sender_index_);
  Index scratch_index = sender->scratch_index.RelaxedLoad();
  Message *message = memory_->GetMessage(scratch_index);
  message->header.queue_index.Invalidate();

  return &message->data[0];
}

void LocklessQueue::Sender::Send(
    const char *data, size_t length,
    aos::monotonic_clock::time_point monotonic_remote_time,
    aos::realtime_clock::time_point realtime_remote_time,
    uint32_t remote_queue_index,
    aos::monotonic_clock::time_point *monotonic_sent_time,
    aos::realtime_clock::time_point *realtime_sent_time,
    uint32_t *queue_index) {
  CHECK_LE(length, size());
  // Flatbuffers write from the back of the buffer to the front.  If we are
  // going to write an explicit chunk of memory into the buffer, we need to
  // adhere to this convention and place it at the end.
  memcpy((reinterpret_cast<char *>(Data()) + size() - length), data, length);
  Send(length, monotonic_remote_time, realtime_remote_time, remote_queue_index,
       monotonic_sent_time, realtime_sent_time, queue_index);
}

void LocklessQueue::Sender::Send(
    size_t length, aos::monotonic_clock::time_point monotonic_remote_time,
    aos::realtime_clock::time_point realtime_remote_time,
    uint32_t remote_queue_index,
    aos::monotonic_clock::time_point *monotonic_sent_time,
    aos::realtime_clock::time_point *realtime_sent_time,
    uint32_t *queue_index) {
  const size_t queue_size = memory_->queue_size();
  CHECK_LE(length, size());

  ::aos::ipc_lib::Sender *const sender = memory_->GetSender(sender_index_);
  // We can do a relaxed load on our sender because we're the only person
  // modifying it right now.
  const Index scratch_index = sender->scratch_index.RelaxedLoad();
  Message *const message = memory_->GetMessage(scratch_index);

  message->header.length = length;
  // Pass these through.  Any alternative behavior can be implemented out a
  // layer.
  message->header.remote_queue_index = remote_queue_index;
  message->header.monotonic_remote_time = monotonic_remote_time;
  message->header.realtime_remote_time = realtime_remote_time;

  while (true) {
    const QueueIndex actual_next_queue_index =
        memory_->next_queue_index.Load(queue_size);
    const QueueIndex next_queue_index = ZeroOrValid(actual_next_queue_index);

    const QueueIndex incremented_queue_index = next_queue_index.Increment();

    // This needs to synchronize with whoever the previous writer at this
    // location was.
    const Index to_replace = memory_->LoadIndex(next_queue_index);

    const QueueIndex decremented_queue_index =
        next_queue_index.DecrementBy(queue_size);

    // See if we got beat.  If we did, try to atomically update
    // next_queue_index in case the previous writer failed and retry.
    if (!to_replace.IsPlausible(decremented_queue_index)) {
      // We don't care about the result.  It will either succeed, or we got
      // beat in fixing it and just need to give up and try again.  If we got
      // beat multiple times, the only way progress can be made is if the queue
      // is updated as well.  This means that if we retry reading
      // next_queue_index, we will be at most off by one and can retry.
      //
      // Both require no further action from us.
      //
      // TODO(austin): If we are having fairness issues under contention, we
      // could have a mode bit in next_queue_index, and could use a lock or some
      // other form of PI boosting to let the higher priority task win.
      memory_->next_queue_index.CompareAndExchangeStrong(
          actual_next_queue_index, incremented_queue_index);

      VLOG(3) << "We were beat.  Try again.  Was " << std::hex
              << to_replace.get() << ", is " << decremented_queue_index.index();
      continue;
    }

    // Confirm that the message is what it should be.
    {
      const QueueIndex previous_index =
          memory_->GetMessage(to_replace)->header.queue_index.Load(queue_size);
      if (previous_index != decremented_queue_index && previous_index.valid()) {
        // Retry.
        VLOG(3) << "Something fishy happened, queue index doesn't match.  "
                   "Retrying.  Previous index was "
                << std::hex << previous_index.index() << ", should be "
                << decremented_queue_index.index();
        continue;
      }
    }

    message->header.monotonic_sent_time = ::aos::monotonic_clock::now();
    message->header.realtime_sent_time = ::aos::realtime_clock::now();
    if (monotonic_sent_time != nullptr) {
      *monotonic_sent_time = message->header.monotonic_sent_time;
    }
    if (realtime_sent_time != nullptr) {
      *realtime_sent_time = message->header.realtime_sent_time;
    }
    if (queue_index != nullptr) {
      *queue_index = next_queue_index.index();
    }

    // Before we are fully done filling out the message, update the Sender state
    // with the new index to write.  This re-uses the barrier for the
    // queue_index store.
    const Index index_to_write(next_queue_index, scratch_index.message_index());

    aos_compiler_memory_barrier();
    // We're the only person who cares about our scratch index, besides somebody
    // cleaning up after us.
    sender->scratch_index.RelaxedStore(index_to_write);
    aos_compiler_memory_barrier();

    message->header.queue_index.Store(next_queue_index);

    aos_compiler_memory_barrier();
    // The message is now filled out, and we have a confirmed slot to store
    // into.
    //
    // Start by writing down what we are going to pull out of the queue.  This
    // was Invalid before now. Only person who will read this is whoever cleans
    // up after us, so no synchronization necessary.
    sender->to_replace.RelaxedStore(to_replace);
    aos_compiler_memory_barrier();

    // Then exchange the next index into the queue.
    if (!memory_->GetQueue(next_queue_index.Wrapped())
             ->CompareAndExchangeStrong(to_replace, index_to_write)) {
      // Aw, didn't succeed.  Retry.
      sender->to_replace.RelaxedInvalidate();
      aos_compiler_memory_barrier();
      VLOG(3) << "Failed to wrap into queue";
      continue;
    }

    // Then update next_queue_index to save the next user some computation time.
    memory_->next_queue_index.CompareAndExchangeStrong(actual_next_queue_index,
                                                       incremented_queue_index);

    aos_compiler_memory_barrier();
    // Now update the scratch space and record that we succeeded.
    sender->scratch_index.Store(to_replace);
    aos_compiler_memory_barrier();
    // And then record that we succeeded, but definitely after the above store.
    sender->to_replace.RelaxedInvalidate();
    break;
  }
}

LocklessQueue::ReadResult LocklessQueue::Read(
    uint32_t uint32_queue_index,
    ::aos::monotonic_clock::time_point *monotonic_sent_time,
    ::aos::realtime_clock::time_point *realtime_sent_time,
    ::aos::monotonic_clock::time_point *monotonic_remote_time,
    ::aos::realtime_clock::time_point *realtime_remote_time,
    uint32_t *remote_queue_index, size_t *length, char *data) {
  const size_t queue_size = memory_->queue_size();

  // Build up the QueueIndex.
  const QueueIndex queue_index =
      QueueIndex::Zero(queue_size).IncrementBy(uint32_queue_index);

  // Read the message stored at the requested location.
  Index mi = memory_->LoadIndex(queue_index);
  Message *m = memory_->GetMessage(mi);

  while (true) {
    // We need to confirm that the data doesn't change while we are reading it.
    // Do that by first confirming that the message points to the queue index we
    // want.
    const QueueIndex starting_queue_index =
        m->header.queue_index.Load(queue_size);
    if (starting_queue_index != queue_index) {
      // If we found a message that is exactly 1 loop old, we just wrapped.
      if (starting_queue_index == queue_index.DecrementBy(queue_size)) {
        VLOG(3) << "Matches: " << std::hex << starting_queue_index.index()
                << ", " << queue_index.DecrementBy(queue_size).index();
        return ReadResult::NOTHING_NEW;
      } else {
        // Someone has re-used this message between when we pulled it out of the
        // queue and when we grabbed its index.  It is pretty hard to deduce
        // what happened. Just try again.
        Message *const new_m = memory_->GetMessage(queue_index);
        if (m != new_m) {
          m = new_m;
          VLOG(3) << "Retrying, m doesn't match";
          continue;
        }

        // We have confirmed that message still points to the same message. This
        // means that the message didn't get swapped out from under us, so
        // starting_queue_index is correct.
        //
        // Either we got too far behind (signaled by this being a valid
        // message), or this is one of the initial messages which are invalid.
        if (starting_queue_index.valid()) {
          VLOG(3) << "Too old.  Tried for " << std::hex << queue_index.index()
                  << ", got " << starting_queue_index.index() << ", behind by "
                  << std::dec
                  << (starting_queue_index.index() - queue_index.index());
          return ReadResult::TOO_OLD;
        }

        VLOG(3) << "Initial";

        // There isn't a valid message at this location.
        //
        // If someone asks for one of the messages within the first go around,
        // then they need to wait.  They got ahead.  Otherwise, they are
        // asking for something crazy, like something before the beginning of
        // the queue.  Tell them that they are behind.
        if (uint32_queue_index < memory_->queue_size()) {
          VLOG(3) << "Near zero, " << std::hex << uint32_queue_index;
          return ReadResult::NOTHING_NEW;
        } else {
          VLOG(3) << "Not near zero, " << std::hex << uint32_queue_index;
          return ReadResult::TOO_OLD;
        }
      }
    }
    VLOG(3) << "Eq: " << std::hex << starting_queue_index.index() << ", "
            << queue_index.index();
    break;
  }

  // Then read the data out.  Copy it all out to be deterministic and so we can
  // make length be from either end.
  *monotonic_sent_time = m->header.monotonic_sent_time;
  *realtime_sent_time = m->header.realtime_sent_time;
  if (m->header.remote_queue_index == 0xffffffffu) {
    *remote_queue_index = queue_index.index();
  } else {
    *remote_queue_index = m->header.remote_queue_index;
  }
  *monotonic_remote_time = m->header.monotonic_remote_time;
  *realtime_remote_time = m->header.realtime_remote_time;
  memcpy(data, &m->data[0], message_data_size());
  *length = m->header.length;

  // And finally, confirm that the message *still* points to the queue index we
  // want.  This means it didn't change out from under us.
  // If something changed out from under us, we were reading it much too late in
  // it's lifetime.
  aos_compiler_memory_barrier();
  const QueueIndex final_queue_index = m->header.queue_index.Load(queue_size);
  if (final_queue_index != queue_index) {
    VLOG(3) << "Changed out from under us.  Reading " << std::hex
            << queue_index.index() << ", finished with "
            << final_queue_index.index() << ", delta: " << std::dec
            << (final_queue_index.index() - queue_index.index());
    return ReadResult::OVERWROTE;
  }

  return ReadResult::GOOD;
}

size_t LocklessQueue::queue_size() const { return memory_->queue_size(); }
size_t LocklessQueue::message_data_size() const {
  return memory_->message_data_size();
}

QueueIndex LocklessQueue::LatestQueueIndex() {
  const size_t queue_size = memory_->queue_size();

  // There is only one interesting case.  We need to know if the queue is empty.
  // That is done with a sentinel value.  At worst, this will be off by one.
  const QueueIndex next_queue_index =
      memory_->next_queue_index.Load(queue_size);
  if (next_queue_index.valid()) {
    const QueueIndex current_queue_index = next_queue_index.DecrementBy(1u);
    return current_queue_index;
  } else {
    return empty_queue_index();
  }
}

namespace {

// Prints out the mutex state.  Not safe to use while the mutex is being
// changed.
::std::string PrintMutex(aos_mutex *mutex) {
  ::std::stringstream s;
  s << "aos_mutex(" << ::std::hex << mutex->futex;

  if (mutex->futex != 0) {
    s << ":";
    if (mutex->futex & FUTEX_OWNER_DIED) {
      s << "FUTEX_OWNER_DIED|";
    }
    s << "tid=" << (mutex->futex & FUTEX_TID_MASK);
  }

  s << ")";
  return s.str();
}

}  // namespace

void PrintLocklessQueueMemory(LocklessQueueMemory *memory) {
  const size_t queue_size = memory->queue_size();
  ::std::cout << "LocklessQueueMemory (" << memory << ") {" << ::std::endl;
  ::std::cout << "  aos_mutex queue_setup_lock = "
              << PrintMutex(&memory->queue_setup_lock) << ::std::endl;
  ::std::cout << "  bool initialized = " << memory->initialized << ::std::endl;
  ::std::cout << "  config {" << ::std::endl;
  ::std::cout << "    size_t num_watchers = " << memory->config.num_watchers
              << ::std::endl;
  ::std::cout << "    size_t num_senders = " << memory->config.num_senders
              << ::std::endl;
  ::std::cout << "    size_t queue_size = " << memory->config.queue_size
              << ::std::endl;
  ::std::cout << "    size_t message_data_size = "
              << memory->config.message_data_size << ::std::endl;

  ::std::cout << "    AtomicQueueIndex next_queue_index = "
              << memory->next_queue_index.Load(queue_size).DebugString()
              << ::std::endl;

  ::std::cout << "  }" << ::std::endl;
  ::std::cout << "  AtomicIndex queue[" << queue_size << "] {" << ::std::endl;
  for (size_t i = 0; i < queue_size; ++i) {
    ::std::cout << "    [" << i << "] -> "
                << memory->GetQueue(i)->Load().DebugString() << ::std::endl;
  }
  ::std::cout << "  }" << ::std::endl;
  ::std::cout << "  Message messages[" << memory->num_messages() << "] {"
              << ::std::endl;
  for (size_t i = 0; i < memory->num_messages(); ++i) {
    Message *m = memory->GetMessage(Index(i, i));
    ::std::cout << "    [" << i << "] -> Message {" << ::std::endl;
    ::std::cout << "      Header {" << ::std::endl;
    ::std::cout << "        AtomicQueueIndex queue_index = "
                << m->header.queue_index.Load(queue_size).DebugString()
                << ::std::endl;
    ::std::cout << "        size_t length = " << m->header.length
                << ::std::endl;
    ::std::cout << "      }" << ::std::endl;
    ::std::cout << "      data: {";

    for (size_t j = 0; j < m->header.length; ++j) {
      char data = m->data[j];
      if (j != 0) {
        ::std::cout << " ";
      }
      if (::std::isprint(data)) {
        ::std::cout << ::std::setfill(' ') << ::std::setw(2) << ::std::hex
                    << data;
      } else {
        ::std::cout << "0x" << ::std::setfill('0') << ::std::setw(2)
                    << ::std::hex << (static_cast<unsigned>(data) & 0xff);
      }
    }
    ::std::cout << ::std::setfill(' ') << ::std::dec << "}" << ::std::endl;
    ::std::cout << "    }," << ::std::endl;
  }
  ::std::cout << "  }" << ::std::endl;

  ::std::cout << "  Sender senders[" << memory->num_senders() << "] {"
              << ::std::endl;
  for (size_t i = 0; i < memory->num_senders(); ++i) {
    Sender *s = memory->GetSender(i);
    ::std::cout << "    [" << i << "] -> Sender {" << ::std::endl;
    ::std::cout << "      aos_mutex tid = " << PrintMutex(&s->tid)
                << ::std::endl;
    ::std::cout << "      AtomicIndex scratch_index = "
                << s->scratch_index.Load().DebugString() << ::std::endl;
    ::std::cout << "      AtomicIndex to_replace = "
                << s->to_replace.Load().DebugString() << ::std::endl;
    ::std::cout << "    }" << ::std::endl;
  }
  ::std::cout << "  }" << ::std::endl;

  ::std::cout << "  Watcher watchers[" << memory->num_watchers() << "] {"
              << ::std::endl;
  for (size_t i = 0; i < memory->num_watchers(); ++i) {
    Watcher *w = memory->GetWatcher(i);
    ::std::cout << "    [" << i << "] -> Watcher {" << ::std::endl;
    ::std::cout << "      aos_mutex tid = " << PrintMutex(&w->tid)
                << ::std::endl;
    ::std::cout << "      pid_t pid = " << w->pid << ::std::endl;
    ::std::cout << "      int priority = " << w->priority << ::std::endl;
    ::std::cout << "    }" << ::std::endl;
  }
  ::std::cout << "  }" << ::std::endl;

  ::std::cout << "}" << ::std::endl;
}

}  // namespace ipc_lib
}  // namespace aos
