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
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(dump_lockless_queue_data, false,
            "If true, print the data out when dumping the queue.");

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

bool IsPinned(LocklessQueueMemory *memory, Index index) {
  DCHECK(index.valid());
  const size_t queue_size = memory->queue_size();
  const QueueIndex message_index =
      memory->GetMessage(index)->header.queue_index.Load(queue_size);
  if (!message_index.valid()) {
    return false;
  }
  DCHECK(memory->GetQueue(message_index.Wrapped())->Load() != index)
      << ": Message is in the queue";
  for (int pinner_index = 0;
       pinner_index < static_cast<int>(memory->config.num_pinners);
       ++pinner_index) {
    ipc_lib::Pinner *const pinner = memory->GetPinner(pinner_index);

    if (pinner->pinned.RelaxedLoad(queue_size) == message_index) {
      return true;
    }
  }
  return false;
}

// Ensures sender->scratch_index (which must contain to_replace) is not pinned.
//
// Returns the new scratch_index value.
Index SwapPinnedSenderScratch(LocklessQueueMemory *const memory,
                              ipc_lib::Sender *const sender,
                              const Index to_replace) {
  // If anybody's trying to pin this message, then grab a message from a pinner
  // to write into instead, and leave the message we pulled out of the queue
  // (currently in our scratch_index) with a pinner.
  //
  // This loop will terminate in at most one iteration through the pinners in
  // any steady-state configuration of the memory. There are only as many
  // Pinner::pinned values to worry about as there are Pinner::scratch_index
  // values to check against, plus to_replace, which means there will always be
  // a free one. We might have to make multiple passes if things are being
  // changed concurrently though, but nobody dying can make this loop fail to
  // terminate (because the number of processes that can die is bounded, because
  // no new ones can start while we've got the lock).
  for (int pinner_index = 0; true;
       pinner_index = (pinner_index + 1) % memory->config.num_pinners) {
    if (!IsPinned(memory, to_replace)) {
      // No pinners on our current scratch_index, so we're fine now.
      VLOG(3) << "No pinners: " << to_replace.DebugString();
      return to_replace;
    }

    ipc_lib::Pinner *const pinner = memory->GetPinner(pinner_index);

    const Index pinner_scratch = pinner->scratch_index.RelaxedLoad();
    CHECK(pinner_scratch.valid())
        << ": Pinner scratch_index should always be valid";
    if (IsPinned(memory, pinner_scratch)) {
      // Wouldn't do us any good to swap with this one, so don't bother, and
      // move onto the next one.
      VLOG(3) << "Also pinned: " << pinner_scratch.DebugString();
      continue;
    }

    sender->to_replace.RelaxedStore(pinner_scratch);
    aos_compiler_memory_barrier();
    // Give the pinner the message (which is currently in
    // sender->scratch_index).
    if (!pinner->scratch_index.CompareAndExchangeStrong(pinner_scratch,
                                                        to_replace)) {
      // Somebody swapped into this pinner before us. The new value is probably
      // pinned, so we don't want to look at it again immediately.
      VLOG(3) << "Pinner " << pinner_index
              << " scratch_index changed: " << pinner_scratch.DebugString()
              << ", " << to_replace.DebugString();
      sender->to_replace.RelaxedInvalidate();
      continue;
    }
    aos_compiler_memory_barrier();
    // Now update the sender's scratch space and record that we succeeded.
    sender->scratch_index.Store(pinner_scratch);
    aos_compiler_memory_barrier();
    // And then record that we succeeded, but definitely after the above
    // store.
    sender->to_replace.RelaxedInvalidate();
    VLOG(3) << "Got new scratch message: " << pinner_scratch.DebugString();

    // If it's in a pinner's scratch_index, it should not be in the queue, which
    // means nobody new can pin it for real. However, they can still attempt to
    // pin it, which means we can't verify !IsPinned down here.

    return pinner_scratch;
  }
}

// Returns true if it succeeded. Returns false if another sender died in the
// middle.
bool DoCleanup(LocklessQueueMemory *memory, const GrabQueueSetupLockOrDie &) {
  // Make sure we start looking at shared memory fresh right now. We'll handle
  // people dying partway through by either cleaning up after them or not, but
  // we want to ensure we clean up after anybody who has already died when we
  // start.
  aos_compiler_memory_barrier();

  const size_t num_senders = memory->num_senders();
  const size_t num_pinners = memory->num_pinners();
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

  ::std::vector<bool> need_recovery(num_senders, false);

  // Do the easy case.  Find all senders who have died.  See if they are either
  // consistent already, or if they have copied over to_replace to the scratch
  // index, but haven't cleared to_replace.  Count them.
  size_t valid_senders = 0;
  for (size_t i = 0; i < num_senders; ++i) {
    Sender *sender = memory->GetSender(i);
    const uint32_t tid =
        __atomic_load_n(&(sender->tid.futex), __ATOMIC_ACQUIRE);
    if (!(tid & FUTEX_OWNER_DIED)) {
      // Not dead.
      ++valid_senders;
      continue;
    }
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
    //
    // If doing a pinner swap, we've definitely done it.

    // 4) scratch_index = yyy
    //    to_replace = invalid
    // Finished, but died.  Looks like 1)

    // Swapping with a pinner's scratch_index passes through the same states.
    // We just need to ensure the message that ends up in the senders's
    // scratch_index isn't pinned, using the same code as sending does.

    // Any cleanup code needs to follow the same set of states to be robust to
    // death, so death can be restarted.

    if (!to_replace.valid()) {
      // 1) or 4).  Make sure we aren't corrupted and declare victory.
      CHECK(scratch_index.valid());

      // If it's in 1) with a pinner, the sender might have a pinned message,
      // so fix that.
      SwapPinnedSenderScratch(memory, sender, scratch_index);

      // If it's in 4), it may not have completed this step yet. This will
      // always be a NOP if it's in 1), verified by a DCHECK.
      memory->GetMessage(scratch_index)->header.queue_index.RelaxedInvalidate();

      __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);
      ++valid_senders;
      continue;
    }

    // Could be 2) or 3) at this point.

    if (to_replace == scratch_index) {
      // 3) for sure.
      // Just need to invalidate to_replace to finish.
      sender->to_replace.Invalidate();

      // Make sure to indicate it's an unused message before a sender gets its
      // hands on it.
      memory->GetMessage(scratch_index)->header.queue_index.RelaxedInvalidate();
      aos_compiler_memory_barrier();

      // And mark that we succeeded.
      __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);
      ++valid_senders;
      continue;
    }

    // Must be 2). Mark it for later.
    need_recovery[i] = true;
  }

  // Cleaning up pinners is easy. We don't actually have to do anything, but
  // invalidating its pinned field might help catch bugs elsewhere trying to
  // read it before it's set.
  for (size_t i = 0; i < num_pinners; ++i) {
    Pinner *const pinner = memory->GetPinner(i);
    const uint32_t tid =
        __atomic_load_n(&(pinner->tid.futex), __ATOMIC_ACQUIRE);
    if (!(tid & FUTEX_OWNER_DIED)) {
      continue;
    }
    pinner->pinned.Invalidate();
    __atomic_store_n(&(pinner->tid.futex), 0, __ATOMIC_RELEASE);
  }

  // If all the senders are (or were made) good, there is no need to do the hard
  // case.
  if (valid_senders == num_senders) {
    return true;
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
        if (!need_recovery[i]) {
          return false;
        }
        ++num_missing;
        continue;
      }
      CHECK(!need_recovery[i]) << ": Somebody else recovered a sender: " << i;
      // We can do a relaxed load here because we're the only person touching
      // this sender at this point, if it matters. If it's not a dead sender,
      // then any message it ever has will eventually be accounted for if we
      // make enough tries through the outer loop.
      const Index scratch_index = sender->scratch_index.RelaxedLoad();
      if (!accounted_for[scratch_index.message_index()]) {
        ++num_accounted_for;
      }
      accounted_for[scratch_index.message_index()] = true;
    }

    for (size_t i = 0; i < queue_size; ++i) {
      // Same logic as above for scratch_index applies here too.
      const Index index = memory->GetQueue(i)->RelaxedLoad();
      if (!accounted_for[index.message_index()]) {
        ++num_accounted_for;
      }
      accounted_for[index.message_index()] = true;
    }

    for (size_t pinner_index = 0; pinner_index < num_pinners; ++pinner_index) {
      // Same logic as above for scratch_index applies here too.
      const Index index =
          memory->GetPinner(pinner_index)->scratch_index.RelaxedLoad();
      if (!accounted_for[index.message_index()]) {
        ++num_accounted_for;
      }
      accounted_for[index.message_index()] = true;
    }

    CHECK_LE(num_accounted_for + num_missing, num_messages);
  }

  while (num_missing != 0) {
    const size_t starting_num_missing = num_missing;
    for (size_t i = 0; i < num_senders; ++i) {
      Sender *sender = memory->GetSender(i);
      const uint32_t tid =
          __atomic_load_n(&(sender->tid.futex), __ATOMIC_ACQUIRE);
      if (!(tid & FUTEX_OWNER_DIED)) {
        CHECK(!need_recovery[i]) << ": Somebody else recovered a sender: " << i;
        continue;
      }
      if (!need_recovery[i]) {
        return false;
      }
      // We can do relaxed loads here because we're the only person touching
      // this sender at this point.
      const Index scratch_index = sender->scratch_index.RelaxedLoad();
      const Index to_replace = sender->to_replace.RelaxedLoad();

      // Candidate.
      if (to_replace.valid()) {
        CHECK_LE(to_replace.message_index(), accounted_for.size());
      }
      if (scratch_index.valid()) {
        CHECK_LE(scratch_index.message_index(), accounted_for.size());
      }
      if (!to_replace.valid() || accounted_for[to_replace.message_index()]) {
        CHECK(scratch_index.valid());
        VLOG(3) << "Sender " << i
                << " died, to_replace is already accounted for";
        // If both are accounted for, we are corrupt...
        CHECK(!accounted_for[scratch_index.message_index()]);

        // to_replace is already accounted for.  This means that we didn't
        // atomically insert scratch_index into the queue yet.  So
        // invalidate to_replace.
        sender->to_replace.Invalidate();
        // Sender definitely will not have gotten here, so finish for it.
        memory->GetMessage(scratch_index)
            ->header.queue_index.RelaxedInvalidate();

        // And then mark this sender clean.
        __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);
        need_recovery[i] = false;

        // And account for scratch_index.
        accounted_for[scratch_index.message_index()] = true;
        --num_missing;
        ++num_accounted_for;
      } else if (!scratch_index.valid() ||
                 accounted_for[scratch_index.message_index()]) {
        VLOG(3) << "Sender " << i
                << " died, scratch_index is already accounted for";
        // scratch_index is accounted for.  That means we did the insert,
        // but didn't record it.
        CHECK(to_replace.valid());

        // Make sure to indicate it's an unused message before a sender gets its
        // hands on it.
        memory->GetMessage(to_replace)->header.queue_index.RelaxedInvalidate();
        aos_compiler_memory_barrier();

        // Finish the transaction.  Copy to_replace, then clear it.

        sender->scratch_index.Store(to_replace);
        sender->to_replace.Invalidate();

        // And then mark this sender clean.
        __atomic_store_n(&(sender->tid.futex), 0, __ATOMIC_RELEASE);
        need_recovery[i] = false;

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
    // CHECK that we are making progress.
    CHECK_NE(num_missing, starting_num_missing);
  }
  return true;
}

void Cleanup(LocklessQueueMemory *memory, const GrabQueueSetupLockOrDie &lock) {
  // The number of iterations is bounded here because there are only a finite
  // number of senders in existence which could die, and no new ones can be
  // created while we're in here holding the lock.
  while (!DoCleanup(memory, lock)) {
  }
}

// Exposes rt_tgsigqueueinfo so we can send the signal *just* to the target
// thread.
// TODO(Brian): Do directly in assembly for armhf at least for efficiency.
int rt_tgsigqueueinfo(pid_t tgid, pid_t tid, int sig, siginfo_t *si) {
  return syscall(SYS_rt_tgsigqueueinfo, tgid, tid, sig, si);
}

QueueIndex ZeroOrValid(QueueIndex index) {
  if (!index.valid()) {
    return index.Clear();
  }
  return index;
}

}  // namespace

size_t LocklessQueueConfiguration::message_size() const {
  // Round up the message size so following data is aligned appropriately.
  // Make sure to leave space to align the message data. It will be aligned
  // relative to the start of the shared memory region, but that might not be
  // aligned for some use cases.
  return LocklessQueueMemory::AlignmentRoundUp(message_data_size +
                                               kChannelDataRedzone * 2 +
                                               (kChannelDataAlignment - 1)) +
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

  CHECK_EQ(size % alignof(Pinner), 0u);
  size += LocklessQueueMemory::SizeOfPinners(config);

  return size;
}

// Calculates the starting byte for a redzone in shared memory. This starting
// value is simply incremented for subsequent bytes.
//
// The result is based on the offset of the region in shared memor, to ensure it
// is the same for each region when we generate and verify, but different for
// each region to help catch forms of corruption like copying out-of-bounds data
// from one place to another.
//
// memory is the base pointer to the shared memory. It is used to calculated
// offsets. starting_data is the start of the redzone's data. Each one will
// get a unique pattern.
uint8_t RedzoneStart(const LocklessQueueMemory *memory,
                     const char *starting_data) {
  const auto memory_int = reinterpret_cast<uintptr_t>(memory);
  const auto starting_int = reinterpret_cast<uintptr_t>(starting_data);
  DCHECK(starting_int >= memory_int);
  DCHECK(starting_int < memory_int + LocklessQueueMemorySize(memory->config));
  const uintptr_t starting_offset = starting_int - memory_int;
  // Just XOR the lower 2 bytes. They higher-order bytes are probably 0
  // anyways.
  return (starting_offset & 0xFF) ^ ((starting_offset >> 8) & 0xFF);
}

// Returns true if the given redzone has invalid data.
bool CheckRedzone(const LocklessQueueMemory *memory,
                  absl::Span<const char> redzone) {
  uint8_t redzone_value = RedzoneStart(memory, redzone.data());

  bool bad = false;

  for (size_t i = 0; i < redzone.size(); ++i) {
    if (memcmp(&redzone[i], &redzone_value, 1)) {
      bad = true;
    }
    ++redzone_value;
  }

  return bad;
}

// Returns true if either of message's redzones has invalid data.
bool CheckBothRedzones(const LocklessQueueMemory *memory,
                       const Message *message) {
  return CheckRedzone(memory,
                      message->PreRedzone(memory->message_data_size())) ||
         CheckRedzone(memory, message->PostRedzone(memory->message_data_size(),
                                                   memory->message_size()));
}

// Fills the given redzone with the expected data.
void FillRedzone(LocklessQueueMemory *memory, absl::Span<char> redzone) {
  uint8_t redzone_value = RedzoneStart(memory, redzone.data());
  for (size_t i = 0; i < redzone.size(); ++i) {
    memcpy(&redzone[i], &redzone_value, 1);
    ++redzone_value;
  }

  // Just double check that the implementations match.
  CHECK(!CheckRedzone(memory, redzone));
}

LocklessQueueMemory *InitializeLocklessQueueMemory(
    LocklessQueueMemory *memory, LocklessQueueConfiguration config) {
  // Everything should be zero initialized already.  So we just need to fill
  // everything out properly.

  // This is the UID we will use for checking signal-sending permission
  // compatibility.
  //
  // The manpage says:
  //   For a process to have permission to send a signal, it must either be
  //   privileged [...], or the real or effective user ID of the sending process
  //   must equal the real or saved set-user-ID of the target process.
  //
  // Processes typically initialize a queue in random order as they start up.
  // This means we need an algorithm for verifying all processes have
  // permissions to send each other signals which gives the same answer no
  // matter what order they attach in. We would also like to avoid maintaining a
  // shared list of the UIDs of all processes.
  //
  // To do this while still giving sufficient flexibility for all current use
  // cases, we track a single UID for the queue. All processes with a matching
  // euid+suid must have this UID. Any processes with distinct euid/suid must
  // instead have a matching ruid.  This guarantees signals can be sent between
  // all processes attached to the queue.
  //
  // In particular, this allows a process to change only its euid (to interact
  // with a queue) while still maintaining privileges via its ruid. However, it
  // can only use privileges in ways that do not require changing the euid back,
  // because while the euid is different it will not be able to receive signals.
  // We can't actually verify that, but we can sanity check that things are
  // valid when the queue is initialized.

  uid_t uid;
  {
    uid_t ruid, euid, suid;
    PCHECK(getresuid(&ruid, &euid, &suid) == 0);
    // If these are equal, then use them, even if that's different from the real
    // UID. This allows processes to keep a real UID of 0 (to have permissions
    // to perform system-level changes) while still being able to communicate
    // with processes running unprivileged as a distinct user.
    if (euid == suid) {
      uid = euid;
      VLOG(1) << "Using euid==suid " << uid;
    } else {
      uid = ruid;
      VLOG(1) << "Using ruid " << ruid;
    }
  }

  // Grab the mutex.  We don't care if the previous reader died.  We are going
  // to check everything anyways.
  GrabQueueSetupLockOrDie grab_queue_setup_lock(memory);

  if (!memory->initialized) {
    // TODO(austin): Check these for out of bounds.
    memory->config.num_watchers = config.num_watchers;
    memory->config.num_senders = config.num_senders;
    memory->config.num_pinners = config.num_pinners;
    memory->config.queue_size = config.queue_size;
    memory->config.message_data_size = config.message_data_size;

    const size_t num_messages = memory->num_messages();
    // There need to be at most MaxMessages() messages allocated.
    CHECK_LE(num_messages, Index::MaxMessages());

    for (size_t i = 0; i < num_messages; ++i) {
      Message *const message =
          memory->GetMessage(Index(QueueIndex::Zero(memory->queue_size()), i));
      message->header.queue_index.Invalidate();
      FillRedzone(memory, message->PreRedzone(memory->message_data_size()));
      FillRedzone(memory, message->PostRedzone(memory->message_data_size(),
                                               memory->message_size()));
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
    memory->uid = uid;

    for (size_t i = 0; i < memory->num_senders(); ++i) {
      ::aos::ipc_lib::Sender *s = memory->GetSender(i);
      // Nobody else can possibly be touching these because we haven't set
      // initialized to true yet.
      s->scratch_index.RelaxedStore(Index(0xffff, i + memory->queue_size()));
      s->to_replace.RelaxedInvalidate();
    }

    for (size_t i = 0; i < memory->num_pinners(); ++i) {
      ::aos::ipc_lib::Pinner *pinner = memory->GetPinner(i);
      // Nobody else can possibly be touching these because we haven't set
      // initialized to true yet.
      pinner->scratch_index.RelaxedStore(
          Index(0xffff, i + memory->num_senders() + memory->queue_size()));
      pinner->pinned.Invalidate();
    }

    aos_compiler_memory_barrier();
    // Signal everything is done.  This needs to be done last, so if we die, we
    // redo initialization.
    memory->initialized = true;
  } else {
    CHECK_EQ(uid, memory->uid) << ": UIDs must match for all processes";
  }

  return memory;
}

void LocklessQueue::Initialize() {
  InitializeLocklessQueueMemory(memory_, config_);
}

LocklessQueueWatcher::~LocklessQueueWatcher() {
  if (watcher_index_ == -1) {
    return;
  }

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

  // Cleanup is cheap. The next user will do it anyways, so no need for us to do
  // anything right now.

  // And confirm that nothing is owned by us.
  const int num_watchers = memory_->num_watchers();
  for (int i = 0; i < num_watchers; ++i) {
    CHECK(!death_notification_is_held(&(memory_->GetWatcher(i)->tid)))
        << ": " << i;
  }
}

std::optional<LocklessQueueWatcher> LocklessQueueWatcher::Make(
    LocklessQueue queue, int priority) {
  queue.Initialize();
  LocklessQueueWatcher result(queue.memory(), priority);
  if (result.watcher_index_ != -1) {
    return std::move(result);
  } else {
    return std::nullopt;
  }
}

LocklessQueueWatcher::LocklessQueueWatcher(LocklessQueueMemory *memory,
                                           int priority)
    : memory_(memory) {
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
    return;
  }

  Watcher *const w = memory_->GetWatcher(watcher_index_);

  w->pid = getpid();
  w->priority = priority;

  // Grabbing a mutex is a compiler and memory barrier, so nothing before will
  // get rearranged afterwords.
  death_notification_init(&(w->tid));
}

LocklessQueueWakeUpper::LocklessQueueWakeUpper(LocklessQueue queue)
    : memory_(queue.const_memory()), pid_(getpid()), uid_(getuid()) {
  queue.Initialize();
  watcher_copy_.resize(memory_->num_watchers());
}

int LocklessQueueWakeUpper::Wakeup(const int current_priority) {
  const size_t num_watchers = memory_->num_watchers();

  CHECK_EQ(watcher_copy_.size(), num_watchers);

  // Grab a copy so it won't change out from underneath us, and we can sort it
  // nicely in C++.
  // Do note that there is still a window where the process can die *after* we
  // read everything.  We will still PI boost and send a signal to the thread in
  // question.  There is no way without pidfd's to close this window, and
  // creating a pidfd is likely not RT.
  for (size_t i = 0; i < num_watchers; ++i) {
    const Watcher *w = memory_->GetWatcher(i);
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

LocklessQueueSender::LocklessQueueSender(LocklessQueueMemory *memory)
    : memory_(memory) {
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
    VLOG(1) << "Too many senders, starting to bail.";
    return;
  }

  ::aos::ipc_lib::Sender *const sender = memory_->GetSender(sender_index_);

  // Indicate that we are now alive by taking over the slot. If the previous
  // owner died, we still want to do this.
  death_notification_init(&(sender->tid));

  const Index scratch_index = sender->scratch_index.RelaxedLoad();
  Message *const message = memory_->GetMessage(scratch_index);
  CHECK(!message->header.queue_index.RelaxedLoad(memory_->queue_size()).valid())
      << ": " << std::hex << scratch_index.get();
}

LocklessQueueSender::~LocklessQueueSender() {
  if (sender_index_ != -1) {
    CHECK(memory_ != nullptr);
    death_notification_release(&(memory_->GetSender(sender_index_)->tid));
  }
}

std::optional<LocklessQueueSender> LocklessQueueSender::Make(
    LocklessQueue queue) {
  queue.Initialize();
  LocklessQueueSender result(queue.memory());
  if (result.sender_index_ != -1) {
    return std::move(result);
  } else {
    return std::nullopt;
  }
}

size_t LocklessQueueSender::size() const {
  return memory_->message_data_size();
}

void *LocklessQueueSender::Data() {
  ::aos::ipc_lib::Sender *sender = memory_->GetSender(sender_index_);
  const Index scratch_index = sender->scratch_index.RelaxedLoad();
  Message *const message = memory_->GetMessage(scratch_index);
  // We should have invalidated this when we first got the buffer. Verify that
  // in debug mode.
  DCHECK(
      !message->header.queue_index.RelaxedLoad(memory_->queue_size()).valid())
      << ": " << std::hex << scratch_index.get();

  return message->data(memory_->message_data_size());
}

void LocklessQueueSender::Send(
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

void LocklessQueueSender::Send(
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
  CHECK(!CheckBothRedzones(memory_, message))
      << ": Somebody wrote outside the buffer of their message";

  // We should have invalidated this when we first got the buffer. Verify that
  // in debug mode.
  DCHECK(
      !message->header.queue_index.RelaxedLoad(memory_->queue_size()).valid())
      << ": " << std::hex << scratch_index.get();

  message->header.length = length;
  // Pass these through.  Any alternative behavior can be implemented out a
  // layer.
  message->header.remote_queue_index = remote_queue_index;
  message->header.monotonic_remote_time = monotonic_remote_time;
  message->header.realtime_remote_time = realtime_remote_time;

  Index to_replace = Index::Invalid();
  while (true) {
    const QueueIndex actual_next_queue_index =
        memory_->next_queue_index.Load(queue_size);
    const QueueIndex next_queue_index = ZeroOrValid(actual_next_queue_index);

    const QueueIndex incremented_queue_index = next_queue_index.Increment();

    // This needs to synchronize with whoever the previous writer at this
    // location was.
    to_replace = memory_->LoadIndex(next_queue_index);

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
    //
    // This is just a best-effort check to skip reading the clocks if possible.
    // If this fails, then the compare-exchange below definitely would, so we
    // can bail out now.
    {
      const QueueIndex previous_index =
          memory_->GetMessage(to_replace)
              ->header.queue_index.RelaxedLoad(queue_size);
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

  DCHECK(!CheckBothRedzones(memory_, memory_->GetMessage(to_replace)))
      << ": Invalid message found in shared memory";
  // to_replace is our current scratch_index. It isn't in the queue, which means
  // nobody new can pin it. They can set their `pinned` to it, but they will
  // back it out, so they don't count. This means that we just need to find a
  // message for which no pinner had it in `pinned`, and then we know this
  // message will never be pinned. We'll start with to_replace, and if that is
  // pinned then we'll look for a new one to use instead.
  const Index new_scratch =
      SwapPinnedSenderScratch(memory_, sender, to_replace);
  DCHECK(!CheckBothRedzones(
      memory_, memory_->GetMessage(sender->scratch_index.RelaxedLoad())))
      << ": Invalid message found in shared memory";

  // If anybody is looking at this message (they shouldn't be), then try telling
  // them about it (best-effort).
  memory_->GetMessage(new_scratch)->header.queue_index.RelaxedInvalidate();
}

int LocklessQueueSender::buffer_index() const {
  ::aos::ipc_lib::Sender *const sender = memory_->GetSender(sender_index_);
  // We can do a relaxed load on our sender because we're the only person
  // modifying it right now.
  const Index scratch_index = sender->scratch_index.RelaxedLoad();
  return scratch_index.message_index();
}

LocklessQueuePinner::LocklessQueuePinner(
    LocklessQueueMemory *memory, const LocklessQueueMemory *const_memory)
    : memory_(memory), const_memory_(const_memory) {
  GrabQueueSetupLockOrDie grab_queue_setup_lock(memory_);

  // Since we already have the lock, go ahead and try cleaning up.
  Cleanup(memory_, grab_queue_setup_lock);

  const int num_pinners = memory_->num_pinners();

  for (int i = 0; i < num_pinners; ++i) {
    ::aos::ipc_lib::Pinner *p = memory->GetPinner(i);
    // This doesn't need synchronization because we're the only process doing
    // initialization right now, and nobody else will be touching pinners which
    // we're interested in.
    const uint32_t tid = __atomic_load_n(&(p->tid.futex), __ATOMIC_RELAXED);
    if (tid == 0) {
      pinner_index_ = i;
      break;
    }
  }

  if (pinner_index_ == -1) {
    VLOG(1) << "Too many pinners, starting to bail.";
    return;
  }

  ::aos::ipc_lib::Pinner *p = memory_->GetPinner(pinner_index_);
  p->pinned.Invalidate();

  // Indicate that we are now alive by taking over the slot. If the previous
  // owner died, we still want to do this.
  death_notification_init(&(p->tid));
}

LocklessQueuePinner::~LocklessQueuePinner() {
  if (pinner_index_ != -1) {
    CHECK(memory_ != nullptr);
    memory_->GetPinner(pinner_index_)->pinned.Invalidate();
    aos_compiler_memory_barrier();
    death_notification_release(&(memory_->GetPinner(pinner_index_)->tid));
  }
}

std::optional<LocklessQueuePinner> LocklessQueuePinner::Make(
    LocklessQueue queue) {
  queue.Initialize();
  LocklessQueuePinner result(queue.memory(), queue.const_memory());
  if (result.pinner_index_ != -1) {
    return std::move(result);
  } else {
    return std::nullopt;
  }
}

// This method doesn't mess with any scratch_index, so it doesn't have to worry
// about message ownership.
int LocklessQueuePinner::PinIndex(uint32_t uint32_queue_index) {
  const size_t queue_size = memory_->queue_size();
  const QueueIndex queue_index =
      QueueIndex::Zero(queue_size).IncrementBy(uint32_queue_index);
  ipc_lib::Pinner *const pinner = memory_->GetPinner(pinner_index_);

  AtomicIndex *const queue_slot = memory_->GetQueue(queue_index.Wrapped());

  // Indicate that we want to pin this message.
  pinner->pinned.Store(queue_index);
  aos_compiler_memory_barrier();

  {
    const Index message_index = queue_slot->Load();
    Message *const message = memory_->GetMessage(message_index);
    DCHECK(!CheckBothRedzones(memory_, message))
        << ": Invalid message found in shared memory";

    const QueueIndex message_queue_index =
        message->header.queue_index.Load(queue_size);
    if (message_queue_index == queue_index) {
      VLOG(3) << "Eq: " << std::hex << message_queue_index.index();
      aos_compiler_memory_barrier();
      return message_index.message_index();
    }
    VLOG(3) << "Message reused: " << std::hex << message_queue_index.index()
            << ", " << queue_index.index();
  }

  // Being down here means we asked to pin a message before realizing it's no
  // longer in the queue, so back that out now.
  pinner->pinned.Invalidate();
  VLOG(3) << "Unpinned: " << std::hex << queue_index.index();
  return -1;
}

size_t LocklessQueuePinner::size() const {
  return const_memory_->message_data_size();
}

const void *LocklessQueuePinner::Data() const {
  const size_t queue_size = const_memory_->queue_size();
  const ::aos::ipc_lib::Pinner *const pinner =
      const_memory_->GetPinner(pinner_index_);
  QueueIndex pinned = pinner->pinned.RelaxedLoad(queue_size);
  CHECK(pinned.valid());
  const Message *message = const_memory_->GetMessage(pinned);

  return message->data(const_memory_->message_data_size());
}

LocklessQueueReader::Result LocklessQueueReader::Read(
    uint32_t uint32_queue_index,
    ::aos::monotonic_clock::time_point *monotonic_sent_time,
    ::aos::realtime_clock::time_point *realtime_sent_time,
    ::aos::monotonic_clock::time_point *monotonic_remote_time,
    ::aos::realtime_clock::time_point *realtime_remote_time,
    uint32_t *remote_queue_index, size_t *length, char *data) const {
  const size_t queue_size = memory_->queue_size();

  // Build up the QueueIndex.
  const QueueIndex queue_index =
      QueueIndex::Zero(queue_size).IncrementBy(uint32_queue_index);

  // Read the message stored at the requested location.
  Index mi = memory_->LoadIndex(queue_index);
  const Message *m = memory_->GetMessage(mi);

  while (true) {
    DCHECK(!CheckBothRedzones(memory_, m))
        << ": Invalid message found in shared memory";
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
        return Result::NOTHING_NEW;
      }

      // Someone has re-used this message between when we pulled it out of the
      // queue and when we grabbed its index.  It is pretty hard to deduce
      // what happened. Just try again.
      const Message *const new_m = memory_->GetMessage(queue_index);
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
        return Result::TOO_OLD;
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
        return Result::NOTHING_NEW;
      } else {
        VLOG(3) << "Not near zero, " << std::hex << uint32_queue_index;
        return Result::TOO_OLD;
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
  if (data) {
    memcpy(data, m->data(memory_->message_data_size()),
           memory_->message_data_size());
  }
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
    return Result::OVERWROTE;
  }

  return Result::GOOD;
}

QueueIndex LocklessQueueReader::LatestIndex() const {
  const size_t queue_size = memory_->queue_size();

  // There is only one interesting case.  We need to know if the queue is empty.
  // That is done with a sentinel value.  At worst, this will be off by one.
  const QueueIndex next_queue_index =
      memory_->next_queue_index.Load(queue_size);
  if (next_queue_index.valid()) {
    const QueueIndex current_queue_index = next_queue_index.DecrementBy(1u);
    return current_queue_index;
  }
  return QueueIndex::Invalid();
}

size_t LocklessQueueSize(const LocklessQueueMemory *memory) {
  return memory->queue_size();
}

size_t LocklessQueueMessageDataSize(const LocklessQueueMemory *memory) {
  return memory->message_data_size();
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
  ::std::cout << "    size_t num_pinners = " << memory->config.num_pinners
              << ::std::endl;
  ::std::cout << "    size_t queue_size = " << memory->config.queue_size
              << ::std::endl;
  ::std::cout << "    size_t message_data_size = "
              << memory->config.message_data_size << ::std::endl;

  ::std::cout << "    AtomicQueueIndex next_queue_index = "
              << memory->next_queue_index.Load(queue_size).DebugString()
              << ::std::endl;

  ::std::cout << "    uid_t uid = " << memory->uid << ::std::endl;

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
    ::std::cout << "    [" << i << "] -> Message 0x" << std::hex
                << (reinterpret_cast<uintptr_t>(
                        memory->GetMessage(Index(i, i))) -
                    reinterpret_cast<uintptr_t>(memory))
                << std::dec << " {" << ::std::endl;
    ::std::cout << "      Header {" << ::std::endl;
    ::std::cout << "        AtomicQueueIndex queue_index = "
                << m->header.queue_index.Load(queue_size).DebugString()
                << ::std::endl;
    ::std::cout << "        monotonic_clock::time_point monotonic_sent_time = "
                << m->header.monotonic_sent_time << " 0x" << std::hex
                << m->header.monotonic_sent_time.time_since_epoch().count()
                << std::dec << ::std::endl;
    ::std::cout << "        realtime_clock::time_point realtime_sent_time = "
                << m->header.realtime_sent_time << " 0x" << std::hex
                << m->header.realtime_sent_time.time_since_epoch().count()
                << std::dec << ::std::endl;
    ::std::cout
        << "        monotonic_clock::time_point monotonic_remote_time = "
        << m->header.monotonic_remote_time << " 0x" << std::hex
        << m->header.monotonic_remote_time.time_since_epoch().count()
        << std::dec << ::std::endl;
    ::std::cout << "        realtime_clock::time_point realtime_remote_time = "
                << m->header.realtime_remote_time << " 0x" << std::hex
                << m->header.realtime_remote_time.time_since_epoch().count()
                << std::dec << ::std::endl;
    ::std::cout << "        size_t length = " << m->header.length
                << ::std::endl;
    ::std::cout << "      }" << ::std::endl;
    if (!CheckBothRedzones(memory, m)) {
      ::std::cout << "      // *** DATA REDZONES ARE CORRUPTED ***"
                  << ::std::endl;
    }
    ::std::cout << "      data: {";

    if (FLAGS_dump_lockless_queue_data) {
      const char *const m_data = m->data(memory->message_data_size());
      for (size_t j = 0; j < m->header.length; ++j) {
        char data = m_data[j];
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

  ::std::cout << "  Pinner pinners[" << memory->num_pinners() << "] {"
              << ::std::endl;
  for (size_t i = 0; i < memory->num_pinners(); ++i) {
    Pinner *p = memory->GetPinner(i);
    ::std::cout << "    [" << i << "] -> Pinner {" << ::std::endl;
    ::std::cout << "      aos_mutex tid = " << PrintMutex(&p->tid)
                << ::std::endl;
    ::std::cout << "      AtomicIndex scratch_index = "
                << p->scratch_index.Load().DebugString() << ::std::endl;
    ::std::cout << "      AtomicIndex pinned = "
                << p->pinned.Load(memory->queue_size()).DebugString()
                << ::std::endl;
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
