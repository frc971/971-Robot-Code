# User Guide/Cookbook

This section attempts to provide a comprehensive overview of all the
functionality available in the core of AOS. While this document should generally
be accurate, it is possible that it may miss _new_ features that get added over
time.

Note that this document is not intended to replace the comments documenting each
API. See the corresponding headers for details on the meanings of arguments and
the available options.

## Terminology

*Application*: An individual process running within AOS. Analogous to
a ROS node.

*Channel*: A stream of messages that can be published to or subscribed
to in an AOS system. Analogous to a ROS topic.

*Configuration*: The `aos.Configuration` flatbuffer message defining the
channels, applications, and nodes present in an AOS system.

*FlatBuffers*: The serialization format used for all AOS message. See
[FlatBuffers](/flatbuffers).

*Node*: A individual device on the network in an AOS system. E.g., if you
had 4 Raspberry Pis connected via Ethernet all talking to one another via
AOS then each Pi would be a node.

## EventLoop Interface

This section covers the functionality available via the interfaces defined in
`//aos/events/event_loop.h`. We will first provide a high-level overview of the
various classes/functionality, and then provide examples for various use-cases.

### High-Level Overview

The `EventLoop` class is the main interface by which applications interact with
the rest of the system. Everything presented by the `EventLoop` interface can be
simulated, run in real life, or replayed back from a log, and should support
expansions to future implementations. For most applications their only
interactions with the rest of the world should be through the `EventLoop`
interface. There will always be some number of applications that serve as
abstraction layers to access hardware, communicate with non-AOS applications,
etc. But the presence of non-AOS external interactions make testing and
simulation much harder and so in general the applications that interact with
hardware should be kept as thin wrappers that publish/receive raw data/outputs,
allowing the main application logic to only interact with the `EventLoop`.

### Clock Access & Time

The `monotonic_now()` and `realtime_now()` methods provide the current time (in
simulation, these will be the simulated time). `monotonic_now()` will be
monotonic (never decreasing) and generally free of any jumps (on a Linux system,
it will typically correspond to `CLOCK_MONOTONIC` from `clock_gettime(3)`).
`monotonic_now()` is what should generally be used by anything that cares about
accurate time intervals, elapsed time, etc. The monotonic time will generally be
a time since boot. `realtime_clock` will be a wall-clock time that may be
corrected by NTP (and so may jump forwards/backwards), and will provide a time
since the Unix epoch. This is generally only useful for providing debugging
information to humans, and is less useful for application logic due to potential
jumps from NTP corrections. If you are looking for a clock that is shared among
nodes on the system, then use the monotonic clock combined with the message
bridge statistics to get the offsets to each other node (see [Correlating
Monotonic Clocks Across Nodes](#correlating-monotonic-clocks-across-nodes);
do not rely on the realtime clocks being identical across the nodes on your
system.

While there is also a global `aos::monotonic_clock::now()` call (and
corresponding `aos::realtime_clock::now()`) that can be used, this will always
correspond to the actual system your code is running on and will not be mocked
out in simulation. As such, you should generally only access time through the
`EventLoop` interface; although if you are, e.g., trying to do ad-hoc
performance benchmarks in simulation, you may want to access the actual system
clock.

There are also situations where you may want to use the time at which an event
occurred (rather than the "current" time). For these situations, you will need
to access the `EventLoop` [Context](#event-loop-context).

Note that it is strongly assumed that all the clocks on the same node are
identical. I.e., calling `event_loop_->monotonic_now()` at the same time
in two separate applications on the same node should result in ~the same value
being output.

#### Examples

Using the monotonic clock to schedule a timer for 1 second from now:
```cpp
  timer_->Setup(event_loop_->monotonic_now() + std::chrono::seconds(1));
```

Using the realtime clock to timestamp a debug file:
```cpp
  std::stringstream filename;
  filename << "debug_" << event_loop_->realtime_now();
  aos::util::WriteStringToFileOrDie(filename.str(), "Hello, World!");
```

Using `aos::monotonic_clock::now()` to do some ad-hoc performance benchmarking
in simulation:

```cpp
void Ping::InterestingFunction() {
  const aos::monotonic_clock::time_point event_loop_start =
      event_loop_->monotonic_now();
  const aos::monotonic_clock::time_point system_start =
      aos::monotonic_clock::now();
  for (int ii = 0; ii < 1000000; ++ii) {
  }
  const aos::monotonic_clock::time_point event_loop_end =
      event_loop_->monotonic_now();
  const aos::monotonic_clock::time_point system_end =
      aos::monotonic_clock::now();
  LOG(INFO) << "EventLoop elapsed (will be zero in simulation): "
            << aos::time::DurationInSeconds(event_loop_end -
                                            event_loop_start);
  LOG(INFO) << "System elapsed: "
            << aos::time::DurationInSeconds(system_end - system_start);
}
```

### Event Loop Context

The `EventLoop` has a `context()` method which provides the current "Context".
The context will only be valid while the `EventLoop` is executing (i.e., it will
not be valid before or after execution, or during breaks in execution if
starting/stopping simulations). The context can be used to access information
about the current event being processed, and is defined at the top of the
`event_loop.h` header:

```cpp
// Struct available on Watchers, Fetchers, Timers, and PhasedLoops with context
// about the current message.
struct Context {
  // Time that the message was sent on this node, or the timer was triggered.
  monotonic_clock::time_point monotonic_event_time;
  // Realtime the message was sent on this node.  This is set to min_time for
  // Timers and PhasedLoops.
  realtime_clock::time_point realtime_event_time;

  // The rest are only valid for Watchers and Fetchers.

  // For a single-node configuration, these two are identical to *_event_time.
  // In a multinode configuration, these are the times that the message was
  // sent on the original node.
  monotonic_clock::time_point monotonic_remote_time;
  realtime_clock::time_point realtime_remote_time;

  // Index in the queue.
  uint32_t queue_index;
  // Index into the remote queue.  Useful to determine if data was lost.  In a
  // single-node configuration, this will match queue_index.
  uint32_t remote_queue_index;

  // Size of the data sent.
  size_t size;
  // Pointer to the data.
  const void *data;

  // buffer_index and source_boot_uuid members omitted.

  // Efficiently copies the flatbuffer into a FlatbufferVector, allocating
  // memory in the process.  It is vital that T matches the type of the
  // underlying flatbuffer.
  template <typename T>
  FlatbufferVector<T> CopyFlatBuffer() const;
};
```

Probably the most common use-case for the context is to access the current
event time. For instance, if you have a timer registered, you may wish to access
the time at which the timer was expected to happen rather than the time at which
it actually happened; similarly, with watchers you may wish to access the time
at which the messages were sent. Doing this has a few advantages over calling
`EventLoop::monotonic_now()`:

1. It gives you a more reproducible time-stamp (e.g., Watcher and Phased Loop
   timestamps are fully reproducible in log replay; normal timers vary).
2. It lets you schedule events independently of the latency of your handler
   itself (e.g., the code that schedules periodic timers uses the scheduled
   time rather than actual callback time so that periodic timers actually get
   called at `N`hz rather than at `N - the_impact_of_wakeup_latency`hz).
3. By reducing your queries to the clock, you can marginally improve
   performance. This is rarely relevant.

However, there are situations where you will prefer using
`event_loop_->monotonic_now()` because you do need the actual current time.


#### Examples

Using the context `monotonic_event_time` to schedule a timer at a set amount of
time after a message was sent, even if the watcher handler was called at an
unpredictable time:

```cpp
aos::TimerHandler *delay_timer = event_loop_->AddTimer([]() {
  LOG(INFO) << "It's been 1 second since the last Pong message was sent!";
});

event_loop_->MakeWatcher(
    "/test", [this, delay_timer](const examples::Pong &pong) {
      HandlePong(pong);
      // Will always get scheduled for the same time, regardless of how long
      // HandlePong() takes.
      delay_timer->Setup(event_loop_->context().monotonic_event_time +
                         std::chrono::seconds(1));
    });
```


### Timers

Timers allow events to be scheduled at some scheduled time. When you create a
Timer via the `AddTimer` method on an `EventLoop`, you specify the function that
will get called when the timer is triggered. In order to actually schedule an
event on the timer you must call `Setup`. Calling `Setup` clears any existing
state of the timer. You can either schedule a timer to be called once at a
particular time in the future or schedule it to be called at a regular interval.

When scheduled for a periodic interval, the timer will skip "extra" cycles if
the next trigger would occur in the past. I.e., if your next trigger is
scheduled for t=1sec on a 1 Hz timer, but your process gets delayed and
doesn't actually trigger until t=2.5sec, then rather than attempting
to "catch up" and call an extra callback to account for the missed t=2sec
trigger, the timer will just schedule the next wakeup for t=3sec.

Note that (currently) we do not log enough information to fully reproduce the
exact times returned by calls to `event_loop_->monotonic_now()`; nor do we log
the exact times at which Timers were called at runtime. As such, if you want
full reproducibility in your log replay you will need to take care in how you
schedule your timers (see [Log Replay Reproducibility](#log-replay-reproducibility)).

#### Examples

Scheduling a timer to get called one second from now:
```cpp
...
  // In the constructor of your class:
  timer_handle_ = event_loop_->AddTimer([this]() { SomeMethod(); });
...

  // Later, when you decide you need to schedule the timer for one second from
  // now:
  timer_handle_->Setup(event_loop_->monotonic_now() + std::chrono::seconds(1));
```

Scheduling a timer to get called one second after startup, and at 100Hz
thereafter.
```cpp
...
  // In the constructor of your class:
  timer_handle_ = event_loop_->AddTimer([this]() { SomeMethod(); });
  event_loop_->OnRun([this]() {
    timer_handle_->Setup(
        event_loop_->monotonic_now() + std::chrono::seconds(1),
        std::chrono::milliseconds(10));
  });
...
```

Setting up a timer to call a method at 1 Hz until some condition is met:

```cpp
...
  // In the constructor of your class:
  timer_handle_ = event_loop_->AddTimer([this]() { SomeMethod(); });
  event_loop_->OnRun([this]() {
    timer_handle_->Setup(event_loop_->monotonic_now(), std::chrono::seconds(1));
  });
...

  // in SomeMethod():
  if (done_working) {
    timer_handle_->Disable();
  }
...

```

Note that setting a 1 Hz repeated timer is *not* equivalent to always
rescheduling a timer for X seconds from the current callback. Consider
the following example, which takes advantage of the `monotonic_event_time`
from the [Event Loop Context](#event-loop-context):

```cpp
...
  timer_handle_ = event_loop_->AddTimer([this]() { SendPing(); });
  // Accessing event_loop_->context() outside of an EventLoop handler is
  // unsupported.
  //timer_handle_->Setup(event_loop_->context().monotonic_event_time);
  event_loop_->OnRun([this]() {
      // The context *is* defined during the OnRun() handlers.
      // So far, this code is fine.
      timer_handle_->Setup(event_loop_->context().monotonic_event_time);

      // Do some really long initialization (ideally, this would be done
      // *before* the OnRun(), but maybe it relies on the EventLoop being
      // running).
      std::this_thread::sleep_for(std::chrono::seconds(100));
  });
...

void Ping::SendPing() {
  LOG(INFO) << "Ping!";
  // This is (typically) bad!
  timer_handle_->Setup(event_loop_->context().monotonic_event_time + std::chrono::seconds(1));
}
```
When the above code is run, if t=0sec during the `OnRun`, then the first handler
will get scheduled for t=1sec; that will schedule another for t=2sec, then
t=3sec, and so on. However, because the initialization in the `OnRun` took
so long, it will be t=100sec before the first handler is called. As such,
on the first call to this handler we will have:
```
event_loop_->monotonic_now(): 100sec
event_loop_->context().monotonic_event_time(): 0sec
```
And so when we schedule the next timer callback for t=1sec, it will get
called immediately. As will the subsequent handler for t=2sec, t=3sec,
and all the way up to the hundredth handler. This is very rarely what
you want!

However, you also don't want to use
`event_loop_->monotonic_now()` instead. If you did this in the above example,
the first `SendPing()` call would be scheduled for
0sec and happen at, say, 100.001 sec. So it will schedule the next call for
101.001. But that won't actually happen until 102.003 sec. And so the next call
will get scheduled for 103.003 sec. And you end up calling your periodic
function at slightly less than your desired 1 Hz.

There are situations where writing timers using the above patterns may make
sense. But typically you should just use the second argument to the timer
`Setup` call demonstrated earlier. Doing this means that in the same
situation with a 100 second delay, your timer callback will get called
at t=0sec, t=101sec, t=102sec, and so on.

### Phased Loops

Phased Loops can be thought of as a sort of special case of timers.
Phased Loops are always periodic, but rather than setting a specific start time
for when the loop will be called, you instead specify a period and an offset.
The callback will then get called at `offset + period * k`sec on the monotonic
clock, for all `k`. Like the regular Timers, Phased Loops will skip iterations
if processing has fallen behind; unlike regular Timers, the Phased Loop will
pass a counter into the callback that lets the callback know how
many iterations have passed since the last call. In the future,
this this feature may also be provided for Timers.

Phased Loops are useful when you want your timer callbacks to happen
at the same intervals, regardless of what base time you used in
your original `Setup` call. In the typical use-case with a regular Timer
people will write something like
`timer_handle_->Setup(event_loop_->monotonic_now(), period)`. But because
the `monotonic_now()` call defines the effective offset, and because the
value of it is non-deterministic, the exact schedule of the resulting
periodic calls is less predictable than with a Phased Loop. This
translates into a few advantages for Phased Loops:

1. Your log replay will be more reproducible, since the timer callbacks
   will get called at the same times as they would've in the original
   system.
2. Runtime system load will be more deterministic; if you, e.g., have
   a bunch of 1 Hz timers, then depending on exactly how your system
   boots up you could get anything from all the Timers getting scheduled
   at exactly the same offset, or end up with them all distributed
   across the second.
3. Deliberately staging periodic tasks to happen with a particular timing
   across processes. You should be careful about leaning too heavily on
   this, as relying on carefully scheduled timing of events across
   different processes can create non-obvious interdependencies that
   future developers may have trouble understanding.

#### Examples

Scheduling a PhasedLoop to occur at 1 Hz (where you either don't care about the
offset, or want the implied default offset of 0):

```cpp
  event_loop_->AddPhasedLoop(
      [](int missed_cycles) {
        LOG(INFO) << "It has been " << missed_cycles
                  << " period(s) since the last callback.";
      },
      std::chrono::seconds(1));
```

Scheduling a PhasedLoop at 0.1 Hz that gets scheduled with an offset of 2 sec
(and so gets called at t=2sec,12sec,22sec,...), with a sanity check that it
is indeed getting called with the correct offset:
```cpp
  event_loop_->AddPhasedLoop(
      [this](int) {
        int scheduled_seconds =
            std::chrono::duration_cast<std::chrono::seconds>(
                event_loop_->context().monotonic_event_time.time_since_epoch()).count();
        int periods = scheduled_seconds / 10;
        CHECK_EQ(2, scheduled_seconds - periods * 10);
        LOG(INFO) << "Scheduled time: "
                  << event_loop_->context().monotonic_event_time
                  << " Current time: " << event_loop_->monotonic_now();
      },
      std::chrono::seconds(10), std::chrono::seconds(2));
```

### Senders

Senders allow you to send messages on AOS channels. You can only
create a `Sender` for a channel that is sendable on the current
node (see [Channels](#channels) for more details).

To create a `Sender` you need to have access to the C++ flatbuffer
type you will be using to send on the channel,[^rawsenders]
 and will need to know which channel name you want to
send on. The `TryMakeSender` and `MakeSender` methods then allow you to create a
sender while initializing your application.

The public interface to the `Sender` type itself has several methods, but the
most common use-case only ever invokes `MakeBuilder` on the `Sender`. The
`Sender::Builder` object exists to represent the lifecycle of constructing and
sending a single message. Once you have constructed a builder with
`MakeBuilder`, all further operations will happen through the `Builder`.
See the examples below for how this works.

When you construct a `Builder` this way, the `Builder` creates a
`FlatBufferBuilder` that builds your message directly into the shared memory for
the AOS channel and which will error if you attempt to build a FlatBuffer larger
than the maximum message size for the channel.

The main exception to this pattern occurs when you want to pre-construct a
message and then just make minor modifications each time before you send it.
This is rarely necessary, and tends to be error-prone due to some of the
subtleties in how mutating pre-built FlatBuffers works. However, we do use
it internally in the `EventLoop` class to create [Timing
Reports](#timing-reports). Note that the
performance benefit from this tends to be too small to be worth
worrying about, especially because you will still have to do a
`memcpy` of every message you send (since once a message is sent, you can no
longer write to that shared memory buffer until it recycles back through
the queue). However, we do include an example of doing this below.

The remaining methods on `Sender` are largely just convenient accessors:

`CheckOk()`: Provides convenient way to die on any error returned from
`Send*()`.

`*_sent_time()`, `sent_queue_index()`, `buffer_index()`: Allow you to retrieve
information about the last sent message.

`SendDetached()`: Specialized `Send()` for if you need to detach the flatbuffer
  yourself before sending it. Not really used anywhere.


There are two main limitations on what messages you can send on a channel:

1. Your message, when serialized, cannot exceed the `max_size` specified in
   the [Configuration](#configurations) for the [Channel](#channels).
   As mentioned above, this is enforced by the `Builder`.
2. You may not send messages above the configured `frequency` for the channel.
   Note that the `frequency` is a maximum for _all_ processes sending
   on the channel in question. See [Sent Too Fast](#sent-too-fast)
   for discussion on this. This is not a fatal error, and you can
   determine if your message was successfully sent by checking
   the return value of `Send()`.


Additionally, while the default is generally fine in most cases, the
[Configuration](#configurations) also includes a maximum number of senders that
can be registered on any channel (across all processes). See
[ShmEventLoop](#shmeventloop) for discussion of why this limitation exists.

#### Examples

The normal pattern for constructing and sending a simple message
(using the `Ping` sample message described in [Getting
Started](/getting_started#defining-flatbuffer-messages):

```cpp
class Ping {
 public:
  Ping(aos::EventLoop *event_loop)
      : sender_(event_loop->MakeSender<aos::examples::Ping>("/test")) {}
  void SendPing() {
    // Create the Sender::Builder that will handle creating a FlatBufferBuilder
    // pointed at the correct memory and which can actually send messages.
    aos::Sender<aos::examples::Ping>::Builder builder = sender_.MakeBuilder();
    // Construct the builder for the specific message we are building.
    aos::examples::Ping::Builder ping_builder =
        builder.MakeBuilder<aos::examples::Ping>();
    ping_builder.add_value(971);
    // Actually send out the completed message.
    builder.CheckOk(builder.Send(ping_builder.Finish()));
  }

 private:
  aos::Sender<aos::examples::Ping> sender_;
};
```

Using the `Builder` to construct a more complex message and send it:

.fbs file for the message we are sending:
```
namespace aos.examples;

table SubMessage {
  list:[int] (id: 0);
}

table Message {
  sub:SubMessage (id: 0);
}

root_type Message;
```

Sending the message (presuming the `sender_` has already been constructed):

```cpp
// We will construct a message that looks like:
// {"sub": { "list": [9, 7, 1] }}

// Providing a Populate*() is a common pattern to allow a separate class
// or method to handle construction of one of the sub-messages of a message.
// E.g., if you have a bunch of widgets each with a status and then an overall
// Status message with a list of WidgetStatus's, then you would probably have
// a PopulateWidgetStatus on your widget class with the same basic signature
// you see here.
flatbuffers::Offset<aos::examples::SubMessage> PopulateSubMessage(
    flatbuffers::FlatBufferBuilder *fbb) {
  // The FlatBufferBuilder requires that we start from the bottom-up,
  // so start by constructing the [9, 7, 1] vector.
  // Note that this example avoids using the CreateVector() overload that
  // takes an std::vector, as using an std::vector will result in dynamic
  // memory allocations, which generally have non-deterministic timing.
  std::array<int, 3> list = {9, 7, 1};
  flatbuffers::Offset<flatbuffers::Vector<int>> list_offset =
      fbb->CreateVector(list.data(), list.size());
  aos::examples::SubMessage::Builder sub_message_builder(*fbb);
  sub_message_builder.add_list(list_offset);
  return sub_message_builder.Finish();
}

void SendMessage() {
  aos::Sender<aos::examples::Message>::Builder builder =
      sender_.MakeBuilder();
  // Note that we need build and finish the SubMessage *before*
  // we create the Builder for the Message table. This is a consequence
  // of how the FlatBufferBuilder works.
  flatbuffers::Offset<aos::examples::SubMessage> sub_offset =
      PopulateSubMessage(builder.fbb());
  aos::examples::Message::Builder message_builder =
      builder.MakeBuilder<aos::examples::Message>();
  message_builder.add_sub(sub_offset);
  builder.CheckOk(builder.Send(message_builder.Finish()));
}
```

For more discussion on dynamic memory allocations and best practices in
realtime-code, see [Realtime Scheduler](#realtime-scheduler). To better
understand some of the constraints associated with the `FlatBufferBuilder`, see
[FlatBuffers](/flatbuffers).

Building a single FlatBuffer at startup and then mutating it before sending
instead of rebuilding the FlatBuffer every time (using the sample
`aos.examples.Ping` message):

```cpp
class Ping {
 public:
  Ping(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        sender_(event_loop_->MakeSender<aos::examples::Ping>("/test")),
        message_(MakePing()) {}

  void SendPing() {
    // Increment the value on each send and update the send time appropriately.
    message_.mutable_message()->mutate_value(message_.message().value() + 1);
    message_.mutable_message()->mutate_send_time(
        event_loop_->monotonic_now().time_since_epoch().count());
    // Actually send out the completed message.
    // Note that this will do a `memcpy` to get the message into shared memory,
    // since, unlike the normal procedure, this does not construct the message
    // in-place in shared memory.
    sender_.CheckOk(sender_.Send(message_));
  }

 private:
  // Constructs the Ping message that will get reused for sending every time.
  static aos::FlatbufferDetachedBuffer<aos::examples::Ping> MakePing() {
    // This code is just going to get called at startup, so the dynamic memory
    // allocation in a default-constructed FlatBufferBuilder is fine.
    flatbuffers::FlatBufferBuilder fbb;
    aos::examples::Ping::Builder ping_builder(fbb);
    // We need to ensure that all the fields that we will want to mutate
    // later are actually populated. Otherwise, the memory won't get
    // allocated for them and we won't be able to add them since
    // the flatbuffer will already be fully built and finished.
    ping_builder.add_value(0);
    ping_builder.add_send_time(0);
    fbb.Finish(ping_builder.Finish());
    return fbb.Release();
  }
  aos::EventLoop *event_loop_;
  aos::Sender<aos::examples::Ping> sender_;
  // Buffer that owns the message that we will be sending.
  aos::FlatbufferDetachedBuffer<aos::examples::Ping> message_;
};
```

Finally, we provide an example where we use the standard `Builder` pattern, but
rather than just building up and sending the message all at once, we build it up
over the course of an entire status iteration. This is sometimes used when
disparate parts of the code need to all add to the status message, but it would
be obnoxious to keep it all in one method:

```cpp
class Ping {
 public:
  Ping(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        sender_(event_loop_->MakeSender<aos::examples::Ping>("/test")),
        builder_(sender_.MakeBuilder()) {
    // Stagger things so that Ping is populated at a 0.5 sec offset from
    // when it actually gets sent.
    event_loop_->AddPhasedLoop([this](int) { PopulatePing(); },
                               std::chrono::seconds(1),
                               std::chrono::milliseconds(500));
    event_loop_->AddPhasedLoop([this](int) { SendPing(); },
                               std::chrono::seconds(1));
  }

  void PopulatePing() {
    aos::examples::Ping::Builder ping_builder =
        builder_.MakeBuilder<aos::examples::Ping>();
    ping_builder.add_value(971);
    ping_builder.add_send_time(
        event_loop_->monotonic_now().time_since_epoch().count());
    ping_offset_ = ping_builder.Finish();
  }

  void SendPing() {
    if (!ping_offset_.IsNull()) {
      // Show how we can access the Ping flatbuffer without having ever
      // completed the table, so long as we have an offset and the
      // FlatBufferBuilder used to create it.
      // This won't work after calling Send() or Detach() on the Builder.
      LOG(INFO) << aos::FlatbufferToJson(
          flatbuffers::GetTemporaryPointer(*builder_.fbb(), ping_offset_));
      sender_.CheckOk(builder_.Send(ping_offset_));
      builder_ = sender_.MakeBuilder();
      // Clear the offset so we don't send the same message twice.
      ping_offset_.o = 0;
    }
  }

 private:
  aos::EventLoop *event_loop_;
  aos::Sender<aos::examples::Ping> sender_;
  aos::Sender<aos::examples::Ping>::Builder builder_;
  // Offset referencing the finished Ping table. Note that since this is the
  // top-level message we could've actually Detach'd the buffer and use
  // SendDetached() (allowing us to both construct the message early and to
  // avoid needing to memcpy anything), but when this pattern is used in
  // practice it tends to involve incrementally adding sub-messages, and
  // so can't be detached early. As such, we demonstrate the more manual
  // pattern here.
  flatbuffers::Offset<aos::examples::Ping> ping_offset_;
};
```

### Watchers

Watchers allow you have a callback be called every time a new message is
received. Watchers currently have relatively little configurability,
although there have been discussions about changing that.

There are two main options for creating watchers: `MakeWatcher` and
`MakeNoArgWatcher`:

1. `MakeWatcher` allows you to register a callback that will get
   called with a single argument that is a reference to the
   received message. This is the typical use-case.
2. `MakeNoArgWatcher` registers a callback that takes no arguments
   and which has no access to the message data via the
   [context](#event-loop-context). This has some performance benefits
   in cases where you don't actually need to read the message
   contents from the watcher and just need to trigger an
   event whenever a message happens.

When the watcher callback is called, metadata about the message you are
receiving can be accessed via the [context](#event-loop-context). This
includes information about when the message was sent on its source node as well
as when it was received on the current node (which will be identical for
messages sent and received on the same node).

Note depending on whether [Message Pinning](#message-pinning) is active, the
message passed to the watcher will either have been copied out of shared memory
into a local buffer or have avoided the copy and still be in shared memory. See
[Message Pinning](#message-pinning) for more details.

Watchers will not be called until the `EventLoop` starts [running](#running).

A single `EventLoop` cannot register both watchers and senders on the same
channel.

Because a Watcher will get called for every single message on a channel, it is
possible for it to get behind if your process is processing messages slower
than they are appearing on the channel. If you end up falling behind by more
than the [Channel Storage Duration](#channel-storage-duration), your process
will be killed. This is likely to be turned into a configurable option
in the future for applications that do not mind missing messages.

While the default is generally fine in most cases, the
[Configuration](#configurations) also includes a maximum number of watchers that
can be registered on any channel (across all processes).

#### Examples

Registering a simple watcher to print message contents every time a message is
received:

```cpp
event_loop_->MakeWatcher("/test", [](const aos::examples::Ping &msg) {
  LOG(INFO) << aos::FlatbufferToJson(&msg);
});
```

Using a no-arg watcher plus a fetcher to trigger an event whenever a message is
received, but only incur the cost of reading the message at most once every
second:

```cpp
class Pong {
 public:
  Pong(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        fetcher_(event_loop_->MakeFetcher<aos::examples::Ping>("/test")) {
    event_loop->MakeNoArgWatcher<aos::examples::Ping>("/test", [this]() {
      constexpr std::chrono::seconds kMaxPeriod{1};
      // See if the new message is more than 1 second newer than the
      // previous message that we saw.
      if (fetcher_.context().monotonic_event_time + kMaxPeriod <
          event_loop_->context().monotonic_event_time) {
        // Update the fetcher; if the watcher got triggered, we
        // should be guaranteed to have received a new message,
        // so Fetch() should always return true.
        CHECK(fetcher_.Fetch());
        LOG(INFO) << aos::FlatbufferToJson(fetcher_.get());
      }
    });

    // Additionally, since Watchers won't get called for messages sent before
    // startup, check for any pre-existing message and print it out if present.
    event_loop->OnRun([this]() {
      if (fetcher_.Fetch()) {
        LOG(INFO) << aos::FlatbufferToJson(fetcher_.get());
      }
    });
  }

 private:
  aos::EventLoop *event_loop_;
  aos::Fetcher<aos::examples::Ping> fetcher_;
};
```

### Fetchers

Fetchers allow you to retrieve messages on a channel without needing to generate
a wakeup on your process for every new message. This can be advantageous over
[Watchers](#watchers) when:

1. You don't actually care about doing anything right when a message arrives
   (and so if you used a `Watcher`, would need to just copy the message to
   store it for later).
2. You don't care about every single incoming message, and want to avoid
   unnecessary wakeups.
3. You need to access the most-recently-sent message that may have been sent
   before your process started.

There are two main options for fetching data with a Fetcher:

1. `Fetch()`: Allows you to fetch the latest message on a channel.
2. `FetchNext()`: Allows you to fetch the message immediately after the
   previously fetched message. This lets you use a fetcher to observe
   every single message on the channel.

Both `Fetch*()` calls return true if they got a new message and false otherwise,
making
```cpp
if (fetcher_.Fetch()) {
  DoStuffWithTheNewMessage(fetcher_.get());
}
```
a common pattern.

Fetchers can be used outside of a context where the parent `EventLoop` is
running. For instance, if a configuration message is sent prior to
your process starting up, then it may make sense to fetch on that channel
during your application's constructor so that you can initialize your
application based on the contents of that configuration. Because we
support this use-case, there are some subtleties to how startup must be managed
in [Log Replay](#log-reading) and [Simulation](#simulation). See those sections
for more detail.

In order to access message metadata, the `Fetcher` class provides a `Context`
with all the same information that would be available in the Watcher callback
for the same message. This is particularly useful for retrieving the send time
of the message being fetched if you care about how stale the message in question
is.

Unlike Watchers and Senders, you can create as many Fetchers as you want on your
`EventLoop`, regardless of the number of other Watchers/Senders/Fetchers that
may have already been created. The only exception to this is if you have
[Message Pinning](#message-pinning) enabled for the channel.

#### Examples

Make a Fetcher and use it to retrieve and print out the most recent message:
```cpp
aos::Fetcher<aos::examples::Ping> fetcher =
    event_loop_->MakeFetcher<aos::examples::Ping>("/test");
if (fetcher.Fetch()) {
  LOG(INFO) << aos::FlatbufferToJson(fetcher.get());
}
```

Periodically getting the latest value of a fetcher and printing it out,
regardless of whether the information in the fetcher is new or not:

```cpp
class Pong {
 public:
  Pong(aos::EventLoop *event_loop)
      : fetcher_(event_loop->MakeFetcher<aos::examples::Ping>("/test")) {
    event_loop->AddPhasedLoop([this](int) {
      // Unconditionally fetch the latest message---we don't care whether the
      // message has changed, we will always print out an update for the user
      // regardless.
      fetcher_.Fetch();
      if (fetcher_.get() != nullptr) {
        LOG(INFO) << "The Ping value is " << fetcher_.get()->value();
      } else {
        LOG(WARNING) << "No messages received yet :(";
      }
    }, std::chrono::seconds(1));
  }

 private:
  aos::Fetcher<aos::examples::Ping> fetcher_;
};
```

Periodically running `FetchNext()` to print out every single new message
received:

```cpp
class Pong {
 public:
  Pong(aos::EventLoop *event_loop)
      : fetcher_(event_loop->MakeFetcher<aos::examples::Ping>("/test")) {
    event_loop->AddPhasedLoop(
        [this](int) {
          // Call FetchNext() until we stop getting new messages.
          while (fetcher_.FetchNext()) {
            LOG(INFO) << "The Ping value is " << fetcher_.get()->value();
          }
        },
        std::chrono::seconds(1));
  }

 private:
  aos::Fetcher<aos::examples::Ping> fetcher_;
};
```


### Advanced Message Passing

This section discusses how the various "Raw" Senders/Watchers/Fetchers work to
allow applications more flexibility in accessing the message-passing layers of
AOS. These are typically only used by specialized applications and by low-level
pieces of AOS infrastructure.

#### RawSenders

Like [RawFetchers](#rawfetchers) and [Raw Watchers](#raw-watchers),
`Sender`s have a corresponding `RawSender`.
This is what is used under the hood of the templated `Sender` class discussed in
the previous section. Unlike the `Sender`, a `RawSender` class has no type
information associated with it and instead will just let you send arbitrary
binary data on the channel. It is also possible to make a `RawSender` on
nodes that aren't able to send on a given channel, but regular users
should *never* use that feature. Sending on non-sendable nodes is only
meant for the [Message Bridge](#message-bridge), which uses this to
implement sending out messages on the destination node from the node
where they were sent.

`MakeRawSender` is generally used in applications where the type of the channel
is not known when the code is compiled. E.g.:

1. [Message Bridge](#message-bridge) is just copying data from the network to
   the shared-memory channels on a device, and so doesn't care about the
   structure of the contents.
2. The [Log Reader](#log-reading) is copying raw data from a file into
   simulated message channels, and similarly doesn't care about the structure
   of the data.
3. [aos_send](#aos_send) needs to be usable at the command line for whatever
   type the user wants send, and so uses the reflection data in the
   [config](#configurations) to parse JSON to a binary FlatBuffer.

The interface to `RawSender` has some similarities to that of the regular
`Sender`, but has a bunch of extra overloads for `Send` that allow variety
of options, including some for simulation-related optimizations as well as
to allow the message bridge to specify details about when the message was
sent on the original node. If you find yourself using `RawSender`, you
will probably not be using any of the overloads except for `Send(size_t)` (which
you use if you constructed your message in-place in the `RawSender`'s
shared memory) or `Send(const void*, size_t)` (if you own the memory
you constructed the message in and want the `RawSender` to copy it).


If you wished to replicate the base functionality of `aos_send` using a
`RawSender`, you would do something like:

```cpp
void SendMessageOnChannel(aos::EventLoop *event_loop,
                          std::string_view channel_name, std::string_view type,
                          std::string_view json) {
  // Retrieve the channel from the config and (after checking that it is valid),
  // make the raw sender.
  std::unique_ptr<aos::RawSender> sender =
      event_loop->MakeRawSender(CHECK_NOTNULL(configuration::GetChannel(
          event_loop->configuration(), channel_name, type, event_loop->name(),
          event_loop->node(), true)));
  // Use the sender's allocator to make a FlatBufferBuilder so that
  // the message will be constructed directly in shared memory.
  flatbuffers::FlatBufferBuilder fbb(sender->fbb_allocator()->size(),
                                     sender->fbb_allocator());
  fbb.ForceDefaults(true);
  // Use the reflection-based JSON-to-flatbuffer parser, taking advantage of
  // the reflection schema included in the configuration's channel definition.
  fbb.Finish(aos::JsonToFlatbuffer(json, sender->channel()->schema(), &fbb));
  sender->CheckOk(sender->Send(fbb.GetSize()));
}

// Sample usage, assuming you already have an EventLoop:
SendMessageOnChannel(event_loop, "/test", "aos.examples.Ping",
                     R"({"value": 10, "send_time": 1000})");
```

#### Raw Watchers

Raw Watchers serve essentially the same role as [RawSenders](#rawsenders)
and [RawFetchers](#rawfetchers)---they
allow you to register callbacks on channels whose type information you may not
know at compile-time. This can be used, e.g., to implement an
[aos_dump](#aos_dump) utility which may allow a command-line user to listen to
an arbitrary channel.

An example of how you might implement an `aos_dump`-like utility:

```cpp
void RegisterRawWatcher(aos::EventLoop *event_loop,
                        std::string_view channel_name, std::string_view type) {
  const aos::Channel *channel = CHECK_NOTNULL(
      configuration::GetChannel(event_loop->configuration(), channel_name, type,
                                event_loop->name(), event_loop->node(), true));
  event_loop->MakeRawWatcher(
      channel, [channel](const aos::Context &context, const void *) {
        LOG(INFO) << aos::FlatbufferToJson(
            channel->schema(), static_cast<const uint8_t *>(context.data));
      });
}

// And then in your main(), before you call Run() on your event loop, you might
// have:
RegisterRawWatcher(&event_loop, "/test", "aos.examples.Ping");
```

#### RawFetchers

Like [RawSenders](#rawsenders) and [Raw Watchers](#raw-watchers), `RawFetcher`s
allow you to fetch message data without knowing message type information at
compile-time. This is used by the [Logger](#log-writing) to efficiently retrieve
all incoming messages. The interface to the `RawFetcher` is essentially
identical to that of the regular `Fetcher`, with the exception that you access
the raw data using `context().data` rather than with the typed `get()` accessor.

Example, similar to the [Raw Watcher Example](#raw-watchers), of using a fetcher
to print the latest data on a channel:

```cpp

void FetchLatestMessage(aos::EventLoop *event_loop,
                        std::string_view channel_name, std::string_view type) {
  const aos::Channel *channel = CHECK_NOTNULL(
      configuration::GetChannel(event_loop->configuration(), channel_name, type,
                                event_loop->name(), event_loop->node(), true));
  std::unique_ptr<aos::RawFetcher> fetcher =
      event_loop->MakeRawFetcher(channel);
  if (fetcher->Fetch()) {
    LOG(INFO) << aos::FlatbufferToJson(
        channel->schema(),
        static_cast<const uint8_t *>(fetcher->context().data));
  } else {
    LOG(ERROR) << "No message available.";
  }
}

// And then in your main(), before you call Run() on your event loop, you might
// have:
FetchLatestMessage(&event_loop, "/test", "aos.examples.Ping");
```

### Realtime Scheduler

Part of the generic `EventLoop` interface is a
`SetRuntimeRealtimePriority(int priority)` method. This allows the process to
make it so that, once the `EventLoop` begins running, the process will be set to
use a realtime scheduler with the specified priority. This does not actually
change the scheduler in simulation, but what it does do is allow the
`EventLoop` to indicate that any code running after startup should obey certain
constraints. As such, in both simulation and in a real system, we will
enforce that no dynamic memory allocation occurs while we are
[Running](#running).

The `EventLoop` will manage entering and exitting the realtime scheduler
at the correct times. This means that it will
not run any non-deterministic initialization code at elevated
realtime priorities and that it will exit the realtime scheduler prior
to any cleanup that happens when your process exits. This includes
exitting the realtime scheduler before a coredump is generated if
a process crashes.

Applications that don't care about deterministic timing do not need to set
themselves to use the realtime scheduler.
The shared memory queues AOS uses are designed to avoid any non-realtime process
blocking a realtime process. A lower priority process blocking a higher-priority
one is called a priority inversion. Of course,
if you create priority-inversions in your application logic, you will still have
issues (e.g., don't make it so that a realtime process will block on a `Watcher`
on a channel that is only sent to by a non-realtime process). But the
infrastructure itself should not be an issue.

See `//aos/realtime.*` for the code that manages entering and exitting realtime
schedulers, as well as for managing the malloc hook that intercepts attempted
memory allocations.

### Timing Reports

Timing reports are an FlatBuffer message sent out periodically by an
`EventLoop` itself to document what is going on with all the various
handlers and events in the `EventLoop`. This is primarily intended for debugging
purposes, where you may want to answer questions like:

1. Is my application handling `Watcher` callbacks promptly, or is it falling
   behind?
2. How long is it taking my timer handlers to run?
3. How consistent is my timer/phased loop callback timing?
4. How consistent is the time it takes to execute my event callbacks?
5. How many messages is this particular process sending/receiving?
   Is it encountering any sent-too-fast errors when sending?

In particular, each timing report sent out by an `EventLoop` will contain
aggregate statistics for all the events that occurred since the previous timing
report was sent out (including statistics associated with the previous timing
report itself). The `EventLoop` will send out timing reports on the
`/aos aos.timing.Report` channel at 1 Hz (unless configured otherwise via the
`-timing_report_ms` flag). Typically, all applications for a
given node will share a timing report channel. An `EventLoop` can be configured
not to send timing reports by calling `SkipTimingReport()`.

Currently, the statistics accumulated into each timing report are:

1. Watchers (per channel):
    1. Total number of watcher callbacks.
    2. Wakeup latency (time between the message send time and when the
       callback was called).
    3. Handler time (time it took the callback to execute).
2. Fetchers (per channel):
    1. Total count of messages fetched.
    2. Latency between when the message was sent and when `Fetch*()`
       read the message.
3. Senders (per channel):
    1. Count of all sent messages.
    2. Count of all send errors.
    3. Size of sent messages.
4. Timers and Phased Loops (per timer, including name if
   `set_name()` was called on the `TimerHandler`/`PhasedLoopHandler`):
    1. Total wakeups.
    2. Wakeup latency (time between scheduled wakeup and when the callback was
       called).
    3. Handler Time (time it took the callback to execute).

Note that message sizes, wakeup latencies, and handler times are all accumulated
as a `Statistics` table that includes average, min, max, and standard deviation
of the statistic. The `aos.timing.Report` FlatBuffer itself is defined in
`//aos/events/event_loop.fbs`. The timing report also includes `name` and
`pid` (process ID) fields to allow you to determine which process generated
a given report.

Since the raw `aos.timing.Report` message can be hard to parse visually, there
is a `aos_timing_report_streamer` binary (build
`//aos/events:aos_timing_report_streamer`) for streaming pretty-ified timing
reports in a live system and a `timing_report_dump` (build
`//aos/events:timing_report_dump`) for examining timing reports in a logfile.
These binaries allow for filtering timing reports by application and can
optionally aggregate timing reports across time into a single summary report.

As an example, a single timing report from the `ping` process in the [Getting
Started](#getting-started) example looks like:

```bash
$ bazel run //aos/events:aos_timing_report_streamer -- --config $(pwd)/bazel-bin/foo/pingpong_config.json  --application=ping
ping[2200238] () (13394719.844105617sec,2022-10-07_16-24-09.459757044):
  Watchers (1):
    Channel Name |              Type | Count |                                   Wakeup Latency |                                        Handler Time
           /test | aos.examples.Pong |   100 | 2.63423e-05 [4.609e-06, 7.065e-05] std 2.057e-05 | 2.00571e-05 [5.069e-06, 2.9803e-05] std 5.69737e-06
  Senders (3):
    Channel Name |                      Type | Count |                   Size | Errors
           /test |         aos.examples.Ping |   100 |      32 [32, 32] std 0 |   0, 0
            /aos |         aos.timing.Report |     1 |   608 [608, 608] std 0 |   0, 0
            /aos | aos.logging.LogMessageFbs |     0 | nan [nan, nan] std nan |   0, 0
  Timers (2):
              Name | Count |                                     Wakeup Latency |                                       Handler Time
              ping |   100 | 2.21471e-05 [2.904e-06, 5.344e-05] std 1.74616e-05 | 1.3115e-05 [5.108e-06, 1.9364e-05] std 3.42399e-06
    timing_reports |     1 |             1.893e-05 [1.893e-05, 1.893e-05] std 0 |          1.0817e-05 [1.0817e-05, 1.0817e-05] std 0
```

### Threading Model and Scheduling

All `EventLoop` callbacks will get called in a single thread. This is a
deliberate choice to avoid dealing with multi-threading in typical
applications. In general, if a task requires concurrency or
parallelism, we aim to achieve that by splitting the work across multiple
processes that then use AOS channels to communicate. For cases where
some multi-threaded work is required, that is permissible. However, each
`EventLoop` and all of its associated objects may only be interacted with
from a single thread.

If the `EventLoop` does get behind in scheduling events (e.g., if you do
a lot of work in a `Watcher` callback and are still doing the work when
the next message comes in), it will still execute those events in order
according to the monotonic event time. The monotonic event time is
the same `monotonic_event_time` that appears in the
[Context](#event-loop-context). The event time for different events is:

1. [Timers](#timers) and [Phased Loops](#phased-loops): The scheduled
   time of the next callback.
2. [Watchers](#watchers): The send time of the message on the current
   node (for messages sent and received on the same node, this is the
   send time of the message itself; for messages sent and received
   on different nodes, Watchers are scheduled based on the time at
   which the [Message Bridge](#message-bridge) was able to send
   the message on the local node after receiving it from the
   sending node).

This does mean that if your code is running behind, it is possible to
insert events into the queue at arbitrary times to get them
scheduled earlier. E.g.:

```cpp
class Pong {
 public:
  Pong(EventLoop *event_loop)
      : event_loop_(event_loop),
        delayed_timer_(
            event_loop_->AddTimer([this]() { DelayedTimerCallback(); })),
        early_timer_(
            event_loop_->AddTimer([this]() { EarlyTimerCallback(); })) {
    event_loop_->OnRun([this]() {
      aos::monotonic_clock::time_point start_time =
          event_loop_->context().monotonic_event_time;
      LOG(INFO) << "Start time " << start_time;
      std::this_thread::sleep_for(std::chrono::seconds(10));
      // Schedule delayed_timer_ for start + 5 seconds, which is currently ~5
      // seconds in the past.
      delayed_timer_->Setup(start_time + std::chrono::seconds(5));
      // Next, insert early_timer_ ahead of delayed_timer_. It's callback will
      // get called first.
      early_timer_->Setup(start_time);
    });
  }

 private:
  void EarlyTimerCallback() {
    LOG(INFO) << "Early Timer. Scheduled time of "
              << event_loop_->context().monotonic_event_time << " current time "
              << event_loop_->monotonic_now();
    // Schedule our next callback for 1 second after our schedule callback time.
    // Note that when we are running behind schedule, this time may be in the
    // past.
    early_timer_->Setup(event_loop_->context().monotonic_event_time +
                        std::chrono::seconds(1));
  }
  void DelayedTimerCallback() {
    LOG(INFO) << "Delayed Timer. Scheduled time of "
              << event_loop_->context().monotonic_event_time << " current time "
              << event_loop_->monotonic_now();
  }
  EventLoop *event_loop_;
  aos::TimerHandler *delayed_timer_;
  aos::TimerHandler *early_timer_;
};
```

Results in an output of:
```
I20221007 16:45:10.538957 2213689 pong_lib.h:21] Start time 13395980.923302827sec
I20221007 16:45:20.539100 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395980.923302827sec current time 13395990.923475792sec
I20221007 16:45:20.539141 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395981.923302827sec current time 13395990.923495430sec
I20221007 16:45:20.539150 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395982.923302827sec current time 13395990.923503789sec
I20221007 16:45:20.539158 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395983.923302827sec current time 13395990.923511693sec
I20221007 16:45:20.539167 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395984.923302827sec current time 13395990.923520481sec
I20221007 16:45:20.539175 2213689 pong_lib.h:44] Delayed Timer. Scheduled time of 13395985.923302827sec current time 13395990.923528989sec
I20221007 16:45:20.539182 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395985.923302827sec current time 13395990.923536119sec
I20221007 16:45:20.539191 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395986.923302827sec current time 13395990.923544159sec
I20221007 16:45:20.539198 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395987.923302827sec current time 13395990.923551990sec
I20221007 16:45:20.539206 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395988.923302827sec current time 13395990.923559876sec
I20221007 16:45:20.539214 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395989.923302827sec current time 13395990.923567812sec
I20221007 16:45:20.539222 2213689 pong_lib.h:34] Early Timer. Scheduled time of 13395990.923302827sec current time 13395990.923575700sec
```

#### "Running"

Certain actions are limited to whether they can or can't be called while
the `EventLoop` is running. You can always check if the `EventLoop`
is running using `is_running()`. The `EventLoop` is "running" whenever
you are in one of the callbacks registered on the `EventLoop` (namely, a
`Watcher`, `Timer`, `PhasedLoop`, or `OnRun` callback). The
[Context](#event-loop-context) will be available only in these callbacks
while the `EventLoop` is running. `Sender`s, `Watcher`s, and `Fetcher`s may
not be created while the `EventLoop` is running.
[Realtime Checks and Schedulers](#realtime-scheduler) will be enabled will
running. `Watcher`, `Timer`, and `PhasedLoop` callbacks will not
be registered until the `EventLoop` `OnRun` methods have been called.
This means if a message gets sent on a channel you are watching between when you
call `MakeWatcher` and when `OnRun` gets called, you will not get
a callback for that message. Messages _can_ be sent and fetched when the
`EventLoop` is not running.

The concept of "running" is also relevant for some simulation-related issues,
where there are subtle distinctions between things that happen before and
after the simulation starts running. For instance, if you want to make a message
be available to fetchers in simulation but don't want to trigger watchers,
you need to ensure that the message gets sent before the simulation is
in the running state.

## ShmEventLoop

## Realtime Code

The ROS2 design docs website has a [reasonable
introduction](https://design.ros2.org/articles/realtime_background.html)
to working with realtime code.

## Configurations

## FlatBuffers

See [FlatBuffers](/flatbuffers).

## Channels

### Message Pinning

### Channel Storage Duration

### Sent Too Fast

## Multi-Node

### Message Bridge

### Correlating Monotonic Clocks Across Nodes

## Simulation

## Logging

### Log Writing

### Log Reading

#### Log Replay Reproducibility

## Starter

## Command-line Utilities

### aos_dump

### aos_send

[^rawsenders]: See [RawSenders](#rawsenders) for how to make senders when you
               do not know the type at compile-time.
