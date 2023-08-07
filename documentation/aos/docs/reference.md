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
  timer_->Schedule(event_loop_->monotonic_now() + std::chrono::seconds(1));
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

  // Index of the message buffer. This will be in [0, NumberBuffers) on
  // read_method=PIN channels, and -1 for other channels.
  //
  // This only tells you about the underlying storage for this message, not
  // anything about its position in the queue. This is only useful for advanced
  // zero-copy use cases, on read_method=PIN channels.
  //
  // This will uniquely identify a message on this channel at a point in time.
  // For senders, this point in time is while the sender has the message. With
  // read_method==PIN, this point in time includes while the caller has access
  // to this context. For other read_methods, this point in time may be before
  // the caller has access to this context, which makes this pretty useless.
  int buffer_index;

  // UUID of the remote node which sent this message, or this node in the case
  // of events which are local to this node.
  UUID source_boot_uuid = UUID::Zero();

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
      delay_timer->Schedule(event_loop_->context().monotonic_event_time +
                            std::chrono::seconds(1));
    });
```


### Timers

Timers allow events to be scheduled at some scheduled time. When you create a
Timer via the `AddTimer` method on an `EventLoop`, you specify the function that
will get called when the timer is triggered. In order to actually schedule an
event on the timer you must call `Schedule`. Calling `Schedule` clears any
existing state of the timer. You can either schedule a timer to be called once
at a particular time in the future or schedule it to be called at a regular
interval.

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
  timer_handle_->Schedule(event_loop_->monotonic_now() + std::chrono::seconds(1));
```

Scheduling a timer to get called one second after startup, and at 100Hz
thereafter.
```cpp
...
  // In the constructor of your class:
  timer_handle_ = event_loop_->AddTimer([this]() { SomeMethod(); });
  event_loop_->OnRun([this]() {
    timer_handle_->Schedule(
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
    timer_handle_->Schedule(event_loop_->monotonic_now(), std::chrono::seconds(1));
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
  //timer_handle_->Schedule(event_loop_->context().monotonic_event_time);
  event_loop_->OnRun([this]() {
      // The context *is* defined during the OnRun() handlers.
      // So far, this code is fine.
      timer_handle_->Schedule(event_loop_->context().monotonic_event_time);

      // Do some really long initialization (ideally, this would be done
      // *before* the OnRun(), but maybe it relies on the EventLoop being
      // running).
      std::this_thread::sleep_for(std::chrono::seconds(100));
  });
...

void Ping::SendPing() {
  LOG(INFO) << "Ping!";
  // This is (typically) bad!
  timer_handle_->Schedule(event_loop_->context().monotonic_event_time + std::chrono::seconds(1));
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
`Schedule` call demonstrated earlier. Doing this means that in the same
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
your original `Schedule` call. In the typical use-case with a regular Timer
people will write something like
`timer_handle_->Schedule(event_loop_->monotonic_now(), period)`. But because
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

There are four main options for fetching data with a Fetcher:

1. `Fetch()`: Allows you to fetch the latest message on a channel.
2. `FetchNext()`: Allows you to fetch the message immediately after the
   previously fetched message. This lets you use a fetcher to observe
   every single message on the channel.
3. `FetchIf()` and `FetchNextIf()`: Identical to the above, but take a
   predicate that causes the fetcher to only fetch if the next message
   causes the predicate to return true (e.g., if you only want to fetch
   messages up until a certain point in time).

All `Fetch*()` calls return true if they got a new message and false otherwise,
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

Additionally, the ROS2 design docs website has a [reasonable
introduction](https://design.ros2.org/articles/realtime_background.html)
to working with realtime code.

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
      delayed_timer_->Schedule(start_time + std::chrono::seconds(5));
      // Next, insert early_timer_ ahead of delayed_timer_. It's callback will
      // get called first.
      early_timer_->Schedule(start_time);
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
    early_timer_->Schedule(event_loop_->context().monotonic_event_time +
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

The `ShmEventLoop` is an implementation of the `EventLoop` API that is intended
to run in realtime on a Linux system and uses shared memory to communicate
between processes. This is currently the only `EventLoop` implementation for
actually running realtime processes (the `SimulatedEventLoop` being the only
other implementation of the `EventLoop` API, and it is only meant for
simulation).

Additionally, there are a set of common utilities & applications that go with
the `ShmEventLoop` to form a complete realtime AOS system:

* The `message_bridge_server` and `message_bridge_client` processes use the
  `ShmEventLoop` to read data from shared memory channels, forward it to other
  node(s) over SCTP, and then republish the data onto the shared memory channels
  on the other node(s).
* The `starterd` manages starting & stopping applications, and allows
  dynamically starting, stopping, and viewing the status of applications by
  way of AOS channels.
* The logger (which is sometimes configured in specialized ways, but generally
  uses the `LogWriter` class).
* `aos_dump`, `aos_send`, `aos_starter`, `aos_timing_report_streamer`, and other
  utilities allow engineers to debug the system at the command line by exposing
  AOS channels in various ways (see [Command-line Utilities](#command-line-utilities)).

The `ShmEventLoop` does a few key things beyond just implementing what is
required by the `EventLoop` API:

* Has `Run()` and `Exit()` calls to directly start/stop execution of the event
  loop.
* Does signal handling to capture `SIGINT` and automatically exit.
* Allows controlling the scheduling of the process (realtime priority, CPU
  affinity)---technically these are actually part of the `EventLoop` API,
  but are meaningless for any other `EventLoop` implementations.
* Exposes the `EPoll` implementation used to back the `EventLoop` (this
  becomes useful when wanting to interact with sockets, files, and sometimes
  even other event loops/schedulers).

### Design of the `ShmEventLoop`

The goal of the `ShmEventLoop` is to provide an `EventLoop` implementation that
can run on most Linux systems, and which has good enough performance to support
soft/firm real-time system where large amounts of data (e.g., raw images) may
need to be moved over the pub-sub channels. In order to implement the core
`EventLoop` API, we use two things:

1. An IPC (InterProcess Communication) library using a lockless shared-memory
   queue for managing the pub-sub channels.
2. `epoll` for managing event scheduling.

For each `EventLoop` feature, this means:

* `Watcher`s are implemented by having the sending event loop general signals,
  where each `Watcher` then has an associated `signalfd` that we use `epoll` to
  watch and wakeup on when the new messages arrived.
* `Timer`s and `PhasedLoop`s are implemented using `timerfd` and `epoll` to
  wakeup when the timers expire.
* `Sender`s and `Fetcher`s do not have to directly interact with the event
  scheduling (beyond generating signals on sends, in the case of `Sender`s),
  but do use the IPC library.

Outside of the "core" `EventLoop` API, the `ShmEventLoop` is also responsible
for setting the current process priority, pinning it to the requested CPU
core(s), and doing things like preventing calls to `malloc` while the
`ShmEventLoop` is [Running](#running).

For additional detail on the underlying IPC design, reference the [Design
Doc](https://docs.google.com/document/d/10xulameLtEqjBFkm54UcN-5N-w5Q_XFNILvNf1Jl1Y4/edit#heading=h.y9blqcmsacou)
and the code at `//aos/ipc_lib:lockless_queue`.

### IPC Performance Considerations

This section provides a discussion of how to use the `EventLoop` API in
situations where you may have strong performance constraints (e.g.,
you are processing large numbers of camera images that require transferring
large amounts of data).

Some discussion in this section can theoretically apply to any `EventLoop`
implementation---everything discussed here just uses the generic `EventLoop`
API and does not actually require doing anything that is specific to the
`ShmEventLoop`. However, it is useful to discuss how the actual implementation
works to understand the performance implications of different options.

At a high level, the most common issues which we have observed regarding
performance are:

1. Copying messages can be very expensive, particularly for raw camera images;
   for this reason, the `EventLoop` API provides zero-copy constructs that
   allow the sender to construct messages in-place in shared memory and for
   readers to read the message directly from shared memory without copying
   it first. There are a variety of places where the "normal" APIs will do
   copies by default, as well as some convenience APIs that are more rarely
   used but which do do extraneous copies.
2. For processes which must consume large numbers of individual messages (e.g.,
   a logger), using `Watcher`s may get expensive due to the cost of waking
   up for every signal. Using `Fetcher`s + a polling timer can mitigate this.
3. Be aware that querying the clocks on the `ShmEventLoop` with
   `monotonic_now()` and `realtime_now()` calls will actually query the system
   clocks---if you want the time of the current event, use the [Event Loop
   Context](#event-loop-context) (note that "doing too many clock queries"
   is not typically a major issue).

#### Avoiding Copies

Copying around large chunks of memory can be expensive. As such, the `EventLoop`
API is designed to allow you to avoid extraneous copies if you do not want to do
so. This does mean paying attention to what you are doing at each step of the
process.

*Sending*: When sending a message, you should make sure to use the
`aos::Sender::MakeBuilder()` call, which provides a `FlatBufferBuilder` that
constructs your message in-place in shared-memory (the `Sender` will acquire
ownership of a single slot in the shared memory queue, and leave the message
unpublished until you call `Send`). If you need to fill your message with
a large blob (e.g., an image), you can use the
[CreateUninitializedVector](https://flatbuffers.dev/classflatbuffers_1_1_flat_buffer_builder.html#a2305b63d367845972b51669dd995cc50)
method to get a pointer to a fixed-length buffer where you can fill in your
data. Be aware that the `aos::Sender::Send()` method which takes a
`NonSizePrefixedFlatbuffer` will do a copy, because it takes a FlatBuffer
which has been constructed outside if its shared memory buffer. This is distinct
from the `aos::Sender::Builder::Send()` calls, which assume that you have built
your flatbuffer up using the provided `Builder` and so don't need to do extra
copies---the `aos::Sender::Builder::Send()` calls are what almost all existing
code uses. As an aside,
because `Sender`s must construct messages in-place, there is a configurable
limit on the maximum number of senders per-channel. If we allowed arbitrarily
many senders per channel, then they could consume all of the slots in the
shared memory queue and prevent any messages from actually flowing.

*Receving*: By default, `Fetcher`s and `Watcher`s will copy a message on
receipt. This allows us to allow arbitrarily many processes to be fetching on a
given channel by default. However, if the `read_method` for a channel is set to
`PIN` in the [configuration](#Configurations) then each reader will acquire a
slot in the shared memory queue (this causes there to be a limit on the maximum
number of allowed readers). If `PIN`d, then when you read the message you are
reading directly from shared memory. Note that there is an independent
`num_watchers` setting in the configuration for a channel; this maximum exists
because the shared memory queue must have a fixed-length list of processes to
notify when a message is sent.

*`NoArgWatcher`s*: Regardless of whether you want zero-copy performance or not,
being aware of `NoArgWatcher`s is useful as they allow you to receive a
callback when a message is received with needing to actually copy the message
out. This is typically paired with a `Fetcher` for that message, although there
may also be situations where you just need to trigger some work or event
whenever a channel is sent on, without caring about the message contents. See
[Watchers](#watchers) for more discussion.

#### High-rate Messages

When dealing with high-rate (usually starting at ~1 kHz) messages, the context
switches associated with using a watcher to wake up your process on every single
message can become non-trivial. In applications where you really do want to wake
up for every single messages, this may still be appropriate, but if instead you
just need to ensure that you are handling every message in a reasonably timely
manner it can be much cheaper to set up a periodic timer at a lower frequency
(e.g., the logger generally polls at ~10 Hz; but even just getting below a
kilohertz is typically good). In order to ensure that you read every message,
you can then use a [Fetcher](#fetchers) with `FetchNext()` to iterate over
all the messages received in the last polling period.

### Typical `ShmEventLoop` usage

There is a good sample `main()` in the [Getting Started](/getting_started/#writing-the-ping-main);
the key pattern that is typically followed is:

```cpp
// We pull in a config that is stored somewhere on disk (as pointed to by
// a --config flag).
aos::FlatbufferDetachedBuffer<aos::Configuration> config =
    aos::configuration::ReadConfig(FLAGS_config);

aos::ShmEventLoop event_loop(&config.message());

// The application being run is provided with an EventLoop*, which enables it to
// access the EventLoop API without knowing whether it is on a realtime system
// or just running in simulation.
aos::Ping ping(&event_loop);

// Actually run the EventLoop. This will block until event_loop.Exit() is
// called or we receive a signal to exit (e.g., a Ctrl-C on the command line).
event_loop.Run();
```

### Running (and Exiting) the `ShmEventLoop`

In the vast majority of cases, your code will simply call `Run()` on the
`ShmEventLoop` and not have to think about it any more. However, there are
situations where you need to worry about more complexity.

A few things to be aware of:

* When the `ShmEventLoop` is [Running](#running), the process priority will be
  elevated and if the priority is real-time it will die on attempted `malloc`s.
  I.e., "Running" == Real-time.
* If you wish to programmatically exit your process (e.g., it is a one-shot
  process that just does some work and exits), use the `ExitHandle` API.
  This way you can use `ShmEventLoop::MakeExitHandle` to provide your
  application with a way to exit the process at runtime, and then use a
  different `ExitHandle` implementation for testing.

### `EPoll` interface

As previously mentioned, the `ShmEventLoop` uses `epoll` under the hood to
trigger wakeups on new events. Typically, you do not need to care about this;
however, the `EPoll` class is exposed from the `ShmEventLoop` in order to enable
you to interact with the system outside of the standard `EventLoop` API while
still having your callbacks and events occur within the main thread of the
`EventLoop`. In order to enable testing & simulation, most applications that do
this will take an abstract `EventLoop*` and an `EPoll*` in their constructor;
however, for cases where the application truly only makes sense to run against
a `ShmEventLoop`, it may take a `ShmEventLoop` directly. By breaking out of
the normal `EventLoop` API your application will become harder to simulate,
replay, etc. However, this is useful in some situations, e.g.:

* An application that must provide a web server of some sort to external
  applications, such as for a live debugging webpage.
* Interacting with a CAN bus, UDP/TCP socket, etc. in order to interface with
  and control some external system.
* Building a camera driver that must read/send out camera images when they
  become available.

## Multi-Node

AOS is built to support distributed systems where applications are running on
separate devices which communicate with one another. The API is designed on the
theory that typical applications should not really need to care about what node
a channel that they are sending/listening on is forwarded to/from. However,
it is expected that some applications will care and that you will need to care
about the data flow for the system as a whole.

There are two key things that change when you begin working with a multi-node
AOS system as compared to a single-node system (or when working within a single
node on a multi-node system):

1. Clocks & time may not be fully synchronized across nodes. By default, AOS
   also allows individual nodes on a system to reboot, complicating clock
   management further.
2. Message delivery between nodes can be unreliable, and will generally have
   significantly worse latency than message delivery within a node. Note that
   there are options to, e.g., allow for reliable message delivery but any
   network-based messaging will be constrained by the networking setup of your
   system.


See [Configuring Multi-Node Systems](#configuring-multi-node-systems) for
information on how to set up an AOS configuration to be multi-node.

Note: Currently, while "single-node" is technically a distinct mode from
"multi-node" in AOS configurations, it is possible that we will deprecate the
single-node mode in the future in the favor of simply requiring that any
single-node systems be multi-node systems with one node.

### Message Forwarding & Reliability

Any given channel can be configured to be forwarded between nodes. That channel
may only be sent on from one node (the `source_node`), and can be forwarded to
arbitrarily many other nodes. Each individual forwarding connection can be
configured separately, such that forwarding channel `/foo` from node A to node B
may be at a higher priority or reliability than from node A to node C. For each
forwarding connection, you may configure:

* Where & how to log timestamp information associated with the forwarding.
* The SCTP priority with which to forward the message.
* Whether to guarantee delivery of the message, akin to TCP ("reliable forwarding").
* The time to live to set for messages with unreliable forwarding.

As alluded to, message forwarding is currently assumed to be implemented by
[SCTP](https://en.wikipedia.org/wiki/Stream_Control_Transmission_Protocol) at
runtime. In simulation, forwarding is entirely deterministic and reliable
unless the user configures the `SimulatedNetworkBridge` otherwise. Should
any additional `EventLoop` implementations be added in the future that do
_not_ use SCTP, they would be expected to provide similar options for
configuration, although one could reasonable create an implementation that did
not implement all the possible configurability. For that matter, some SCTP
implementations may not implement the priority option.

Ordering is guaranteed within a channel---an observer on a channel will never
observe a newer message before an older message. Ordering between channels is
not guaranteed. Consider, for instance, two channels:

* Channel `/big_and_slow` sends infrequent but large messages. At t=0sec it
  sends a 100MB message which takes ~1 sec to transfer across the network.
* Channel `/small_and_fast` sends frequent but small and high priority messages.
  At t=0.01sec it sends a 100 byte message which crosses the network in <1ms.

Depending on the SCTP implementation, it is entirely possible that the
`/small_and_fast` message will arrive at the destination node before the
`/big_and_slow` message. This is desired behavior (we wouldn't want a
low-priority channel able to block the delivery of high-priority messages).

Once the messages have arrived on the destination node, ordering will again be
~guaranteed[^ordering-guarantee] across channels, but with respect to the time
at which the message was sent in shared memory by the message bridge on
the current node.

[^ordering-guarantee]: There is actually an outstanding bug where in some
  corner-cases, ordering is not actually guaranteed. See
  https://github.com/frc971/971-Robot-Code/issues/29

#### Forwarded Message Metadata

Forwarded messages have additional metadata populated in the [Event Loop
Context](#event-loop-context). The following fields become relevant for
forwarded messages:

```cpp
// For a single-node configuration, these two are identical to *_event_time.
// In a multinode configuration, these are the times that the message was
// sent on the original node.
monotonic_clock::time_point monotonic_remote_time;
realtime_clock::time_point realtime_remote_time;

// Index into the remote queue.  Useful to determine if data was lost.  In a
// single-node configuration, this will match queue_index.
uint32_t remote_queue_index;

// UUID of the remote node which sent this message, or this node in the case
// of events which are local to this node.
UUID source_boot_uuid = UUID::Zero();
```

The remote times tell you the time at which the message was sent on the original
node, and represent the clocks _on that node_. As such, in order to meaningfully
compare the remote times to local times (e.g., if you want to figure out how
long ago the message was actually sent) you must either (a) trust the realtime
clocks; or (b) [compensate for the monotonic clock
offsets](#correlating-monotonic-clocks-across-nodes). If you want the "event"
time for when your current watcher or the such got triggered, use the regular
`monotonic_event_time`. The only real use of the `remote_queue_index` would be
to detect when messages were dropped over the network.

The `source_boot_uuid` can be used to determine if the source node rebooted
between observing messages. Arbitrarily many messages may have been dropped by
the forwarding during a reboot, as we cannot guarantee that every single message
sent from a rebooting node while it is rebooting gets forwarded.

#### Reliable vs. Unreliable Forwarding

"Unreliable" forwarding is essentially what it sounds like---messages will be
forwarded; if they do not make it within the `time_to_live`, then they will not
get delivered to the receiving node. Unreliable messages that were sent from the
sending node prior to the recipient node having become connected to the sending
node are simply dropped. Unreliable forwarding is generally the default state
for most channels.

"Reliable" forwarding, on the other hand, carries two primary guarantees beyond
the normal unreliable messages:

* As long as two nodes are connected, all messages on reliable channels will be
  forwarded, regardless of how much time must be spent retrying (this generally
  makes reliable forwarding a poor choice for high-rate, latency-critical
  messages).
* When two nodes connect, the most recently sent message (if any) on each
  reliable channel will get forwarded to the receiving node. This makes it so
  that, e.g., if you have a central node that sends out a configuration message
  once on startup then you can make it so that whenever the edge nodes connect
  they will get the configuration message forwarded to them, even if they were
  not online when the configuration message was originally published.

Note that if the message bridges on two nodes disconnect and then reconnect in
the future (without a node reboot occurring), then the semantics of reliable
messages are similar to what happens on boot. Namely:

* If no reliable messages were sent during the disconnect, then nothing happens.
* If 1 or more reliable messages were sent during the disconnect, then the
  latest message will get forwarded to the destination node.

#### Message Bridge

The applications that manage the forwarding of messages between nodes are
collectively known as the "message bridge." In simulation, this is managed by
the `SimulatedMessageBridge`, which at it's core is a bunch of
`std::deque`'s that are used to introduce simulated network latency (and/or
unreliability) to the simulation. At runtime, the `message_bridge_client` and
`message_bridge_server` handle forwarding messages from shared memory channels
to SCTP. The server is responsible for listening to messages sent on the
`source_node` and forwarding them to the network, while the client is
responsible for subscribing to the server, receiving messages, and republishing
them onto the shared memory channels locally.

The message bridge processes themselves are "just" regular AOS processes. They
use a `ShmEventLoop` to interface with AOS, and do need to break out of the
normal abstractions to interact with SCTP. The main things which they do which
do break out of the abstractions which most users should worry about are:

* The `message_bridge_client` uses `RawSender`s to allow itself to send on
  channels which other applications cannot (since you shouldn't be able to send
  on channels where you are not the `source_node`). Additionally, when sending
  these messages, the client manually populates the various remote message
  metadata.
* The `message_bridge_server` sends special `RemoteMessage` messages on the
  remote timestamp channels (the sending of these does not actually require
  anything special, but the logger treats these messages specially).

Typically, the operation of message bridge should be reasonably transparent to
most users; however, there are times when it is useful to watch the message
bridge status messages. The status messages contain a variety of information,
but the main pieces of information that you are likely to care about are:

* The connection state. Unless the state for a given node is `CONNECTED`, then
  messages will not be flowing. Note that states for the message bridge client
  and server are tracked separately, so it is possible that you may be connected
  to a node such that you are receiving messages from it but not successfully
  sending to it, or vice-versa.
* The current estimates of the monotonic clock offsets between the nodes. See
  [Correlating Monotonic Clocks Across
  Nodes](#correlating-monotonic-clocks-across-nodes)

### Cross-node Clocks

Dealing with time & clocks across nodes can be tricky. While we do require that
the monotonic clock on each device (generally corresponding to `CLOCK_MONOTONIC`
on the system, and virtually always a time since boot) be monotonic, the
realtime clock (i.e., the clock providing the time since the Unix Epoch) may not
be monotonic, or even if it is monotonic, it may jump or speed up/slow down
significantly. If you have NTP (or some similar protocol) set up on your system,
then you may be able to control the behavior of the clocks more tightly than we
guarantee, but AOS itself does not provide any guarantees around the realtime
clock.

Additionally, the log reading code currently makes assumptions about how quickly
the monotonic clocks on different nodes can drift apart from one another. This
is currently a [hard-coded value if 1ms /
sec](https://github.com/frc971/971-Robot-Code/blob/790cb54590e4f28f61e2f1bcd2e6e12ca47d7713/aos/network/timestamp_filter.h#L21-L26)
(i.e., we assume that over one second of time on one node's clock, the other
node's clock will have advanced by somewhere between 999 and 1001 milliseconds).
This number could plausibly be changed, but we have not yet encountered clocks
actually drifting by enough to require that.

#### Correlating Monotonic Clocks Across Nodes

When dealing with time across nodes, we rely on the ongoing flow of messages to
try and estimate the offsets between different clocks. The results of this
estimation are published in the `aos.message_bridge.ServerStatistics` and
`aos.message_bridge.ClientStatistics` messages (note that the offsets provided
in these messages may be slightly different, as they are estimated independently).
These estimates should be reasonable, but are currently not well validated in
all possible corner cases. If you discover a situation where they are too
unreliable for your use-case, that would be something we would want to fix.

As an example, consider a situation where you are receiving sensor data from a
node named `sensor`. This sensor data is on the channel `/input` with a type of
`SensorData`. You wish to determine how old the sensor data is, but do not have
accurate realtime clocks set up on your machine. As such, you would have
something like:

```cpp
class SensorAgeReader {
 public:
  SensorAgeReader(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        clock_offset_fetcher_(
            event_loop->MakeFetcher<aos::message_bridge::ServerStatistics>(
                "/aos")) {
    event_loop_->MakeWatcher(
        "/input", [this](const SensorData &msg) { HandleSensorData(msg); });
  }

  void HandleSensorData(const SensorData &msg) {
    std::chrono::nanoseconds monotonic_offset{0};
    clock_offset_fetcher_.Fetch();
    if (clock_offset_fetcher_.get() != nullptr) {
      for (const auto connection : *clock_offset_fetcher_->connections()) {
        if (connection->has_node() && connection->node()->has_name() &&
            connection->node()->name()->string_view() == "sensor") {
          if (connection->has_monotonic_offset()) {
            monotonic_offset =
                std::chrono::nanoseconds(connection->monotonic_offset());
          } else {
            // If we don't have a monotonic offset, that means we aren't
            // connected, in which case we should just exit early.
            // The ServerStatistics message will always populate statuses for
            // every node, so we don't have to worry about missing the "sensor"
            // node (although it can be good practice to check that the node you
            // are looking for actually exists, to protect against programming
            // errors).
            LOG(WARNING) << "Message bridge disconnected.";
            return;
          }
          break;
        }
      }
    } else {
      LOG(WARNING) << "No message bridge status available.";
      return;
    }
    const aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
    // The monotonic_remote_time will be the time that the message was sent on
    // the source node; by offsetting it by the monotonic_offset, we should get
    // a reasonable estimate of when it was sent. This does not account for any
    // delays between the sensor reading and when it actually got sent.
    const aos::monotonic_clock::time_point send_time(
        event_loop_->context().monotonic_remote_time - monotonic_offset);
    // Many sensors may include some sort of hardware timestamp indicating when
    // the measurement was taken, which is likely before the sent time. This can
    // be populated as a data field inside of the message, and if it is using
    // the same monotonic clock as AOS is then we can do the same offset
    // computation, but get a timestamp for when the data was actually captured.
    const aos::monotonic_clock::time_point capture_time(
        std::chrono::nanoseconds(msg.hardware_capture_time_ns()) -
        monotonic_offset);
    LOG(INFO) << "The sensor data was sent "
              << aos::time::DurationInSeconds(now - send_time)
              << " seconds ago.";
    LOG(INFO) << "The sensor data was read off of the hardware "
              << aos::time::DurationInSeconds(now - capture_time)
              << " seconds ago.";
  }

  aos::EventLoop *event_loop_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> clock_offset_fetcher_;
};
```

## Configurations

### Configuring Multi-Node Systems

## FlatBuffers

See [FlatBuffers](/flatbuffers).

## Channels

### Message Pinning

### Channel Storage Duration

### Sent Too Fast

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
