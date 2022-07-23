#![warn(unsafe_op_in_unsafe_fn)]

//! This module provides a Rust async runtime on top of the C++ `aos::EventLoop` interface.
//!
//! # Rust async with `aos::EventLoop`
//!
//! The async runtimes we create are not general-purpose. They may only await the objects provided
//! by this module. Awaiting anything else will hang, until it is woken which will panic. Also,
//! doing any long-running task (besides await) will block the C++ EventLoop thread, which is
//! usually bad.
//!
//! ## Multiple tasks
//!
//! This runtime only supports a single task (aka a single [`Future`]) at a time. For many use
//! cases, this is sufficient. If you want more than that, one of these may be appropriate:
//!
//! 1. If you have a small number of tasks determined at compile time, [`futures::join`] can await
//!    them all simultaneously.
//! 2. [`futures::stream::FuturesUnordered`] can wait on a variable number of futures. It also
//!    supports adding them at runtime. Consider something like
//!    `FuturesUnordered<Pin<Box<dyn Future<Output = ()>>>` if you want a generic "container of any
//!    future".
//! 3. Multiple applications are better suited to multiple `EventLoopRuntime`s, on separate
//!    `aos::EventLoop`s. Otherwise they can't send messages to each other, among other
//!    restrictions. https://github.com/frc971/971-Robot-Code/issues/12 covers creating an adapter
//!    that provides multiple `EventLoop`s on top of a single underlying implementation.
//!
//! ## Design
//!
//! The design of this is tricky. This is a complicated API interface between C++ and Rust. The big
//! considerations in arriving at this design include:
//!   * `EventLoop` implementations alias the objects they're returning from C++, which means
//!     creating Rust unique references to them is unsound. See
//!     https://github.com/google/autocxx/issues/1146 for details.
//!   * For various reasons autocxx can't directly wrap APIs using types ergonomic for C++. This and
//!     the previous point mean we wrap all of the C++ objects specifically for this class.
//!   * Keeping track of all the lifetimes and creating appropriate references for the callbacks is
//!     really hard in Rust. Even doing it for the library implementation turned out to be hard
//!     enough to look for alternatives. I think you'd have to make extensive use of pointers, but
//!     Rust makes that hard, and it's easy to create references in ways that violate Rust's
//!     aliasing rules.
//!   * We can't use [`futures::stream::Stream`] and all of its nice [`futures::stream::StreamExt`]
//!     helpers for watchers because we need lifetime-generic `Item` types. Effectively we're making
//!     a lending stream. This is very close to lending iterators, which is one of the motivating
//!     examples for generic associated types (https://github.com/rust-lang/rust/issues/44265).

use std::{fmt, future::Future, marker::PhantomData, pin::Pin, slice, task::Poll, time::Duration};

use autocxx::{
    subclass::{is_subclass, CppSubclass},
    WithinBox,
};
use cxx::UniquePtr;
use futures::{future::FusedFuture, never::Never};
use thiserror::Error;
use uuid::Uuid;

pub use aos_configuration::{Channel, ChannelLookupError, Configuration, ConfigurationExt, Node};
pub use aos_uuid::UUID;

autocxx::include_cpp! (
#include "aos/events/event_loop_runtime.h"

safety!(unsafe)

generate_pod!("aos::Context")
generate!("aos::WatcherForRust")
generate!("aos::RawSender_Error")
generate!("aos::SenderForRust")
generate!("aos::FetcherForRust")
generate!("aos::EventLoopRuntime")

subclass!("aos::ApplicationFuture", RustApplicationFuture)

extern_cpp_type!("aos::Configuration", crate::Configuration)
extern_cpp_type!("aos::Channel", crate::Channel)
extern_cpp_type!("aos::Node", crate::Node)
extern_cpp_type!("aos::UUID", crate::UUID)
);

pub type EventLoop = ffi::aos::EventLoop;

/// # Safety
///
/// This should have a `'event_loop` lifetime and `future` should include that in its type, but
/// autocxx's subclass doesn't support that. Even if it did, it wouldn't be enforced. C++ is
/// enforcing the lifetime: it destroys this object along with the C++ `EventLoopRuntime`, which
/// must be outlived by the EventLoop.
#[doc(hidden)]
#[is_subclass(superclass("aos::ApplicationFuture"))]
pub struct RustApplicationFuture {
    /// This logically has a `'event_loop` bound, see the class comment for details.
    future: Pin<Box<dyn Future<Output = Never>>>,
}

impl ffi::aos::ApplicationFuture_methods for RustApplicationFuture {
    fn Poll(&mut self) {
        // This is always allowed because it can never create a value of type `Ready<Never>` to
        // return, so it must always return `Pending`. That also means the value it returns doesn't
        // mean anything, so we ignore it.
        let _ =
            Pin::new(&mut self.future).poll(&mut std::task::Context::from_waker(&panic_waker()));
    }
}

impl RustApplicationFuture {
    pub fn new<'event_loop>(
        future: impl Future<Output = Never> + 'event_loop,
    ) -> UniquePtr<ffi::aos::ApplicationFuture> {
        /// # Safety
        ///
        /// This completely removes the `'event_loop` lifetime, the caller must ensure that is
        /// sound.
        unsafe fn remove_lifetime<'event_loop>(
            future: Pin<Box<dyn Future<Output = Never> + 'event_loop>>,
        ) -> Pin<Box<dyn Future<Output = Never>>> {
            // SAFETY: Caller is responsible.
            unsafe { std::mem::transmute(future) }
        }

        Self::as_ApplicationFuture_unique_ptr(Self::new_cpp_owned(Self {
            // SAFETY: C++ manages observing the lifetime, see [`RustApplicationFuture`] for
            // details.
            future: unsafe { remove_lifetime(Box::pin(future)) },
            cpp_peer: Default::default(),
        }))
    }
}

pub struct EventLoopRuntime<'event_loop>(
    Pin<Box<ffi::aos::EventLoopRuntime>>,
    // This is the lifetime of the underlying EventLoop, which is held in C++ via `.0`.
    PhantomData<&'event_loop mut ()>,
);

/// Manages the Rust interface to a *single* `aos::EventLoop`. This is intended to be used by a
/// single application.
impl<'event_loop> EventLoopRuntime<'event_loop> {
    /// Creates a new runtime. This must be the only user of the underlying `aos::EventLoop`, or
    /// things may panic unexpectedly.
    ///
    /// Call [`spawn`] to respond to events. The non-event-driven APIs may be used without calling
    /// this.
    ///
    /// This is an async runtime, but it's a somewhat unusual one. See the module-level
    /// documentation for details.
    ///
    /// # Safety
    ///
    /// `event_loop` must be valid for `'event_loop`. Effectively we want the argument to be
    /// `&'event_loop mut EventLoop`, but we can't do that (see the module-level documentation for
    /// details).
    ///
    /// This is a tricky thing to guarantee, be very cautious calling this function. It's an unbound
    /// lifetime so you should probably wrap it in a function that directly attaches a known
    /// lifetime. One common pattern is calling this in the constructor of an object whose lifetime
    /// is managed by C++; C++ doesn't inherit the Rust lifetime but we do have a lot of C++ code
    /// that obeys the rule of destroying the object before the EventLoop, which is equivalent to
    /// this restriction.
    ///
    /// In Rust terms, this is equivalent to storing `event_loop` in the returned object, which
    /// will dereference it throughout its lifetime, and the caller must guarantee this is sound.
    pub unsafe fn new(event_loop: *mut ffi::aos::EventLoop) -> Self {
        Self(
            // SAFETY: We push all the validity requirements for this up to our caller.
            unsafe { ffi::aos::EventLoopRuntime::new(event_loop) }.within_box(),
            PhantomData,
        )
    }

    /// Returns the pointer passed into the constructor.
    ///
    /// The returned value should only be used for destroying it (_after_ `self` is dropped) or
    /// calling other C++ APIs.
    pub fn raw_event_loop(&mut self) -> *mut ffi::aos::EventLoop {
        self.0.as_mut().event_loop()
    }

    // TODO(Brian): Expose `name`. Need to sort out the lifetimes. C++ can reallocate the pointer
    // independent of Rust. Use it in `get_raw_channel` instead of passing the name in.

    pub fn get_raw_channel(
        &self,
        name: &str,
        typename: &str,
        application_name: &str,
    ) -> Result<&'event_loop Channel, ChannelLookupError> {
        self.configuration()
            .get_channel(name, typename, application_name, self.node())
    }

    // TODO(Brian): `get_channel<T>`.

    /// Starts running the given `task`, which may not return (as specified by its type). If you
    /// want your task to stop, return the result of awaiting [`futures::future::pending`], which
    /// will never complete. `task` will not be polled after the underlying `aos::EventLoop` exits.
    ///
    /// TODO(Brian): Make this paragraph true:
    /// Note that task will be polled immediately. If you want to defer work until the event loop
    /// starts running, await TODO in the task.
    ///
    /// # Panics
    ///
    /// Panics if called more than once. See the module-level documentation for alternatives if you
    /// want to do this.
    ///
    /// # Examples with interesting return types
    ///
    /// These are all valid futures which never return:
    /// ```
    /// # fn compile_check(mut runtime: aos_events_event_loop_runtime::EventLoopRuntime) {
    /// # use futures::{never::Never, future::pending};
    /// async fn pending_wrapper() -> Never {
    ///     pending().await
    /// }
    /// async fn loop_forever() -> Never {
    ///     loop {}
    /// }
    ///
    /// runtime.spawn(pending());
    /// runtime.spawn(async { pending().await });
    /// runtime.spawn(pending_wrapper());
    /// runtime.spawn(async { loop {} });
    /// runtime.spawn(loop_forever());
    /// runtime.spawn(async { println!("all done"); pending().await });
    /// # }
    /// ```
    /// but this is not:
    /// ```compile_fail
    /// # fn compile_check(mut runtime: aos_events_event_loop_runtime::EventLoopRuntime) {
    /// # use futures::ready;
    /// runtime.spawn(ready());
    /// # }
    /// ```
    /// and neither is this:
    /// ```compile_fail
    /// # fn compile_check(mut runtime: aos_events_event_loop_runtime::EventLoopRuntime) {
    /// # use futures::ready;
    /// runtime.spawn(async { println!("all done") });
    /// # }
    /// ```
    ///
    /// # Examples with capturing
    ///
    /// The future can capture things. This is important to access other objects created from the
    /// runtime, either before calling this function:
    /// ```
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// #     channel1: &'event_loop aos_events_event_loop_runtime::Channel,
    /// #     channel2: &'event_loop aos_events_event_loop_runtime::Channel,
    /// # ) {
    /// let mut watcher1 = runtime.make_raw_watcher(channel1);
    /// let mut watcher2 = runtime.make_raw_watcher(channel2);
    /// runtime.spawn(async move { loop {
    ///     watcher1.next().await;
    ///     watcher2.next().await;
    /// }});
    /// # }
    /// ```
    /// or after:
    /// ```
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// #     channel1: &'event_loop aos_events_event_loop_runtime::Channel,
    /// #     channel2: &'event_loop aos_events_event_loop_runtime::Channel,
    /// # ) {
    /// # use std::{cell::RefCell, rc::Rc};
    /// let runtime = Rc::new(RefCell::new(runtime));
    /// runtime.borrow_mut().spawn({
    ///     let mut runtime = runtime.clone();
    ///     async move {
    ///         let mut runtime = runtime.borrow_mut();
    ///         let mut watcher1 = runtime.make_raw_watcher(channel1);
    ///         let mut watcher2 = runtime.make_raw_watcher(channel2);
    ///         loop {
    ///             watcher1.next().await;
    ///             watcher2.next().await;
    ///         }
    ///     }
    /// });
    /// # }
    /// ```
    /// or both:
    /// ```
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// #     channel1: &'event_loop aos_events_event_loop_runtime::Channel,
    /// #     channel2: &'event_loop aos_events_event_loop_runtime::Channel,
    /// # ) {
    /// # use std::{cell::RefCell, rc::Rc};
    /// let mut watcher1 = runtime.make_raw_watcher(channel1);
    /// let runtime = Rc::new(RefCell::new(runtime));
    /// runtime.borrow_mut().spawn({
    ///     let mut runtime = runtime.clone();
    ///     async move {
    ///         let mut runtime = runtime.borrow_mut();
    ///         let mut watcher2 = runtime.make_raw_watcher(channel2);
    ///         loop {
    ///             watcher1.next().await;
    ///             watcher2.next().await;
    ///         }
    ///     }
    /// });
    /// # }
    /// ```
    ///
    /// But you cannot capture local variables:
    /// ```compile_fail
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// # ) {
    /// let mut local: i32 = 971;
    /// let local = &mut local;
    /// runtime.spawn(async move { loop {
    ///     println!("have: {}", local);
    /// }});
    /// # }
    /// ```
    pub fn spawn(&mut self, task: impl Future<Output = Never> + 'event_loop) {
        self.0.as_mut().spawn(RustApplicationFuture::new(task));
    }

    pub fn configuration(&self) -> &'event_loop Configuration {
        // SAFETY: It's always a pointer valid for longer than the underlying EventLoop.
        unsafe { &*self.0.configuration() }
    }

    pub fn node(&self) -> Option<&'event_loop Node> {
        // SAFETY: It's always a pointer valid for longer than the underlying EventLoop, or null.
        unsafe { self.0.node().as_ref() }
    }

    pub fn monotonic_now(&self) -> MonotonicInstant {
        MonotonicInstant(self.0.monotonic_now())
    }

    /// Note that the `'event_loop` input lifetime is intentional. The C++ API requires that it is
    /// part of `self.configuration()`, which will always have this lifetime.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_raw_watcher(&mut self, channel: &'event_loop Channel) -> RawWatcher {
        // SAFETY: `channel` is valid for the necessary lifetime, all other requirements fall under
        // the usual autocxx heuristics.
        RawWatcher(unsafe { self.0.as_mut().MakeWatcher(channel) }.within_box())
    }

    /// Note that the `'event_loop` input lifetime is intentional. The C++ API requires that it is
    /// part of `self.configuration()`, which will always have this lifetime.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_raw_sender(&mut self, channel: &'event_loop Channel) -> RawSender {
        // SAFETY: `channel` is valid for the necessary lifetime, all other requirements fall under
        // the usual autocxx heuristics.
        RawSender(unsafe { self.0.as_mut().MakeSender(channel) }.within_box())
    }

    /// Note that the `'event_loop` input lifetime is intentional. The C++ API requires that it is
    /// part of `self.configuration()`, which will always have this lifetime.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_raw_fetcher(&mut self, channel: &'event_loop Channel) -> RawFetcher {
        // SAFETY: `channel` is valid for the necessary lifetime, all other requirements fall under
        // the usual autocxx heuristics.
        RawFetcher(unsafe { self.0.as_mut().MakeFetcher(channel) }.within_box())
    }

    // TODO(Brian): Expose timers and phased loops. Should we have `sleep`-style methods for those,
    // instead of / in addition to mirroring C++ with separate setup and wait?

    // TODO(Brian): Expose OnRun. That should only be called once, so coalesce and have it return
    // immediately afterwards.
}

// SAFETY: If this outlives the parent EventLoop, the C++ code will LOG(FATAL).
#[repr(transparent)]
pub struct RawWatcher(Pin<Box<ffi::aos::WatcherForRust>>);

/// Provides async blocking access to messages on a channel. This will return every message on the
/// channel, in order.
///
/// Use [`EventLoopRuntime::make_raw_watcher`] to create one of these.
///
/// This is the non-typed API, which is mainly useful for reflection and does not provide safe APIs
/// for actually interpreting messages. You probably want a [`Watcher`] instead.
///
/// This is the same concept as [`futures::stream::Stream`], but can't follow that API for technical
/// reasons.
///
/// # Design
///
/// We can't use [`futures::stream::Stream`] because our `Item` type is `Context<'_>`, which means
/// it's different for each `self` lifetime so we can't write a single type alias for it. We could
/// write an intermediate type with a generic lifetime that implements `Stream` and is returned
/// from a `make_stream` method, but that's what `Stream` is doing in the first place so adding
/// another level doesn't help anything.
///
/// We also drop the extraneous `cx` argument that isn't used by this implementation anyways.
///
/// We also run into some limitations in the borrow checker trying to implement `poll`, I think it's
/// the same one mentioned here:
/// https://blog.rust-lang.org/2022/08/05/nll-by-default.html#looking-forward-what-can-we-expect-for-the-borrow-checker-of-the-future
/// We get around that one by moving the unbounded lifetime from the pointer dereference into the
/// function with the if statement.
impl RawWatcher {
    /// Returns a Future to await the next value. This can be canceled (ie dropped) at will,
    /// without skipping any messages.
    ///
    /// Remember not to call `poll` after it returns `Poll::Ready`, just like any other future. You
    /// will need to call this function again to get the succeeding message.
    ///
    /// # Examples
    ///
    /// The common use case is immediately awaiting the next message:
    /// ```
    /// # async fn await_message(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// println!("received: {:?}", watcher.next().await);
    /// # }
    /// ```
    ///
    /// You can also await the first message from any of a set of channels:
    /// ```
    /// # async fn select(
    /// #     mut watcher1: aos_events_event_loop_runtime::RawWatcher,
    /// #     mut watcher2: aos_events_event_loop_runtime::RawWatcher,
    /// # ) {
    /// futures::select! {
    ///     message1 = watcher1.next() => println!("channel 1: {:?}", message1),
    ///     message2 = watcher2.next() => println!("channel 2: {:?}", message2),
    /// }
    /// # }
    /// ```
    ///
    /// Note that due to the returned object borrowing the `self` reference, the borrow checker will
    /// enforce only having a single of these returned objects at a time. Drop the previous message
    /// before asking for the next one. That means this will not compile:
    /// ```compile_fail
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// let first = watcher.next();
    /// let second = watcher.next();
    /// first.await;
    /// # }
    /// ```
    /// and nor will this:
    /// ```compile_fail
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// let first = watcher.next().await;
    /// watcher.next();
    /// println!("still have: {:?}", first);
    /// # }
    /// ```
    /// but this is fine:
    /// ```
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// let first = watcher.next().await;
    /// println!("have: {:?}", first);
    /// watcher.next();
    /// # }
    /// ```
    pub fn next(&mut self) -> RawWatcherNext {
        RawWatcherNext(Some(self))
    }
}

/// The type returned from [`RawWatcher::next`], see there for details.
pub struct RawWatcherNext<'a>(Option<&'a mut RawWatcher>);

impl<'a> Future for RawWatcherNext<'a> {
    type Output = Context<'a>;
    fn poll(mut self: Pin<&mut Self>, _: &mut std::task::Context) -> Poll<Context<'a>> {
        let inner = self
            .0
            .take()
            .expect("May not call poll after it returns Ready");
        let maybe_context = inner.0.as_mut().PollNext();
        if maybe_context.is_null() {
            // We're not returning a reference into it, so we can safely replace the reference to
            // use again in the future.
            self.0.replace(inner);
            Poll::Pending
        } else {
            // SAFETY: We just checked if it's null. If not, it will be a valid pointer. It will
            // remain a valid pointer for the borrow of the underlying `RawWatcher` (ie `'a`)
            // because we're dropping `inner` (which is that reference), so it will need to be
            // borrowed again which cannot happen before the end of `'a`.
            Poll::Ready(Context(unsafe { &*maybe_context }))
        }
    }
}

impl FusedFuture for RawWatcherNext<'_> {
    fn is_terminated(&self) -> bool {
        self.0.is_none()
    }
}

// SAFETY: If this outlives the parent EventLoop, the C++ code will LOG(FATAL).
#[repr(transparent)]
pub struct RawFetcher(Pin<Box<ffi::aos::FetcherForRust>>);

/// Provides access to messages on a channel, without the ability to wait for a new one. This
/// provides APIs to get the latest message at some time, and to follow along and retrieve each
/// message in order.
///
/// Use [`EventLoopRuntime::make_raw_fetcher`] to create one of these.
///
/// This is the non-typed API, which is mainly useful for reflection and does not provide safe APIs
/// for actually interpreting messages. You probably want a [`Fetcher`] instead.
impl RawFetcher {
    pub fn fetch_next(&mut self) -> bool {
        self.0.as_mut().FetchNext()
    }

    pub fn fetch(&mut self) -> bool {
        self.0.as_mut().Fetch()
    }

    pub fn context(&self) -> Context {
        Context(self.0.context())
    }
}

// SAFETY: If this outlives the parent EventLoop, the C++ code will LOG(FATAL).
#[repr(transparent)]
pub struct RawSender(Pin<Box<ffi::aos::SenderForRust>>);

/// Allows sending messages on a channel.
///
/// This is the non-typed API, which is mainly useful for reflection and does not provide safe APIs
/// for actually creating messages to send. You probably want a [`Sender`] instead.
///
/// Use [`EventLoopRuntime::make_raw_sender`] to create one of these.
impl RawSender {
    fn buffer(&mut self) -> &mut [u8] {
        // SAFETY: This is a valid slice, and `u8` doesn't have any alignment requirements.
        unsafe { slice::from_raw_parts_mut(self.0.as_mut().data(), self.0.as_mut().size()) }
    }

    /// Returns an object which can be used to build a message.
    ///
    /// # Examples
    ///
    /// ```
    /// # use pong_rust_fbs::aos::examples::PongBuilder;
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::RawSender) {
    /// # unsafe {
    /// let mut builder = sender.make_builder();
    /// let pong = PongBuilder::new(builder.fbb()).finish();
    /// builder.send(pong);
    /// # }
    /// # }
    /// ```
    ///
    /// You can bail out of building a message and build another one:
    /// ```
    /// # use pong_rust_fbs::aos::examples::PongBuilder;
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::RawSender) {
    /// # unsafe {
    /// let mut builder1 = sender.make_builder();
    /// builder1.fbb();
    /// let mut builder2 = sender.make_builder();
    /// let pong = PongBuilder::new(builder2.fbb()).finish();
    /// builder2.send(pong);
    /// # }
    /// # }
    /// ```
    /// but you cannot build two messages at the same time with a single builder:
    /// ```compile_fail
    /// # use pong_rust_fbs::aos::examples::PongBuilder;
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::RawSender) {
    /// # unsafe {
    /// let mut builder1 = sender.make_builder();
    /// let mut builder2 = sender.make_builder();
    /// PongBuilder::new(builder2.fbb()).finish();
    /// PongBuilder::new(builder1.fbb()).finish();
    /// # }
    /// # }
    /// ```
    pub fn make_builder(&mut self) -> RawBuilder {
        // TODO(Brian): Actually use the provided buffer instead of just using its
        // size to allocate a separate one.
        //
        // See https://github.com/google/flatbuffers/issues/7385.
        let fbb = flatbuffers::FlatBufferBuilder::with_capacity(self.buffer().len());
        RawBuilder {
            raw_sender: self,
            fbb,
        }
    }
}

#[derive(Clone, Copy, Eq, PartialEq, Debug, Error)]
pub enum SendError {
    #[error("messages have been sent too fast on this channel")]
    MessagesSentTooFast,
    #[error("invalid redzone data, shared memory corruption detected")]
    InvalidRedzone,
}

/// Used for building a message. See [`RawSender::make_builder`] for details.
pub struct RawBuilder<'sender> {
    raw_sender: &'sender mut RawSender,
    fbb: flatbuffers::FlatBufferBuilder<'sender>,
}

impl<'sender> RawBuilder<'sender> {
    pub fn fbb(&mut self) -> &mut flatbuffers::FlatBufferBuilder<'sender> {
        &mut self.fbb
    }

    /// # Safety
    ///
    /// `T` must match the type of the channel of the sender this builder was created from.
    pub unsafe fn send<T>(mut self, root: flatbuffers::WIPOffset<T>) -> Result<(), SendError> {
        self.fbb.finish_minimal(root);
        let data = self.fbb.finished_data();

        use ffi::aos::RawSender_Error as FfiError;
        // SAFETY: This is a valid buffer we're passing.
        match unsafe {
            self.raw_sender
                .0
                .as_mut()
                .CopyAndSend(data.as_ptr(), data.len())
        } {
            FfiError::kOk => Ok(()),
            FfiError::kMessagesSentTooFast => Err(SendError::MessagesSentTooFast),
            FfiError::kInvalidRedzone => Err(SendError::InvalidRedzone),
        }
    }
}

#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct Context<'context>(&'context ffi::aos::Context);

impl fmt::Debug for Context<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // TODO(Brian): Add the realtime timestamps here.
        f.debug_struct("Context")
            .field("monotonic_event_time", &self.monotonic_event_time())
            .field("monotonic_remote_time", &self.monotonic_remote_time())
            .field("queue_index", &self.queue_index())
            .field("remote_queue_index", &self.remote_queue_index())
            .field("size", &self.data().map(|data| data.len()))
            .field("buffer_index", &self.buffer_index())
            .field("source_boot_uuid", &self.source_boot_uuid())
            .finish()
    }
}

// TODO(Brian): Add the realtime timestamps here.
impl<'context> Context<'context> {
    pub fn monotonic_event_time(self) -> MonotonicInstant {
        MonotonicInstant(self.0.monotonic_event_time)
    }

    pub fn monotonic_remote_time(self) -> MonotonicInstant {
        MonotonicInstant(self.0.monotonic_remote_time)
    }

    pub fn queue_index(self) -> u32 {
        self.0.queue_index
    }
    pub fn remote_queue_index(self) -> u32 {
        self.0.remote_queue_index
    }

    pub fn data(self) -> Option<&'context [u8]> {
        if self.0.data.is_null() {
            None
        } else {
            // SAFETY:
            //  * `u8` has no alignment requirements
            //  * It must be a single initialized flatbuffers buffer
            //  * The borrow in `self.0` guarantees it won't be modified for `'context`
            Some(unsafe { slice::from_raw_parts(self.0.data as *const u8, self.0.size) })
        }
    }

    pub fn buffer_index(self) -> i32 {
        self.0.buffer_index
    }

    pub fn source_boot_uuid(self) -> &'context Uuid {
        // SAFETY: `self` has a valid C++ object. C++ guarantees that the return value will be
        // valid until something changes the context, which is `'context`.
        Uuid::from_bytes_ref(&self.0.source_boot_uuid)
    }
}

/// Represents a `aos::monotonic_clock::time_point` in a natural Rust way. This
/// is intended to have the same API as [`std::time::Instant`], any missing
/// functionality can be added if useful.
///
/// TODO(Brian): Do RealtimeInstant too. Use a macro? Integer as a generic
/// parameter to distinguish them? Or just copy/paste?
#[repr(transparent)]
#[derive(Clone, Copy, Eq, PartialEq)]
pub struct MonotonicInstant(i64);

impl MonotonicInstant {
    /// `aos::monotonic_clock::min_time`, commonly used as a sentinel value.
    pub const MIN_TIME: Self = Self(i64::MIN);

    pub fn is_min_time(self) -> bool {
        self == Self::MIN_TIME
    }

    pub fn duration_since_epoch(self) -> Option<Duration> {
        if self.is_min_time() {
            None
        } else {
            Some(Duration::from_nanos(self.0.try_into().expect(
                "monotonic_clock::time_point should always be after the epoch",
            )))
        }
    }
}

impl fmt::Debug for MonotonicInstant {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.duration_since_epoch().fmt(f)
    }
}

mod panic_waker {
    use std::task::{RawWaker, RawWakerVTable, Waker};

    unsafe fn clone_panic_waker(_data: *const ()) -> RawWaker {
        raw_panic_waker()
    }

    unsafe fn noop(_data: *const ()) {}

    unsafe fn wake_panic(_data: *const ()) {
        panic!("Nothing should wake EventLoopRuntime's waker");
    }

    const PANIC_WAKER_VTABLE: RawWakerVTable =
        RawWakerVTable::new(clone_panic_waker, wake_panic, wake_panic, noop);

    fn raw_panic_waker() -> RawWaker {
        RawWaker::new(std::ptr::null(), &PANIC_WAKER_VTABLE)
    }

    pub fn panic_waker() -> Waker {
        // SAFETY: The implementations of the RawWakerVTable functions do what is required of them.
        unsafe { Waker::from_raw(raw_panic_waker()) }
    }
}

use panic_waker::panic_waker;
