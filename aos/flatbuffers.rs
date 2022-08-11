//! Utilities for working with FlatBuffers types.
//!
//! These address similar use cases as the C++ library with the same name, but unlike most of these
//! wrappers the APIs look very different in most cases to be ergnomic in Rust.

use std::marker::PhantomData;

use flatbuffers::{
    FlatBufferBuilder, Follow, FollowWith, ForwardsUOffset, InvalidFlatbuffer, SkipSizePrefix,
    Verifiable, Verifier, WIPOffset,
};

/// Represents the various forms of buffers we can store tables in.
///
/// We need this trait to mark types which won't change the data in a flatbuffer after we verify
/// it. Rust's type system cannot express this constraint directly, so we use an unsafe trait to
/// call attention to what implementing types are promising.
///
/// # Safety
///
/// Implementors of this trait must return a slice with the same contents every time
/// [`AsRef.as_ref`] is called without any intervening mutable access to the object. In other words,
/// types implementing this trait must have exclusive ownership of the underlying storage, and not
/// do anything unusal like returning different pieces of storage each time.
///
/// Note that the slice is not required to have the same address each time [`AsRef.as_ref`] is
/// called, just the same contents. This means types using inline storage can implement this trait
/// without needing pinning.
pub unsafe trait FlatbufferStorage: AsRef<[u8]> {}

// SAFETY: Vec has exclusive ownership of its single storage.
unsafe impl FlatbufferStorage for Vec<u8> {}

// SAFETY: We have a shared reference to the slice and u8 does not have interior mutability, so
// nobody can change the memory as long as the shared reference exists. We're only using the one
// slice too.
unsafe impl FlatbufferStorage for &[u8] {}

// SAFETY: The array is its own storage.
unsafe impl<const N: usize> FlatbufferStorage for [u8; N] {}

/// A type to wrap buffers returned from [`FlatBufferBuilder.collapse`].
///
/// See [`Flatbuffer.finish_minimal_and_collapse`] for an easy way to make use of this.
#[derive(Clone)]
pub struct CollapsedFlatBufferBuilder(pub Vec<u8>, pub usize);

impl AsRef<[u8]> for CollapsedFlatBufferBuilder {
    fn as_ref(&self) -> &[u8] {
        &self.0[self.1..]
    }
}

// SAFETY: The Vec has exclusive ownership of its single storage, and we always access the same
// part of it.
unsafe impl FlatbufferStorage for CollapsedFlatBufferBuilder {}

/// An internal sealed trait to manage the size-prefixed versions vs non-size-prefixed variants.
pub trait PrefixedOrNot: private_size_prefixed::Sealed {
    /// [`FlatBufferBuilder::finish`] or [`FlatBufferBuilder::finish_size_prefixed`].
    fn finish_builder<T>(
        fbb: &mut FlatBufferBuilder,
        root: WIPOffset<T>,
        file_identifier: Option<&str>,
    );

    /// [`flatbuffers::root`] or [`flatbuffers::size_prefixed_root`].
    fn root<'buf, T: 'buf + Follow<'buf> + Verifiable>(
        data: &'buf [u8],
    ) -> Result<T::Inner, InvalidFlatbuffer>;

    /// [`flatbuffers::root_unchecked`] or [`flatbuffers::size_prefixed_root_unchecked`].
    unsafe fn root_unchecked<'buf, T: 'buf + Follow<'buf>>(data: &'buf [u8]) -> T::Inner;

    /// [`ForwardsUOffset::run_verifier`], after skipping the size if present.
    fn run_verifier<T: Verifiable>(
        v: &mut Verifier<'_, '_>,
        pos: usize,
    ) -> Result<(), InvalidFlatbuffer>;
}

/// Marker type to implement [`PrefixedOrNot`] on.
pub struct NotSizePrefixed;

impl PrefixedOrNot for NotSizePrefixed {
    fn finish_builder<T>(
        fbb: &mut FlatBufferBuilder,
        root: WIPOffset<T>,
        file_identifier: Option<&str>,
    ) {
        fbb.finish::<T>(root, file_identifier);
    }

    fn root<'buf, T: 'buf + Follow<'buf> + Verifiable>(
        data: &'buf [u8],
    ) -> Result<T::Inner, InvalidFlatbuffer> {
        flatbuffers::root::<'buf, T>(data)
    }

    unsafe fn root_unchecked<'buf, T: 'buf + Follow<'buf>>(data: &'buf [u8]) -> T::Inner {
        flatbuffers::root_unchecked::<'buf, T>(data)
    }

    fn run_verifier<T: Verifiable>(
        v: &mut Verifier<'_, '_>,
        pos: usize,
    ) -> Result<(), InvalidFlatbuffer> {
        <ForwardsUOffset<T>>::run_verifier(v, pos)
    }
}

/// Marker type to implement [`PrefixedOrNot`] on.
pub struct SizePrefixed;

impl PrefixedOrNot for SizePrefixed {
    fn finish_builder<T>(
        fbb: &mut FlatBufferBuilder,
        root: WIPOffset<T>,
        file_identifier: Option<&str>,
    ) {
        fbb.finish_size_prefixed::<T>(root, file_identifier);
    }

    fn root<'buf, T: 'buf + Follow<'buf> + Verifiable>(
        data: &'buf [u8],
    ) -> Result<T::Inner, InvalidFlatbuffer> {
        flatbuffers::size_prefixed_root::<'buf, T>(data)
    }

    unsafe fn root_unchecked<'buf, T: 'buf + Follow<'buf>>(data: &'buf [u8]) -> T::Inner {
        flatbuffers::size_prefixed_root_unchecked::<'buf, T>(data)
    }

    fn run_verifier<T: Verifiable>(
        v: &mut Verifier<'_, '_>,
        pos: usize,
    ) -> Result<(), InvalidFlatbuffer> {
        <SkipSizePrefix<ForwardsUOffset<T>>>::run_verifier(v, pos)
    }
}

mod private_size_prefixed {
    pub trait Sealed {}

    impl Sealed for super::NotSizePrefixed {}
    impl Sealed for super::SizePrefixed {}
}

/// A flatbuffer along with its backing storage.
///
/// `T` should be a generated flatbuffer table type. Its lifetime argument does not matter, using
/// `'static` is recommended for simplicity. This type uses the [`FollowWith`] trait to access the
/// equivalents of `T` with appropriate lifetimes as needed.
///
/// This is a bit tricky in Rust because we want to avoid internal pointers (requiring the buffer
/// to be pinned before using it would be hard to use). Instead of actually storing the flatbuffer
/// with its pointer to the storage, we (optionally) verify it once and then create the flatbuffer
/// type on demand.
pub trait Flatbuffer<T>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    /// Provides access to the message.
    ///
    /// Note that we can't implement `Deref` because we're not returning a reference. We're
    /// effectively returning a fat pointer which is semantically equivalent to a Rust reference,
    /// but it's not the same thing to the language. Also implementing `Deref` would allow wrapping
    /// this with `Pin`, which doesn't do anything because the actual type being dereferenced is
    /// still `Unpin`, even though other languages commonly store pointers into the flatbuffer.
    fn message<'this>(&'this self) -> <<T as FollowWith<'this>>::Inner as Follow<'this>>::Inner;

    /// Revalidates the data. If this returns `Err`, then something has violated the safety
    /// requirements and code responsible is likely unsound.
    ///
    /// You should probably not be using this. It's intended for debug assertions when working with
    /// cross-language boundaries and otherwise doing tricky things with buffers. Once again, you
    /// probably have unsoundness if you're using this, verify your flatbuffers before wrapping them
    /// in a [`Flatbuffer`], not after.
    fn revalidate<'this>(&'this self) -> Result<(), InvalidFlatbuffer>
    where
        <T as FollowWith<'this>>::Inner: Verifiable;
}

/// A generic implementation of [`Flatbuffer`]. `Storage` is the type used to store the buffer.
/// `MaybePrefixed` controls whether we're using size-prefixed flatbuffers or not.
///
/// Flatbuffers provides a parallel set of APIs for handling size-prefixed buffers. This just means
/// the size is placed at the beginning, so the root table's offset isn't at the very beginning of
/// the buffer.
///
/// [`NonSizePrefixedFlatbuffer`] and [`SizePrefixedFlatbuffer`] provide more convenient names.
pub struct MaybePrefixedFlatbuffer<T, Storage: FlatbufferStorage, MaybePrefixed: PrefixedOrNot> {
    // SAFETY: We have an invariant that this is always a valid flatbuffer.
    //
    // All of our constructors verify that, and we never hand out mutable references so the
    // requirements for implementors of [`FlatbufferStorage`] ensure it stays valid.
    storage: Storage,

    /// Pretend we store one of these. The compiler needs this information to clarify that two
    /// instances with different `T` and/or `MaybePrefixed` are not the same type.
    _marker: PhantomData<(T, MaybePrefixed)>,
}

impl<T, Storage: FlatbufferStorage, MaybePrefixed: PrefixedOrNot> Flatbuffer<T>
    for MaybePrefixedFlatbuffer<T, Storage, MaybePrefixed>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    fn message<'this>(&'this self) -> <<T as FollowWith<'this>>::Inner as Follow<'this>>::Inner {
        // SAFETY: This being a valid flatbuffer is an invariant.
        unsafe {
            MaybePrefixed::root_unchecked::<<T as FollowWith<'this>>::Inner>(self.storage.as_ref())
        }
    }

    fn revalidate<'this>(&'this self) -> Result<(), InvalidFlatbuffer>
    where
        <T as FollowWith<'this>>::Inner: Verifiable,
    {
        MaybePrefixed::root::<<T as FollowWith<'this>>::Inner>(self.storage.as_ref())?;
        Ok(())
    }
}

impl<T, Storage: FlatbufferStorage, MaybePrefixed: PrefixedOrNot>
    MaybePrefixedFlatbuffer<T, Storage, MaybePrefixed>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a> + Verifiable,
{
    /// Verifies the flatbuffer and then creates an instance.
    ///
    /// Note that `storage` must only refer to the flatbuffer. `FlatBufferBuilder::collapse`
    /// returns a buffer with unused space at the beginning, for example, you will have to either
    /// use a slice here or remove the unused space first.
    pub fn new(storage: Storage) -> Result<Self, InvalidFlatbuffer> {
        MaybePrefixed::root::<<T as FollowWith<'_>>::Inner>(storage.as_ref())?;
        Ok(Self {
            storage,
            _marker: PhantomData,
        })
    }
}

impl<T, Storage: FlatbufferStorage, MaybePrefixed: PrefixedOrNot>
    MaybePrefixedFlatbuffer<T, Storage, MaybePrefixed>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    /// Wraps a flatbuffer without verifying it.
    ///
    /// Note that `storage` must only refer to the flatbuffer. `FlatBufferBuilder::collapse`
    /// returns a buffer with unused space at the beginning, for example, so you can not pass that
    /// here directly. See [`CollapsedFlatBufferBuilder`] for that particular case.
    ///
    /// # Safety
    ///
    /// `storage` must refer to a valid flatbuffer.
    pub unsafe fn new_unchecked(storage: Storage) -> Self {
        Self {
            storage,
            _marker: PhantomData,
        }
    }

    /// Returns a reference to the underlying storage.
    pub fn storage_ref(&self) -> &[u8] {
        self.storage.as_ref()
    }

    /// Converts back into the underlying storage.
    pub fn into_storage(self) -> Storage {
        self.storage
    }
}

impl<T, Storage: FlatbufferStorage + Clone, MaybePrefixed: PrefixedOrNot> Clone
    for MaybePrefixedFlatbuffer<T, Storage, MaybePrefixed>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    fn clone(&self) -> Self {
        // SAFETY: We already know it's valid because it was used by `self`.
        unsafe { Self::new_unchecked(self.storage.clone()) }
    }
}

impl<T, MaybePrefixed: PrefixedOrNot>
    MaybePrefixedFlatbuffer<T, CollapsedFlatBufferBuilder, MaybePrefixed>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    /// Finishes a [`FlatBufferBuilder`] and then grabs its buffer. This does not require
    /// verification to be safe.
    ///
    /// We have to encapsulate the `finish` call to enforce matching types.
    pub fn finish_minimal_and_collapse(mut fbb: FlatBufferBuilder<'_>, root: WIPOffset<T>) -> Self {
        MaybePrefixed::finish_builder(&mut fbb, root, None);
        let (vec, offset) = fbb.collapse();
        // SAFETY: We just finished the builder with an offset of the correct type. The builder
        // maintained the invariant that it was building a valid flatbuffer, which we are directly
        // using.
        unsafe { Self::new_unchecked(CollapsedFlatBufferBuilder(vec, offset)) }
    }
}

impl<T, F: Flatbuffer<T> + ?Sized> Flatbuffer<T> for Box<F>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    fn message<'this>(&'this self) -> <<T as FollowWith<'this>>::Inner as Follow<'this>>::Inner {
        self.as_ref().message()
    }

    fn revalidate<'this>(&'this self) -> Result<(), InvalidFlatbuffer>
    where
        <T as FollowWith<'this>>::Inner: Verifiable,
    {
        self.as_ref().revalidate()
    }
}

pub type NonSizePrefixedFlatbuffer<T, Storage> =
    MaybePrefixedFlatbuffer<T, Storage, NotSizePrefixed>;
pub type SizePrefixedFlatbuffer<T, Storage> = MaybePrefixedFlatbuffer<T, Storage, SizePrefixed>;

/// This is the alias for a generic "some kind of buffer that contains a flatbuffer". Use this if
/// you want your code to work with any type of buffer without being generic.
///
/// `'buf` is the lifetime of the underlying storage.
///
/// If you only want to support flatbuffers that own their underlying buffer, and don't want to
/// even make your functions lifetime-generic, use `DynFlatbuffer<'static, T>`. However, note
/// that being lifetime-generic has no cost in code size or vtable lookups, and if your code isn't
/// storing references to the flatbuffer anywhere it's a strict increase in flexibility.
pub type DynFlatbuffer<'buf, T> = Box<dyn Flatbuffer<T> + 'buf>;

#[cfg(test)]
mod tests {
    use super::*;

    use aos_json_to_flatbuffer_fbs::aos::testing::{Location, LocationBuilder};

    /// Tests various methods that take flatbuffers as arguments.
    #[test]
    fn test_take() {
        fn take_box_dyn_ref(flatbuffer: &DynFlatbuffer<'_, Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_box_dyn_ref_static(flatbuffer: &DynFlatbuffer<'static, Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_bare_dyn_ref(flatbuffer: &dyn Flatbuffer<Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_impl_ref(flatbuffer: &impl Flatbuffer<Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_impl_ref_static(flatbuffer: &(impl Flatbuffer<Location<'static>> + 'static)) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_box_dyn(flatbuffer: DynFlatbuffer<'_, Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_box_dyn_static(flatbuffer: DynFlatbuffer<'static, Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_impl(flatbuffer: impl Flatbuffer<Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_impl_static(flatbuffer: impl Flatbuffer<Location<'static>> + 'static) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_nondyn_ref(
            flatbuffer: &NonSizePrefixedFlatbuffer<Location<'static>, CollapsedFlatBufferBuilder>,
        ) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        fn take_nondyn(
            flatbuffer: NonSizePrefixedFlatbuffer<Location<'static>, CollapsedFlatBufferBuilder>,
        ) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        let flatbuffer = {
            let mut fbb = FlatBufferBuilder::new();
            let xyz = fbb.create_string("xyz");
            let mut location = LocationBuilder::new(&mut fbb);
            location.add_name(xyz);
            location.add_frequency(971);
            let location = location.finish();
            NonSizePrefixedFlatbuffer::finish_minimal_and_collapse(fbb, location)
        };
        let make_dyn_flatbuffer = || Box::new(flatbuffer.clone());
        let dyn_flatbuffer: DynFlatbuffer<_> = make_dyn_flatbuffer();

        take_box_dyn_ref(&dyn_flatbuffer);
        take_box_dyn_ref_static(&dyn_flatbuffer);
        take_bare_dyn_ref(&dyn_flatbuffer);
        take_bare_dyn_ref(&flatbuffer);
        take_impl_ref(&dyn_flatbuffer);
        take_impl_ref(&flatbuffer);
        take_impl_ref_static(&dyn_flatbuffer);
        take_impl_ref_static(&flatbuffer);
        take_box_dyn(make_dyn_flatbuffer());
        take_box_dyn_static(make_dyn_flatbuffer());
        take_impl(make_dyn_flatbuffer());
        take_impl(flatbuffer.clone());
        take_impl_static(make_dyn_flatbuffer());
        take_impl_static(flatbuffer.clone());
        take_nondyn_ref(&flatbuffer);
        take_nondyn(flatbuffer.clone());
    }

    /// Tests creating a DynFlatbuffer from various types.
    #[test]
    fn test_make_dyn() {
        fn take_ref(flatbuffer: &impl Flatbuffer<Location<'static>>) {
            assert_eq!(Some("xyz"), flatbuffer.message().name());
            assert_eq!(971, flatbuffer.message().frequency());
        }

        let flatbuffer = {
            let mut fbb = FlatBufferBuilder::new();
            let xyz = fbb.create_string("xyz");
            let mut location = LocationBuilder::new(&mut fbb);
            location.add_name(xyz);
            location.add_frequency(971);
            let location = location.finish();
            NonSizePrefixedFlatbuffer::finish_minimal_and_collapse(fbb, location)
        };
        take_ref(&flatbuffer);
        let slice_flatbuffer =
            NonSizePrefixedFlatbuffer::<Location<'static>, &[u8]>::new(flatbuffer.storage_ref())
                .unwrap();
        take_ref(&slice_flatbuffer);

        {
            let dyn_flatbuffer: DynFlatbuffer<_> = Box::new(flatbuffer.clone());
            take_ref(&dyn_flatbuffer);
        }
        {
            let dyn_flatbuffer: DynFlatbuffer<_> = Box::new(slice_flatbuffer.clone());
            take_ref(&dyn_flatbuffer);
        }
    }

    macro_rules! test_variant {
        ($maybe_prefixed:ident, $modname:ident, $finish:ident) => {
            mod $modname {
                use super::*;

                use aos_json_to_flatbuffer_fbs::aos::testing::{Location, LocationBuilder};

                #[test]
                fn test_basic() {
                    let fbb = {
                        let mut fbb = FlatBufferBuilder::new();
                        let xyz = fbb.create_string("xyz");
                        let mut location = LocationBuilder::new(&mut fbb);
                        location.add_name(xyz);
                        location.add_frequency(971);
                        let location = location.finish();
                        fbb.$finish(location, None);
                        fbb
                    };

                    {
                        let flatbuffer = $maybe_prefixed::<Location, _>::new(
                            fbb.finished_data().iter().copied().collect::<Vec<u8>>(),
                        )
                        .unwrap();
                        assert_eq!(Some("xyz"), flatbuffer.message().name());
                        assert_eq!(971, flatbuffer.message().frequency());
                        flatbuffer.revalidate().unwrap();
                    }

                    {
                        let flatbuffer =
                            $maybe_prefixed::<Location, _>::new(fbb.finished_data()).unwrap();
                        assert_eq!(Some("xyz"), flatbuffer.message().name());
                        assert_eq!(971, flatbuffer.message().frequency());
                        flatbuffer.revalidate().unwrap();
                    }

                    {
                        let mut array_data = [0; 64];
                        let finished_data = fbb.finished_data();
                        array_data[..finished_data.len()].copy_from_slice(finished_data);
                        let flatbuffer = $maybe_prefixed::<Location, _>::new(array_data).unwrap();
                        assert_eq!(Some("xyz"), flatbuffer.message().name());
                        assert_eq!(971, flatbuffer.message().frequency());
                        flatbuffer.revalidate().unwrap();
                    }
                }

                #[test]
                fn test_collapse() {
                    let mut fbb = FlatBufferBuilder::new();
                    let xyz = fbb.create_string("xyz");
                    let mut location = LocationBuilder::new(&mut fbb);
                    location.add_name(xyz);
                    location.add_frequency(971);
                    let location = location.finish();
                    let flatbuffer = $maybe_prefixed::finish_minimal_and_collapse(fbb, location);

                    assert_eq!(Some("xyz"), flatbuffer.message().name());
                    assert_eq!(971, flatbuffer.message().frequency());
                    flatbuffer.revalidate().unwrap();
                }

                #[test]
                fn test_verification_failed() {
                    // Something arbitrary that is not a valid flatbuffer. The test will fail if this does end
                    // up being valid.
                    let data = [5u8; 1];
                    assert!($maybe_prefixed::<Location, _>::new(data).is_err());
                }

                mod bad {
                    use super::*;

                    use std::cell::Cell;

                    struct Bad {
                        a: Vec<u8>,
                        b: [u8; 1],
                        first: Cell<bool>,
                    }

                    impl AsRef<[u8]> for Bad {
                        fn as_ref(&self) -> &[u8] {
                            if self.first.replace(false) {
                                self.a.as_ref()
                            } else {
                                self.b.as_ref()
                            }
                        }
                    }

                    unsafe impl FlatbufferStorage for Bad {}

                    /// A demonstration of how using an incorrect `FlatbufferStorage` implementation can go
                    /// wrong. Note that this test is careful to not actually do anything unsound by never
                    /// using the invalid flatbuffer, it only verifies validation fails on it.
                    #[test]
                    fn test_bad() {
                        let fbb = {
                            let mut fbb = FlatBufferBuilder::new();
                            let mut location = LocationBuilder::new(&mut fbb);
                            location.add_frequency(971);
                            let location = location.finish();
                            fbb.$finish(location, None);
                            fbb
                        };

                        let data = Bad {
                            a: fbb.finished_data().iter().copied().collect(),
                            // Something arbitrary that is not a valid flatbuffer. The test will fail if this
                            // does end up being valid.
                            b: [5; 1],
                            first: Cell::new(true),
                        };

                        let flatbuffer = $maybe_prefixed::<Location, _>::new(data).unwrap();
                        assert!(flatbuffer.revalidate().is_err());
                    }
                }
            }
        };
    }

    test_variant!(NonSizePrefixedFlatbuffer, non_prefixed, finish);
    test_variant!(SizePrefixedFlatbuffer, prefixed, finish_size_prefixed);
}
