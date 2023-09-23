# FlatBuffers

This document covers the "static flatbuffers API".

This API is a custom C++ API for serializing flatbuffers developed for AOS. The
serialized flatbuffers are fully compatible with the existing flatbuffers
specification.

## Design

The overall goal of the static flatbuffers API is to make it so that a user can
construct flatbuffers against a fixed-size memory buffer while being able to
readily mutate any part of the flatbuffer object at any point during
construction (rather than being forced to construct things from the bottom up).

In particular:

* The API should be able to both construct flatbuffers against fixed-size memory
  buffers (for use in realtime code) as well as against variable-size buffers
  (for easy offline flatbuffer manipulation).
* We want to be able to select vector sizes at runtime (including in realtime
  code) so that we can support using e.g. the same camera message schema (which
  would generally contain a byte array of data) for multiple different
  resolutions of an image.
* The API should require minimal modifications to existing .fbs files (in fact,
  it is usable without any modifications).
* We want to be able to provide an option for deriving strict upper bounds for
  AOS channel `max_size`'s (this is not currently fully implemented). This does
  require specifying maximum vector sizes (and adhering to them) in the message
  schemas.
* There should be low performance impacts when using the API normally (as
  compared to trying to use the `FlatBufferBuilder` API).
* The API should be difficult to accidentally use incorrectly (e.g., the
  existing flatbuffers API requires that you not build multiple tables at once,
  and enforces this with runtime debug assertions; this is both hard to develop
  and prone to memory corruption in situations where someone never runs debug
  builds of their code).

In order to accomplish this, we provide a codegen'd interface in place of the
regular flatbuffer API. It provides the following objects/interfaces to work with:

* For each table, a codegen'd class which inherits from the `aos::fbs::Table`
  object. This provides accessors to let you set, get, clear, and mutate table
  members at any time. The table objects take in an aligned `std::span` into
  which they construct the flatbuffer. This class will be named `FooStatic`
  for a given flatbuffer type `Foo`.
* For flatbuffer vector/strings, `Vector` and `String` objects are provided
  (the `Vector` object will generally be created by calling `add_*` on the
  appropriate member of a table). These generally operate similarly in concept
  to the table objects, allowing you to add/remove/modify elements at will.
  `Vector`s and `String`s can have a nominal maximum length specified in order
  to have the memory for those elements statically allocated, while also having
  the ability to dynamically increase the size of the vectors.
* In order to allow the construction of a flatbuffer table, a
  templated `aos::fbs::Builder` object is provided which can
  take an allocator and then provide the relevant table class to the user.
* We provide an `Allocator` class and various implementations (e.g., a
  `VectorAllocator` backed by an `std::vector`) for managing the memory into
  which the `Builder` will serialize the flatbuffer.
* A new `MakeStaticBuilder` method is provided on the `aos::Sender` class which
  constructs an `aos::fbs::Builder` to allow you to construct a message to be
  sent on an AOS channel.
* Existing `flatbuffer_cc_library` bazel targets get turned into `static_flatbuffer`
  targets of the same name. Libraries do not need to change how they depend on
  the bazel target, although using the new API in your code will require
  importing a different header and using a different class than before.

### Alignment

Significant effort must be made to ensure that all objects are correctly
aligned. This includes the `force_align` attribute which can be added to vectors
to, e.g., allow you to over-align byte vectors that may need to be used to
store aligned data. The current alignment approach is relatively conservative,
which may result in excessive padding (all padding in the serialized flatbuffers
should get cleared to zero, hopefully allowing compression algorithms to handle
the extra bytes reasonably efficiently).

As a user, you should never need to do anything else to get correct alignment.
Further discussion in this section is mostly relevant for those
modifying/reviewing the internals.

Internally, every `Vector` and `Table` type tracks its required alignment using
a `kAlign` constant. This constant is set as the maximum alignment of any
members of the object (this will always be a minimum of `4` because every
flatbuffer table includes a 4-byte vtable offset and every flatbuffer vector
includes a 4-byte length). The buffers provided to the constructors of these
objects must be aligned, and the `kSize` constant that the objects provide will
always be a multiple of the alignment. Additional discussion of the detailed
layout of memory inside of the `Vector` and `Table` types can be found in the
comments on the respective class declarations.

In order to handle alignment correctly in our `Builder` and `Allocator` classes,
we end up forcing the `Builder` to be able to accept semi-arbitrarily aligned
buffers in order to ease the `Allocator` implementation (e.g., the
`VectorAllocator` uses a `std::vector` internally which does not necessarily
align its memory). The `Builder` then adds padding as needed and passes an
appropriately aligned buffer down to the `Table` class.

## Basic API Examples

This example will walk through what the API for the following set of tables
looks like (see `//aos/flatbuffers:test.fbs`:

```cpp
// Note: in the actual sample code, these are two separate files where one is
// included in the other.
namespace aos.fbs.testing.included;
enum TestEnum : ubyte {
  A = 0,
  B = 1,
}

table IncludedTable {
 foo:TestEnum (id: 0);
}

namespace aos.fbs.testing;

struct SubStruct {
  x:double;
  y:double;
}

table SubTable {
 foo:short (id: 0);
 bar:short (id: 1, deprecated);
 baz:float (id: 2);
}

attribute "static_length";
attribute "static_vector_string_length";

table TestTable {
  scalar:int (id: 0);
  vector_of_scalars:[int] (id: 1, static_length: 3);
  string:string (id: 2, static_length: 20);
  vector_of_strings:[string] (id: 3, static_length: 3, static_vector_string_length: 10);
  substruct:SubStruct (id: 4);
  subtable:SubTable (id: 5);
  // The force-aligned vector is deliberately put in the middle of the table
  // both by ID and alphabetically (both of these can affect the order in which
  // certain things are evaluated, and during development there were some issues
  // with this).
  vector_aligned:[int] (id: 6, force_align: 64, static_length: 3);
  vector_of_structs:[SubStruct] (id: 7, static_length: 3);
  vector_of_tables:[SubTable] (id: 8, static_length: 3);
  included_table:aos.fbs.testing.included.IncludedTable (id: 9);
  unspecified_length_vector:[ubyte] (id: 10);
  unspecified_length_string:string (id: 11);
  unspecified_length_vector_of_strings:[string] (id: 12);
}

root_type TestTable;
```

All created types have an `AsFlatbuffer()` method which allows you to access the
type using the regular generated flatbuffer API and a `FromFlatbuffer()` method
which attempts to copy the specified flatbuffer into the current object.

### Sample Usage

The below example constructs a table of the above example `TestTable`:

```cpp
aos::FixedAllocator allocator(TestTableStatic::kUnalignedBufferSize);
Builder<TestTableStatic> builder(&allocator);
TestTableStatic *object = builder.get();
object->set_scalar(123);
{
  auto vector = object->add_vector_of_scalars();
  CHECK(vector->emplace_back(4));
  CHECK(vector->emplace_back(5));
}
{
  auto string = object->add_string();
  string->SetString("Hello, World!");
}
{
  auto vector_of_strings = object->add_vector_of_strings();
  auto sub_string = CHECK_NOTNULL(vector_of_strings->emplace_back());
  CHECK(sub_string->emplace_back('D'));
}
{
  object->set_substruct({971, 254});
}
{
  auto subtable = object->add_subtable();
  subtable->set_foo(1234);
}
{
  auto vector = object->add_vector_of_structs();
  CHECK(vector->emplace_back({48, 67}));
  CHECK(vector->emplace_back({118, 148}));
  CHECK(vector->emplace_back({971, 973}));
  // Max vector size is three; this should fail.
  CHECK(!vector->emplace_back({1114, 2056}));
}
{
  auto vector = object->add_vector_of_tables();
  auto subobject = vector->emplace_back();
  subobject->set_foo(222);
}
{
  auto subtable = object->add_included_table();
  subtable->set_foo(included::TestEnum::B);
}
LOG(INFO) <<
    aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                          {.multi_line = true});
```

This will then output:

```json
{
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971.0,
  "y": 254.0
 },
 "subtable": {
  "foo": 1234
 },
 "vector_of_structs": [
  {
   "x": 48.0,
   "y": 67.0
  },
  {
   "x": 118.0,
   "y": 148.0
  },
  {
   "x": 971.0,
   "y": 973.0
  }
 ],
 "vector_of_tables": [
  {
   "foo": 222
  }
 ],
 "included_table": {
  "foo": "B"
 }
}
```

### Converting `Populate*()` methods

With existing flatbuffer code it is common to have
`flatbuffers::Offset<> Populate*(FlatBufferBuilder*)` methods for populating
subtables of a message. When converting these to the static API, you can
keep the same patterns (although you have more flexibility available if you
choose), but modify the `Populate` call slightly:

```cpp
namespace {
flatbuffers::Offset<SubTable> PopulateOld(flatbuffers::FlatBufferBuilder *fbb) {
  SubTable::Builder builder(*fbb);
  builder.add_foo(1234);
  return builder.Finish();
}
void PopulateStatic(SubTableStatic *subtable) { subtable->set_foo(1234); }
}  // namespace
TEST_F(StaticFlatbuffersTest, PopulateMethodConversionExample) {
  // Using a FlatBufferBuilder:
  flatbuffers::FlatBufferBuilder fbb;
  // Note: the PopulateOld() *must* be called prior to creating the builder.
  const flatbuffers::Offset<SubTable> subtable_offset = PopulateOld(&fbb);
  TestTable::Builder testtable_builder(fbb);
  testtable_builder.add_subtable(subtable_offset);
  fbb.Finish(testtable_builder.Finish());
  aos::FlatbufferDetachedBuffer<TestTable> fbb_finished = fbb.Release();

  // Using the static flatbuffer API.
  aos::fbs::VectorAllocator allocator;
  Builder<TestTableStatic> static_builder(&allocator);
  PopulateStatic(CHECK_NOTNULL(static_builder.get()->add_subtable()));

  // And confirm that they both contain the expected flatbuffer:
  const std::string expected = R"json({ "subtable": { "foo": 1234 }})json";
  EXPECT_EQ(expected, aos::FlatbufferToJson(fbb_finished));
  EXPECT_EQ(expected, aos::FlatbufferToJson(static_builder.AsFlatbufferSpan()));
}
```

### Scalar Fields

Scalar fields have an API which is reasonably close to that of the base
flatbuffer builder API. Because space for the scalar fields (as with everything)
is pre-allocated, these accessors may be called at any time.

For an `int` field named `scalar`, we will have the following methods. Note that
prior to any `set_*` method being called, the value will not be populated and so
`has_*` methods will return false and accessors will return `nullopt/nullptr`:

```cpp
// Populates the value and sets it to the requested value. Calling set_scalar()
// will cause has_scalar() to return true.
void set_scalar(const int32_t &value);

// Returns the value of scalar, if populated. Otherwise, returns nullopt.
std::optional<int32_t> scalar() const;

// Returns a pointer to the scalar, if populated. Otherwise, returns nullptr.
// Note that because of the nature of this API we _could_ support always
// returning a valid pointer, but then it would be relatively easy for a user
// to modify the value of a field without ever causing it to become "populated."
int32_t *mutable_scalar();

// Clears the field. Does not invalidate pointers returned by
// `mutable_scalar()`, although it will set the value of the field to zero.
void clear_scalar();

// Returns true if the scalar field is populated.
bool has_scalar() const;
```

### Enum fields

Enum fields operate identically to scalar fields, except that the type in
question is the flatbuffer enum type rather than a C++ scalar of some sort.

### Struct fields

Struct fields operate identically to scalar fields, except that the type in
question is the flatbuffer C-struct type rather than a scalar.

*Note*: This is different than how the raw flatbuffer API handles structs.
Regular flatbuffers actually pass around pointers to the structs rather than
references.

### Table fields

For fields of a table which are themselves tables, the accessors will return a
pointer to an object that can be used to access/populate the subtable in
question. The accessors are generally similar to those used by the scalar
fields.

The accessors that will be generated for a field named `subtable` of type
`SubTable` are below:

```cpp
// Creates a SubTable at the subtable member.
// Will die if the field is already populated (this aspect of the API is
// subject to change if we discover that people like to be able to call
// add_* multiple times).
aos::fbs::testing::SubTableStatic *add_subtable();

// The following will return pointers to the subtable member, or nullptr
// if it is not populated.
const aos::fbs::testing::SubTableStatic *subtable() const:
aos::fbs::testing::SubTableStatic *mutable_subtable();

// Depopulates the subtable member.
void clear_subtable();

// Returns true if the subtable has been populated. This does not
// mean that there is necessarily anything interesting *in* the table,
// just that it exists and can be modified.
bool has_subtable() const;
```

### Vectors

A vector may contain any other type, except for other vectors (with the
exception of strings---vectors of strings are permitted). The APIs for
inline types versus not-inline types (name improvements are welcome...
maybe "object"?) are slightly different because of differences in how
the underlying serialization works.

As already mentioned, each vector will have a "static" size, which is specified
in the flatbuffer schema by the `static_length` attribute (in order to use
this attribute you must have an `attribute "static_length";` line
somewhere in your `.fbs` file). This represents the number of elements that will
have space pre-allocated in the vector. Changing this number does not cause any issues
with backwards compatibility because the underlying flatbuffer representation
permits arbitrary (up to 2^32) length vectors. This is necessary for
choosing how much space to allocate when constructing the flatbuffer.
The maximum size of a vector may be accessed at runtime using the `capacity()`
accessor on the `aos::fbs::Vector` type.

*Note*: You may not use dynamically sized vectors of strings or tables in
realtime code, as allocating space for each additional string/table member
requires overhead which cannot be conveniently allocated anywhere except the
heap. The primary use-case for dynamically sized vectors in realtime code
is for vectors of scalars; if this changes, we can try to add options to
support this. Dynamically sized vectors of tables/strings are supported in
non-realtime code.

If you wish to increase the alignment of a vector beyond the base alignment, you
can use the `force_align` attribute , as seen below:

```
vector_aligned:[int] (id: 6, force_align: 64, static_length: 3);
```

If you do this, the first element of the vector will be aligned to the requested
alignment.

The `aos::fbs::Vector` API is designed to mirror the `std::vector` API, with
some changes to accommodate better error-handling. Common accessors:

* `capacity()`: Maximum number of elements that this vector can accommodate.
* `size()`: Current number of elements populated in this vector.
* `T *emplace_back()`: Adds a not-inline (string or table) type to the vector and returns
  the added object. If there is no more space, returns `nullptr` (call
  `reserve()` to attempt to allocate more space).
* `bool emplace_back(T)`: Adds an inline (scalar, enum, or struct) type to the vector and
  returns true on success. Returns false if there is no more space in the
  vector (call `reserve()` to attempt to allocate more space).
* `AsFlatbufferVector()`, `AsMutableFlatbufferVector()`: Returns a
  `flatbuffer::Vector` of the appropriate type pointing to the vector
  that we are constructing.
* `T &at(size_t index)`/`T& operator[](size_t index)`: Returns the
  object at the requested index. Dies if `index >= size()` (unlike
  `std::vector`, `operator[]` does do bounds checking. Use `unsafe_at()` if you
  want to avoid the performance overhead of bounds checking).
* `resize_inline(size_t size, SetZero set_zero)`/`resize(size_t size)`:
  Resizes the vector to the requested size (dies if the vector cannot
  accommodate the requested size). For inline types,
  you may optionally leave any newly inserted elements uninitialized.
  For not-inline types, will default construct new elements.
* `T* data()`: Returns a pointer to the first element of the vector. Only valid
  for inline data types.
* `bool reserve(size_t new_length)`: Used to dynamically change the amount of
  space allocated for the vector; returns false on failure (e.g., if you are in
  a fixed-size allocator that does not support increasing the size past a
  certain point).
* `bool FromFlatbuffer(const flatbuffers::Vector<>*)`: Attempts to copy an
  existing vector into this `Vector`. This may attempt to call `reserve()`
  if the new vector is longer than `capacity()`. If the copy fails for
  any reason, returns `false`.

#### Managing Resizing of Vectors

When dealing with sizes of vectors, there are two separate "lengths" that are
relevant at any given time:

1. The `capacity`/allocated length of the vector. This is the length for which
   there is currently space allocated in the flatbuffer array and in the
   `Vector` object itself. Upon initialization, this will be equal to the
   `static_length` for the vector. This can only be changed by calling
   `reserve()` or indirectly in `FromFlatbuffer()` (which calls `reserve()`).
2. The `size`/current actual length of the vector. This is the number of
   elements that are currently actually populated in the vector. The current
   `size` of the vector cannot exceed the `capacity`. This will be modified by
   calls to `emplace_back()`/`Resize*()` (and indirectly by `FromFlatbuffer()`).

Because `emplace_back()` and `Resize*()` do not call `reserve()` themselves, they will
return `false` if the capacity of the vector does not currently allow for the
element to be added; when `emplace_back()` returns false, you may call `reserve()` to
attempt to allocate the requisite space; if the allocation itself fails (e.g.,
if you are allocating against a fixed size buffer and do not have sufficient
space for the requested allocation). This means that the user will be forced to
explicitly request changes to the memory layout and allocation of the flatbuffer
rather than being able to hide it inside of calls to `emplace_back()` or the such.

### Strings

Strings are a special case of vectors. The key differences are:

* They are always null-terminated (this is enforced by the API
  itself).
* Because they are null-terminated, the actual vector length will
  be `static_length + 1`. Users should not attempt to
  access this extra character.
* The vector elements are of type `char`.
* Strings are supposed to be UTF-8. This C++ API does not enforce
  this constraint, but if you want a vector of bytes, then use
  a vector of bytes (`[ubyte]`) instead.
* For the special-case where you have a vector of strings, you may specify
  the static length of the strings inside of the vector using the
  `static_vector_string_length` attribute.


## Use With EventLoop Senders

In order to use the static API with `aos::Sender`s you need to:

1. Change the `aos::Sender` to be templated on the `MessageStatic` instead of
   the `Message` type.
2. Use the `MakeStaticBuilder` method instead of the `MakeBuilder` method.
3. Alter the actual flatbuffer API calls to use the new object.
4. In the `Send()` call, remove the `builder.Finish()` as it is no longer necessary.

### Simple Conversion Example

This is an extremely simple example of converting a sender to the new API, taken
from the `event_loop_param_test.cc`:

First, the bazel targets must be updated to generate the new code:

```python
# This load must be added so that the static_flatbuffer rule is available.
load("@org_frc971//aos/flatbuffers:generate.bzl", "static_flatbuffer")

# Remove the prior `flatbuffer_cc_library` and replace it with something like
# this.
# This target had no dependencies, but any deps will also need to be
# upgraded to static_flatbuffer rules.
static_flatbuffer(
    name = "test_message_fbs",
    src = "test_message.fbs",
)
```

Before:
```cpp
  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");

  loop->OnRun([&]() {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    msg.CheckOk(msg.Send(builder.Finish()));
  });
```

After:
```cpp
  aos::Sender<TestMessageStatic> sender =
      loop1->MakeSender<TestMessageStatic>("/test");

  loop->OnRun([&]() {
    aos::Sender<TestMessageStatic>::StaticBuilder msg =
        sender.MakeStaticBuilder();
    msg.get()->set_value(200);
    msg.CheckOk(msg.Send());
  });
```

## Future Improvements

### Suggested API Additions/Improvements

* A  `add_or_get_subtable` generated method that avoids the need for the user to
  check `has_subtable()` before calling `add_subtable()`.
* `operator->()` in places to reduce syntactic overhead.
* Make naming of `StaticVector` methods more consistent with `std::vector`.
