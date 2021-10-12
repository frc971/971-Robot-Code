#include "aos/flatbuffer_merge.h"

#include <cstdio>

#include "aos/flatbuffer_utils.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/minireflect.h"

namespace aos {

namespace {

// Simple structure to hold both field_offsets and elements.
struct OffsetAndFieldOffset {
  OffsetAndFieldOffset(flatbuffers::voffset_t new_field_offset,
                       flatbuffers::Offset<flatbuffers::String> new_element)
      : field_offset(new_field_offset), element(new_element) {}
  OffsetAndFieldOffset(flatbuffers::voffset_t new_field_offset,
                       flatbuffers::Offset<flatbuffers::Table> new_element)
      : field_offset(new_field_offset), element(new_element.o) {}

  flatbuffers::voffset_t field_offset;
  flatbuffers::Offset<flatbuffers::String> element;
};

// Merges a single element to a builder for the provided field.
// One or both of t1 and t2 must be non-null.  If one is null, this method
// copies instead of merging.
template <typename T>
void MergeElement(flatbuffers::voffset_t field_offset,
                  const flatbuffers::Table *t1, const flatbuffers::Table *t2,
                  flatbuffers::FlatBufferBuilder *fbb) {
  const uint8_t *val1 =
      t1 != nullptr ? t1->GetAddressOf(field_offset) : nullptr;
  const uint8_t *val2 =
      t2 != nullptr ? t2->GetAddressOf(field_offset) : nullptr;
  const bool t1_has = val1 != nullptr;
  const bool t2_has = val2 != nullptr;

  if (t2_has) {
    fbb->AddElement<T>(field_offset, flatbuffers::ReadScalar<T>(val2));
  } else if (t1_has) {
    fbb->AddElement<T>(field_offset, flatbuffers::ReadScalar<T>(val1));
  }
}

// Merges a single string to a builder for the provided field.
// One or both of t1 and t2 must be non-null.  If one is null, this method
// copies instead of merging.
void MergeString(flatbuffers::voffset_t field_offset,
                 const flatbuffers::Table *t1, const flatbuffers::Table *t2,
                 flatbuffers::FlatBufferBuilder *fbb,
                 ::std::vector<OffsetAndFieldOffset> *elements) {
  const uint8_t *val1 =
      t1 != nullptr ? t1->GetAddressOf(field_offset) : nullptr;
  const uint8_t *val2 =
      t2 != nullptr ? t2->GetAddressOf(field_offset) : nullptr;
  const bool t1_has = val1 != nullptr;
  const bool t2_has = val2 != nullptr;

  if (t2_has) {
    val2 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val2);
    const flatbuffers::String *string2 =
        reinterpret_cast<const flatbuffers::String *>(val2);
    elements->emplace_back(field_offset,
                           fbb->CreateString(string2->data(), string2->size()));
  } else if (t1_has) {
    val1 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val1);
    const flatbuffers::String *string1 =
        reinterpret_cast<const flatbuffers::String *>(val1);
    elements->emplace_back(field_offset,
                           fbb->CreateString(string1->data(), string1->size()));
  }
}

// Merges an object to a builder for the provided field.
// One or both of t1 and t2 must be non-null.  If one is null, this method
// copies instead of merging.
void MergeTables(flatbuffers::voffset_t field_offset,
                 const flatbuffers::Table *t1, const flatbuffers::Table *t2,
                 const flatbuffers::TypeTable *sub_typetable,
                 flatbuffers::FlatBufferBuilder *fbb,
                 ::std::vector<OffsetAndFieldOffset> *elements) {
  const uint8_t *val1 =
      t1 != nullptr ? t1->GetAddressOf(field_offset) : nullptr;
  const uint8_t *val2 =
      t2 != nullptr ? t2->GetAddressOf(field_offset) : nullptr;
  const bool t1_has = val1 != nullptr;
  const bool t2_has = val2 != nullptr;
  if (t1_has || t2_has) {
    if (val1 != nullptr) {
      val1 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val1);
    }
    if (val2 != nullptr) {
      val2 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val2);
    }

    const flatbuffers::Table *sub_t1 =
        reinterpret_cast<const flatbuffers::Table *>(val1);
    const flatbuffers::Table *sub_t2 =
        reinterpret_cast<const flatbuffers::Table *>(val2);

    elements->emplace_back(
        field_offset, MergeFlatBuffers(sub_typetable, sub_t1, sub_t2, fbb));
  }
}

// Adds a vector of strings to the elements vector so it can be added later.
// One or both of t1 and t2 must be non-null.  If one is null, this method
// copies instead of merging.
void AddVectorOfStrings(flatbuffers::ElementaryType elementary_type,
                        flatbuffers::voffset_t field_offset,
                        const flatbuffers::Table *t1,
                        const flatbuffers::Table *t2,
                        flatbuffers::FlatBufferBuilder *fbb,
                        ::std::vector<OffsetAndFieldOffset> *elements) {
  const uint8_t *val1 =
      t1 != nullptr ? t1->GetAddressOf(field_offset) : nullptr;
  const uint8_t *val2 =
      t2 != nullptr ? t2->GetAddressOf(field_offset) : nullptr;
  const bool t1_has = val1 != nullptr;
  const bool t2_has = val2 != nullptr;

  // Compute end size of the vector.
  size_t size = 0;
  if (t1_has) {
    val1 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val1);
    auto vec1 = reinterpret_cast<
        const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> *>(
        val1);
    size += vec1->size();
  }
  if (t2_has) {
    val2 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val2);
    auto vec2 = reinterpret_cast<
        const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> *>(
        val2);
    size += vec2->size();
  }

  // Only add the vector if there is something to add.
  if (t1_has || t2_has) {
    const size_t inline_size =
        flatbuffers::InlineSize(elementary_type, nullptr);

    ::std::vector<flatbuffers::Offset<flatbuffers::String>> string_elements;

    // Pack the contents in in reverse order.
    if (t2_has) {
      auto vec2 = reinterpret_cast<const flatbuffers::Vector<
          flatbuffers::Offset<flatbuffers::String>> *>(val2);
      for (auto i = vec2->rbegin(); i != vec2->rend(); ++i) {
        const flatbuffers::String *s = *i;
        string_elements.emplace_back(fbb->CreateString(s->data(), s->size()));
      }
    }
    if (t1_has) {
      auto vec1 = reinterpret_cast<const flatbuffers::Vector<
          flatbuffers::Offset<flatbuffers::String>> *>(val1);
      for (auto i = vec1->rbegin(); i != vec1->rend(); ++i) {
        const flatbuffers::String *s = *i;
        string_elements.emplace_back(fbb->CreateString(s->data(), s->size()));
      }
    }

    // Start the vector.
    fbb->StartVector(size, inline_size);

    for (const flatbuffers::Offset<flatbuffers::String> &element :
         string_elements) {
      fbb->PushElement(element);
    }

    // And then finish the vector and put it in the list of offsets to add to
    // the message when it finishes.
    elements->emplace_back(
        field_offset,
        flatbuffers::Offset<flatbuffers::String>(fbb->EndVector(size)));
  }
}

// Adds a vector of values to the elements vector so it can be added later.
// One or both of t1 and t2 must be non-null.  If one is null, this method
// copies instead of merging.
template <typename T>
void AddVector(flatbuffers::ElementaryType elementary_type,
               flatbuffers::voffset_t field_offset,
               const flatbuffers::Table *t1, const flatbuffers::Table *t2,
               flatbuffers::FlatBufferBuilder *fbb,
               ::std::vector<OffsetAndFieldOffset> *elements) {
  const uint8_t *val1 =
      t1 != nullptr ? t1->GetAddressOf(field_offset) : nullptr;
  const uint8_t *val2 =
      t2 != nullptr ? t2->GetAddressOf(field_offset) : nullptr;
  const bool t1_has = val1 != nullptr;
  const bool t2_has = val2 != nullptr;

  // Compute end size of the vector.
  size_t size = 0;
  if (t1_has) {
    val1 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val1);
    auto vec1 = reinterpret_cast<const flatbuffers::Vector<T> *>(val1);
    size += vec1->size();
  }
  if (t2_has) {
    val2 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val2);
    auto vec2 = reinterpret_cast<const flatbuffers::Vector<T> *>(val2);
    size += vec2->size();
  }

  // Only add the vector if there is something to add.
  if (t1_has || t2_has) {
    const size_t inline_size =
        flatbuffers::InlineSize(elementary_type, nullptr);

    // Start the vector.
    fbb->StartVector(size, inline_size);

    // Pack the contents in in reverse order.
    if (t2_has) {
      auto vec2 = reinterpret_cast<const flatbuffers::Vector<T> *>(val2);
      // Iterate backwards.
      for (auto i = vec2->rbegin(); i != vec2->rend(); ++i) {
        fbb->PushElement<T>(*i);
      }
    }
    if (t1_has) {
      auto vec1 = reinterpret_cast<const flatbuffers::Vector<T> *>(val1);
      // Iterate backwards.
      for (auto i = vec1->rbegin(); i != vec1->rend(); ++i) {
        fbb->PushElement<T>(*i);
      }
    }
    // And then finish the vector and put it in the list of offsets to add to
    // the message when it finishes.
    elements->emplace_back(
        field_offset,
        flatbuffers::Offset<flatbuffers::String>(fbb->EndVector(size)));
  }
}

void AddVectorOfObjects(flatbuffers::FlatBufferBuilder *fbb,
                        ::std::vector<OffsetAndFieldOffset> *elements,
                        flatbuffers::ElementaryType elementary_type,
                        const flatbuffers::TypeTable *sub_typetable,
                        flatbuffers::voffset_t field_offset,
                        const flatbuffers::Table *t1,
                        const flatbuffers::Table *t2) {
  const uint8_t *val1 =
      t1 != nullptr ? t1->GetAddressOf(field_offset) : nullptr;
  const uint8_t *val2 =
      t2 != nullptr ? t2->GetAddressOf(field_offset) : nullptr;
  const bool t1_has = val1 != nullptr;
  const bool t2_has = val2 != nullptr;

  // Compute end size of the vector.
  size_t size = 0;
  if (t1_has) {
    val1 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val1);
    auto vec1 = reinterpret_cast<
        const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::Table>> *>(
        val1);
    size += vec1->size();
  }
  if (t2_has) {
    val2 += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val2);
    auto vec2 = reinterpret_cast<
        const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::Table>> *>(
        val2);
    size += vec2->size();
  }

  // Only add the vector if there is something to add.
  if (t1_has || t2_has) {
    const size_t inline_size =
        flatbuffers::InlineSize(elementary_type, sub_typetable);

    ::std::vector<flatbuffers::Offset<flatbuffers::Table>> object_elements;

    // Pack the contents in in reverse order.
    if (t2_has) {
      auto vec2 = reinterpret_cast<
          const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::Table>> *>(
          val2);
      for (auto i = vec2->rbegin(); i != vec2->rend(); ++i) {
        const flatbuffers::Table *t = *i;

        flatbuffers::Offset<flatbuffers::Table> end =
            MergeFlatBuffers(sub_typetable, t, nullptr, fbb);

        object_elements.emplace_back(end);
      }
    }
    if (t1_has) {
      auto vec1 = reinterpret_cast<
          const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::Table>> *>(
          val1);
      for (auto i = vec1->rbegin(); i != vec1->rend(); ++i) {
        const flatbuffers::Table *t = *i;

        flatbuffers::Offset<flatbuffers::Table> end =
            MergeFlatBuffers(sub_typetable, t, nullptr, fbb);

        object_elements.emplace_back(end);
      }
    }

    // Start the vector.
    fbb->StartVector(size, inline_size);

    for (const flatbuffers::Offset<flatbuffers::Table> &element :
         object_elements) {
      fbb->PushElement(element);
    }

    // And then finish the vector and put it in the list of offsets to add to
    // the message when it finishes.
    elements->emplace_back(
        field_offset,
        flatbuffers::Offset<flatbuffers::String>(fbb->EndVector(size)));
  }
}

}  // namespace

flatbuffers::Offset<flatbuffers::Table> MergeFlatBuffers(
    const flatbuffers::TypeTable *typetable, const flatbuffers::Table *t1,
    const flatbuffers::Table *t2, flatbuffers::FlatBufferBuilder *fbb) {
  ::std::vector<OffsetAndFieldOffset> elements;

  // We need to do this in 2 passes
  // The first pass builds up all the sub-objects which are encoded in the
  // message as offsets.
  // The second pass builds up the actual table by adding all the values to the
  // messages, and encoding the offsets in the table.
  for (size_t field_index = 0; field_index < typetable->num_elems;
       ++field_index) {
    const flatbuffers::TypeCode type_code = typetable->type_codes[field_index];
    const flatbuffers::ElementaryType elementary_type =
        static_cast<flatbuffers::ElementaryType>(type_code.base_type);

    const flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
        static_cast<flatbuffers::voffset_t>(field_index));

    switch (elementary_type) {
      case flatbuffers::ElementaryType::ET_UTYPE:
        if (!type_code.is_repeating) continue;
        printf("ET_UTYPE, %s\n", typetable->names[field_index]);
        break;
      case flatbuffers::ElementaryType::ET_BOOL:
        if (!type_code.is_repeating) continue;
        AddVector<uint8_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_CHAR:
        if (!type_code.is_repeating) continue;
        AddVector<int8_t>(elementary_type, field_offset, t1, t2, fbb,
                          &elements);
        break;
      case flatbuffers::ElementaryType::ET_UCHAR:
        if (!type_code.is_repeating) continue;
        AddVector<uint8_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_SHORT:
        if (!type_code.is_repeating) continue;
        AddVector<int16_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_USHORT:
        if (!type_code.is_repeating) continue;
        AddVector<uint16_t>(elementary_type, field_offset, t1, t2, fbb,
                            &elements);
        break;
      case flatbuffers::ElementaryType::ET_INT:
        if (!type_code.is_repeating) continue;
        AddVector<int32_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_UINT:
        if (!type_code.is_repeating) continue;
        AddVector<uint32_t>(elementary_type, field_offset, t1, t2, fbb,
                            &elements);
        break;
      case flatbuffers::ElementaryType::ET_LONG:
        if (!type_code.is_repeating) continue;
        AddVector<int64_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_ULONG:
        if (!type_code.is_repeating) continue;
        AddVector<uint64_t>(elementary_type, field_offset, t1, t2, fbb,
                            &elements);
        break;
      case flatbuffers::ElementaryType::ET_FLOAT:
        if (!type_code.is_repeating) continue;
        AddVector<float>(elementary_type, field_offset, t1, t2, fbb, &elements);
        break;
      case flatbuffers::ElementaryType::ET_DOUBLE:
        if (!type_code.is_repeating) continue;
        AddVector<double>(elementary_type, field_offset, t1, t2, fbb,
                          &elements);
        break;
      case flatbuffers::ElementaryType::ET_STRING:
        if (!type_code.is_repeating) {
          MergeString(field_offset, t1, t2, fbb, &elements);
        } else {
          AddVectorOfStrings(elementary_type, field_offset, t1, t2, fbb,
                             &elements);
        }
        break;
      case flatbuffers::ElementaryType::ET_SEQUENCE: {
        const flatbuffers::TypeTable *sub_typetable =
            typetable->type_refs[type_code.sequence_ref]();
        if (!type_code.is_repeating) {
          MergeTables(field_offset, t1, t2, sub_typetable, fbb, &elements);
        } else {
          const flatbuffers::TypeTable *sub_typetable =
              typetable->type_refs[type_code.sequence_ref]();

          AddVectorOfObjects(fbb, &elements, elementary_type, sub_typetable,
                             field_offset, t1, t2);
        }
      } break;
    }
  }

  const flatbuffers::uoffset_t start = fbb->StartTable();

  // We want to do this the same way as the json library.  Rip through the
  // fields and generate a list of things to add.  Then add them.
  // Also need recursion for subtypes.
  for (size_t field_index = 0; field_index < typetable->num_elems;
       ++field_index) {
    const flatbuffers::TypeCode type_code = typetable->type_codes[field_index];
    if (type_code.is_repeating) {
      continue;
    }
    const flatbuffers::ElementaryType elementary_type =
        static_cast<flatbuffers::ElementaryType>(type_code.base_type);

    const flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
        static_cast<flatbuffers::voffset_t>(field_index));

    switch (elementary_type) {
      case flatbuffers::ElementaryType::ET_UTYPE:
        // TODO(austin): Need to see one and try it.
        printf("ET_UTYPE, %s\n", typetable->names[field_index]);
        break;
      case flatbuffers::ElementaryType::ET_BOOL: {
        MergeElement<uint8_t>(field_offset, t1, t2, fbb);
      } break;
      case flatbuffers::ElementaryType::ET_CHAR:
        MergeElement<int8_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_UCHAR:
        MergeElement<uint8_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_SHORT:
        MergeElement<int16_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_USHORT:
        MergeElement<uint16_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_INT:
        MergeElement<int32_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_UINT:
        MergeElement<uint32_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_LONG:
        MergeElement<int64_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_ULONG:
        MergeElement<uint64_t>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_FLOAT:
        MergeElement<float>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_DOUBLE:
        MergeElement<double>(field_offset, t1, t2, fbb);
        break;
      case flatbuffers::ElementaryType::ET_STRING:
      case flatbuffers::ElementaryType::ET_SEQUENCE:
        // Already handled above since this is an uoffset.
        break;
    }
  }

  // And there is no need to check for duplicates since we are creating this
  // list very carefully from the type table.
  for (const OffsetAndFieldOffset &element : elements) {
    fbb->AddOffset(element.field_offset, element.element);
  }

  return fbb->EndTable(start);
}

bool CompareFlatBuffer(const flatbuffers::TypeTable *typetable,
                       const flatbuffers::Table *t1,
                       const flatbuffers::Table *t2) {
  // Copying flatbuffers is deterministic for the same typetable.  So, copy both
  // to guarantee that they are sorted the same, then check that the memory
  // matches.
  //
  // There has to be a better way to do this, but the efficiency hit of this
  // implementation is fine for the usages that we have now.  We are better off
  // abstracting this into a library call where we can fix it later easily.
  flatbuffers::FlatBufferBuilder fbb1;
  fbb1.ForceDefaults(true);
  fbb1.Finish(MergeFlatBuffers(typetable, t1, nullptr, &fbb1));
  flatbuffers::FlatBufferBuilder fbb2;
  fbb2.ForceDefaults(true);
  fbb2.Finish(MergeFlatBuffers(typetable, t2, nullptr, &fbb2));

  if (fbb1.GetSize() != fbb2.GetSize()) return false;

  return memcmp(fbb1.GetBufferPointer(), fbb2.GetBufferPointer(),
                fbb1.GetSize()) == 0;
}

// Struct to track a range of memory.
struct Bounds {
  const uint8_t *min;
  const uint8_t *max;

  absl::Span<const uint8_t> span() {
    return {min, static_cast<size_t>(max - min)};
  }
};

// Grows the range of memory to contain the pointer.
void Extend(Bounds *b, const uint8_t *ptr) {
  b->min = std::min(ptr, b->min);
  b->max = std::max(ptr, b->max);
}

// Grows the range of memory to contain the span.
void Extend(Bounds *b, absl::Span<const uint8_t> data) {
  b->min = std::min(data.data(), b->min);
  b->max = std::max(data.data() + data.size(), b->max);
}

// Finds the extents of the provided string.  Returns the containing span and
// required alignment.
std::pair<absl::Span<const uint8_t>, size_t> ExtentsString(
    const flatbuffers::String *s) {
  const uint8_t *s_uint8 = reinterpret_cast<const uint8_t *>(s);
  // Strings are null terminated.
  Bounds b{.min = s_uint8,
           .max = s_uint8 + sizeof(flatbuffers::uoffset_t) + s->size() + 1};
  return std::make_pair(b.span(), sizeof(flatbuffers::uoffset_t));
}

// Finds the extents of the provided table.  Returns the containing span and the
// required alignment.
std::pair<absl::Span<const uint8_t>, size_t> ExtentsTable(
    const flatbuffers::TypeTable *type_table, const flatbuffers::Table *t1) {
  const uint8_t *t1_uint8 = reinterpret_cast<const uint8_t *>(t1);
  // Count the offset to the vtable.
  Bounds b{.min = t1_uint8, .max = t1_uint8 + sizeof(flatbuffers::soffset_t)};
  // Find the limits of the vtable and start of table.
  const uint8_t *vt = t1->GetVTable();
  Extend(&b, vt);
  Extend(&b, vt + flatbuffers::ReadScalar<flatbuffers::voffset_t>(vt));
  // We need to be at least as aligned as the vtable pointer.  Start there.
  size_t alignment = sizeof(flatbuffers::uoffset_t);

  // Now do all our fields.
  for (size_t field_index = 0; field_index < type_table->num_elems;
       ++field_index) {
    const flatbuffers::TypeCode type_code = type_table->type_codes[field_index];
    const flatbuffers::ElementaryType elementary_type =
        static_cast<flatbuffers::ElementaryType>(type_code.base_type);
    const flatbuffers::TypeTable *field_type_table =
        type_code.sequence_ref >= 0
            ? type_table->type_refs[type_code.sequence_ref]()
            : nullptr;

    // Note: we don't yet support enums, structs, or unions.  That is mostly
    // because we haven't had a use case yet.

    // Compute the pointer to our field.
    const uint8_t *val = nullptr;
    if (type_table->st == flatbuffers::ST_TABLE) {
      val = t1->GetAddressOf(flatbuffers::FieldIndexToOffset(
          static_cast<flatbuffers::voffset_t>(field_index)));
      // Bail on non-populated fields.
      if (val == nullptr) continue;
    } else {
      val = t1_uint8 + type_table->values[field_index];
    }

    // Now make sure the field is aligned properly.
    const size_t field_size =
        flatbuffers::InlineSize(elementary_type, field_type_table);
    alignment = std::max(
        alignment, std::min(sizeof(flatbuffers::largest_scalar_t), field_size));

    absl::Span<const uint8_t> field_span(val, field_size);

    Extend(&b, field_span);

    if (type_code.is_repeating) {
      // Go look inside the vector and track the base size.
      val += flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val);
      const flatbuffers::Vector<uint8_t> *vec =
          reinterpret_cast<const flatbuffers::Vector<uint8_t> *>(val);
      absl::Span<const uint8_t> vec_span(
          val, sizeof(flatbuffers::uoffset_t) +
                   vec->size() * flatbuffers::InlineSize(elementary_type,
                                                         field_type_table));
      Extend(&b, vec_span);
      // Non-scalar vectors need their pointers followed.
      if (elementary_type == flatbuffers::ElementaryType::ET_STRING) {
        for (size_t i = 0; i < vec->size(); ++i) {
          const uint8_t *field_ptr =
              vec->Data() + i * InlineSize(elementary_type, field_type_table);
          std::pair<absl::Span<const uint8_t>, size_t> str_data =
              ExtentsString(reinterpret_cast<const flatbuffers::String *>(
                  field_ptr +
                  flatbuffers::ReadScalar<flatbuffers::uoffset_t>(field_ptr)));
          Extend(&b, str_data.first);
          alignment = std::max(alignment, str_data.second);
        }
      } else if (elementary_type == flatbuffers::ElementaryType::ET_SEQUENCE) {
        for (size_t i = 0; i < vec->size(); ++i) {
          const uint8_t *field_ptr =
              vec->Data() + i * InlineSize(elementary_type, field_type_table);
          CHECK(type_table->st == flatbuffers::ST_TABLE)
              << ": Only tables are supported right now.  Patches welcome.";

          std::pair<absl::Span<const uint8_t>, size_t> sub_data = ExtentsTable(
              field_type_table,
              reinterpret_cast<const flatbuffers::Table *>(
                  field_ptr +
                  flatbuffers::ReadScalar<flatbuffers::uoffset_t>(field_ptr)));
          alignment = std::max(alignment, sub_data.second);
          Extend(&b, sub_data.first);
        }
      }

      continue;
    }

    switch (elementary_type) {
      case flatbuffers::ElementaryType::ET_UTYPE:
      case flatbuffers::ElementaryType::ET_BOOL:
      case flatbuffers::ElementaryType::ET_CHAR:
      case flatbuffers::ElementaryType::ET_UCHAR:
      case flatbuffers::ElementaryType::ET_SHORT:
      case flatbuffers::ElementaryType::ET_USHORT:
      case flatbuffers::ElementaryType::ET_INT:
      case flatbuffers::ElementaryType::ET_UINT:
      case flatbuffers::ElementaryType::ET_LONG:
      case flatbuffers::ElementaryType::ET_ULONG:
      case flatbuffers::ElementaryType::ET_FLOAT:
      case flatbuffers::ElementaryType::ET_DOUBLE:
        // This is covered by the field and size above.
        break;
      case flatbuffers::ElementaryType::ET_STRING: {
        std::pair<absl::Span<const uint8_t>, size_t> str_data =
            ExtentsString(reinterpret_cast<const flatbuffers::String *>(
                val + flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val)));
        alignment = std::max(alignment, str_data.second);
        Extend(&b, str_data.first);
      } break;
      case flatbuffers::ElementaryType::ET_SEQUENCE: {
        switch (type_table->st) {
          case flatbuffers::ST_TABLE: {
            const flatbuffers::Table *sub_table =
                reinterpret_cast<const flatbuffers::Table *>(
                    val + flatbuffers::ReadScalar<flatbuffers::uoffset_t>(val));
            std::pair<absl::Span<const uint8_t>, size_t> sub_data =
                ExtentsTable(field_type_table, sub_table);
            alignment = std::max(alignment, sub_data.second);
            Extend(&b, sub_data.first);
          } break;
          case flatbuffers::ST_ENUM:
            LOG(FATAL) << "Copying enums not implemented yet";
          case flatbuffers::ST_STRUCT:
            LOG(FATAL) << "Copying structs not implemented yet";
          case flatbuffers::ST_UNION:
            LOG(FATAL) << "Copying unions not implemented yet";
        }
      }
    }
  }

  // To be a parsable flatbuffer, the flatbuffer needs to be aligned up to the
  // maximum internal alignment.  Both in length and starting point.  We know
  // that for this to be actually true, the start and end pointers will need to
  // be aligned to the required alignment.
  CHECK((alignment & (alignment - 1)) == 0)
      << ": Invalid alignment: " << alignment << ", needs to be a power of 2.";
  while (reinterpret_cast<uintptr_t>(b.min) & (alignment - 1)) {
    --b.min;
  }
  while (reinterpret_cast<uintptr_t>(b.max) & (alignment - 1)) {
    ++b.max;
  }

  return std::make_pair(b.span(), alignment);
}

// Computes the offset, containing span, and alignment of the provided
// flatbuffer.
std::tuple<flatbuffers::Offset<flatbuffers::Table>, absl::Span<const uint8_t>,
           size_t>
Extents(const flatbuffers::TypeTable *type_table,
        const flatbuffers::Table *t1) {
  std::pair<absl::Span<const uint8_t>, size_t> data =
      ExtentsTable(type_table, t1);

  return std::make_tuple(flatbuffers::Offset<flatbuffers::Table>(
                             static_cast<flatbuffers::uoffset_t>(
                                 data.first.data() + data.first.size() -
                                 reinterpret_cast<const uint8_t *>(t1))),
                         data.first, data.second);
}

flatbuffers::Offset<flatbuffers::Table> CopyFlatBuffer(
    const flatbuffers::Table *t1, const flatbuffers::TypeTable *typetable,
    flatbuffers::FlatBufferBuilder *fbb) {
  std::tuple<flatbuffers::Offset<flatbuffers::Table>, absl::Span<const uint8_t>,
             size_t>
      r = Extents(typetable, t1);

  // Pad out enough so that the flatbuffer alignment is preserved.
  fbb->Align(std::get<2>(r));

  // Now push everything we found.  And offsets are tracked from the end of the
  // buffer while building, so recompute the offset returned from the back.
  fbb->PushBytes(std::get<1>(r).data(), std::get<1>(r).size());
  return fbb->GetSize() + std::get<0>(r).o - std::get<1>(r).size();
}

}  // namespace aos
