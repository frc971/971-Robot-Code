#include "aos/flatbuffer_merge.h"

#include <stdio.h>

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
    fbb->AddElement<T>(field_offset, flatbuffers::ReadScalar<T>(val2), 0);
  } else if (t1_has) {
    fbb->AddElement<T>(field_offset, flatbuffers::ReadScalar<T>(val1), 0);
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

    elements->emplace_back(field_offset,
                          MergeFlatBuffers(sub_typetable, sub_t1, sub_t2, fbb));
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
        if (!type_code.is_vector) continue;
        printf("ET_UTYPE, %s\n", typetable->names[field_index]);
        break;
      case flatbuffers::ElementaryType::ET_BOOL:
        if (!type_code.is_vector) continue;
        AddVector<uint8_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_CHAR:
        if (!type_code.is_vector) continue;
        AddVector<int8_t>(elementary_type, field_offset, t1, t2, fbb,
                          &elements);
        break;
      case flatbuffers::ElementaryType::ET_UCHAR:
        if (!type_code.is_vector) continue;
        AddVector<uint8_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_SHORT:
        if (!type_code.is_vector) continue;
        AddVector<int16_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_USHORT:
        if (!type_code.is_vector) continue;
        AddVector<uint16_t>(elementary_type, field_offset, t1, t2, fbb,
                            &elements);
        break;
      case flatbuffers::ElementaryType::ET_INT:
        if (!type_code.is_vector) continue;
        AddVector<int32_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_UINT:
        if (!type_code.is_vector) continue;
        AddVector<uint32_t>(elementary_type, field_offset, t1, t2, fbb,
                            &elements);
        break;
      case flatbuffers::ElementaryType::ET_LONG:
        if (!type_code.is_vector) continue;
        AddVector<int64_t>(elementary_type, field_offset, t1, t2, fbb,
                           &elements);
        break;
      case flatbuffers::ElementaryType::ET_ULONG:
        if (!type_code.is_vector) continue;
        AddVector<uint64_t>(elementary_type, field_offset, t1, t2, fbb,
                            &elements);
        break;
      case flatbuffers::ElementaryType::ET_FLOAT:
        if (!type_code.is_vector) continue;
        AddVector<float>(elementary_type, field_offset, t1, t2, fbb, &elements);
        break;
      case flatbuffers::ElementaryType::ET_DOUBLE:
        if (!type_code.is_vector) continue;
        AddVector<double>(elementary_type, field_offset, t1, t2, fbb,
                          &elements);
        break;
      case flatbuffers::ElementaryType::ET_STRING:
        if (!type_code.is_vector) {
          MergeString(field_offset, t1, t2, fbb, &elements);
        } else {
          AddVectorOfStrings(elementary_type, field_offset, t1, t2, fbb,
                             &elements);
        }
        break;
      case flatbuffers::ElementaryType::ET_SEQUENCE: {
        const flatbuffers::TypeTable *sub_typetable =
            typetable->type_refs[type_code.sequence_ref]();
        if (!type_code.is_vector) {
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
    if (type_code.is_vector) {
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

flatbuffers::Offset<flatbuffers::Table> MergeFlatBuffers(
    const flatbuffers::TypeTable *typetable, const uint8_t *data1,
    const uint8_t *data2, flatbuffers::FlatBufferBuilder *fbb) {
  // Grab the 2 tables.
  const flatbuffers::Table *t1 =
      data1 != nullptr ? flatbuffers::GetRoot<flatbuffers::Table>(data1)
                       : nullptr;
  const flatbuffers::Table *t2 =
      data2 != nullptr ? flatbuffers::GetRoot<flatbuffers::Table>(data2)
                       : nullptr;

  // And then do the actual build.  This doesn't contain finish so we can nest
  // them nicely.
  return flatbuffers::Offset<flatbuffers::Table>(
      MergeFlatBuffers(typetable, t1, t2, fbb));
}

flatbuffers::DetachedBuffer MergeFlatBuffers(
    const flatbuffers::TypeTable *typetable, const uint8_t *data1,
    const uint8_t *data2) {
  // Build up a builder.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);

  // Finish up the buffer and return it.
  fbb.Finish(MergeFlatBuffers(typetable, data1, data2, &fbb));

  return fbb.Release();
}

}  // namespace aos
