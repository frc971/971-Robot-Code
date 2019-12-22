#include "aos/json_to_flatbuffer.h"

#include <cstddef>
#include "stdio.h"

#include <string_view>

#include "aos/flatbuffer_utils.h"
#include "aos/json_tokenizer.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/minireflect.h"
#include "glog/logging.h"

// TODO(austin): Can we just do an Offset<void> ?  It doesn't matter, so maybe
// just say that.
//
// TODO(austin): I've yet to see how to create an ET_UTYPE, so I don't know what
// one is and how to test it.  So everything rejects it.

namespace aos {

// Finds the field index in the table given the name.
int FieldIndex(const flatbuffers::TypeTable *typetable,
               const char *field_name) {
  CHECK(typetable->values == nullptr);
  for (size_t i = 0; i < typetable->num_elems; ++i) {
    if (strcmp(field_name, typetable->names[i]) == 0) {
      return i;
    }
  }
  return -1;
}

namespace {

// Class to hold one of the 3 json types for an array.
struct Element {
  // The type.
  enum class ElementType { INT, DOUBLE, OFFSET };

  // Constructs an Element holding an integer.
  Element(int64_t new_int_element)
      : int_element(new_int_element), type(ElementType::INT) {}
  // Constructs an Element holding an double.
  Element(double new_double_element)
      : double_element(new_double_element), type(ElementType::DOUBLE) {}
  // Constructs an Element holding an Offset.
  Element(flatbuffers::Offset<flatbuffers::String> new_offset_element)
      : offset_element(new_offset_element), type(ElementType::OFFSET) {}

  // Union for the various datatypes.
  union {
    int64_t int_element;
    double double_element;
    flatbuffers::Offset<flatbuffers::String> offset_element;
  };

  // And an enum signaling which one is in use.
  ElementType type;
};

// Structure to represent a field element.
struct FieldElement {
  FieldElement(int new_field_index, int64_t int_element)
      : element(int_element), field_index(new_field_index) {}
  FieldElement(int new_field_index, double double_element)
      : element(double_element), field_index(new_field_index) {}
  FieldElement(int new_field_index,
               flatbuffers::Offset<flatbuffers::String> offset_element)
      : element(offset_element), field_index(new_field_index) {}

  // Data to write.
  Element element;
  // Field index.  The type table which this index is for is stored outside this
  // object.
  int field_index;
};

// Adds a single element.  This assumes that vectors have been dealt with
// already.  Returns true on success.
bool AddSingleElement(const flatbuffers::TypeTable *typetable,
                      const FieldElement &field_element,
                      ::std::vector<bool> *fields_in_use,
                      flatbuffers::FlatBufferBuilder *fbb);
bool AddSingleElement(const flatbuffers::TypeTable *typetable, int field_index,
                      int64_t int_value, flatbuffers::FlatBufferBuilder *fbb);
bool AddSingleElement(const flatbuffers::TypeTable *typetable, int field_index,
                      double double_value, flatbuffers::FlatBufferBuilder *fbb);
bool AddSingleElement(const flatbuffers::TypeTable *typetable, int field_index,
                      flatbuffers::Offset<flatbuffers::String> offset_element,
                      flatbuffers::FlatBufferBuilder *fbb);


// Writes an array of FieldElement (with the definition in the type
// table) to the builder.  Returns the offset of the table.
flatbuffers::uoffset_t WriteTable(const flatbuffers::TypeTable *typetable,
                                  const ::std::vector<FieldElement> &elements,
                                  flatbuffers::FlatBufferBuilder *fbb) {
  // End of a nested struct!  Add it.
  const flatbuffers::uoffset_t start = fbb->StartTable();

  ::std::vector<bool> fields_in_use(typetable->num_elems, false);

  for (const FieldElement &field_element : elements) {
    AddSingleElement(typetable, field_element, &fields_in_use, fbb);
  }

  return fbb->EndTable(start);
}

// Class to parse JSON into a flatbuffer.
//
// The basic strategy is that we need to do everything backwards.  So we need to
// build up what we need to do fully in memory, then do it.
//
// The driver for this is that strings need to be fully created before the
// tables that use them.  Same for sub messages.  But, we only know we have them
// all when the structure ends.  So, store each sub message in a
// FieldElement and put them in the table at the end when we finish up
// each message.  Same goes for vectors.
class JsonParser {
 public:
  JsonParser() { fbb_.ForceDefaults(1); }
  ~JsonParser() {}

  // Parses the json into a flatbuffer.  Returns either an empty vector on
  // error, or a vector with the flatbuffer data in it.
  flatbuffers::DetachedBuffer Parse(const std::string_view data,
                                    const flatbuffers::TypeTable *typetable) {
    flatbuffers::uoffset_t end = 0;
    bool result = DoParse(typetable, data, &end);

    if (result) {
      // On success, finish the table and build the vector.
      auto o = flatbuffers::Offset<flatbuffers::Table>(end);
      fbb_.Finish(o);

      return fbb_.Release();
    } else {
      // Otherwise return an empty vector.
      return flatbuffers::DetachedBuffer();
    }
  }

 private:
  // Setters and getters for in_vector (at the current level of the stack)
  bool in_vector() const { return stack_.back().in_vector; }
  void set_in_vector(bool in_vector) { stack_.back().in_vector = in_vector; }

  // Parses the flatbuffer.  This is a second method so we can do easier
  // cleanup at the top level.  Returns true on success.
  bool DoParse(const flatbuffers::TypeTable *typetable,
               const std::string_view data,
               flatbuffers::uoffset_t *table_end);

  // Adds *_value for the provided field.  If we are in a vector, queues the
  // data up in vector_elements.  Returns true on success.
  bool AddElement(int field_index, int64_t int_value);
  bool AddElement(int field_index, double double_value);
  bool AddElement(int field_index, const ::std::string &data);

  // Finishes a vector for the provided field index.  Returns true on success.
  bool FinishVector(int field_index);

  // Pushes an element as part of a vector.  Returns true on success.
  bool PushElement(flatbuffers::ElementaryType elementary_type,
                   int64_t int_value);
  bool PushElement(flatbuffers::ElementaryType elementary_type,
                   double double_value);
  bool PushElement(flatbuffers::ElementaryType elementary_type,
                   flatbuffers::Offset<flatbuffers::String> offset_value);

  flatbuffers::FlatBufferBuilder fbb_;

  // This holds the state information that is needed as you recurse into
  // nested structures.
  struct FlatBufferContext {
    // Type of the current type.
    const flatbuffers::TypeTable *typetable;
    // If true, we are parsing a vector.
    bool in_vector;
    // The field index of the current field.
    int field_index;
    // Name of the current field.
    ::std::string field_name;

    // Field elements that need to be inserted.
    ::std::vector<FieldElement> elements;

    // For scalar types (not strings, and not nested tables), the vector ends
    // up being implemented as a start and end, and a block of data.  So we
    // can't just push offsets in as we go.  We either need to reproduce the
    // logic inside flatbuffers, or build up vectors of the data.  Vectors will
    // be a bit of extra stack space, but whatever.
    //
    // Strings and nested structures are vectors of offsets.
    // into the vector. Once you get to the end, you build up a vector and
    // push that into the field.
    ::std::vector<Element> vector_elements;
  };
  ::std::vector<FlatBufferContext> stack_;
};

bool JsonParser::DoParse(const flatbuffers::TypeTable *typetable,
                         const std::string_view data,
                         flatbuffers::uoffset_t *table_end) {
  ::std::vector<const flatbuffers::TypeTable *> stack;

  Tokenizer t(data);

  // Main loop.  Run until we get an end.
  while (true) {
    Tokenizer::TokenType token = t.Next();

    switch (token) {
      case Tokenizer::TokenType::kEnd:
        if (stack_.size() != 0) {
          fprintf(stderr, "Failed to unwind stack all the way\n");
          return false;
        } else {
          return true;
        }
        break;
      case Tokenizer::TokenType::kError:
        return false;
        break;

      case Tokenizer::TokenType::kStartObject:  // {
        if (stack_.size() == 0) {
          stack_.push_back({typetable, false, -1, "", {}, {}});
        } else {
          int field_index = stack_.back().field_index;

          const flatbuffers::TypeCode &type_code =
              stack_.back().typetable->type_codes[field_index];

          if (type_code.base_type != flatbuffers::ET_SEQUENCE) {
            fprintf(stderr, "Field '%s' is not a sequence\n",
                    stack_.back().field_name.c_str());
            return false;
          }

          flatbuffers::TypeFunction type_function =
              stack_.back().typetable->type_refs[type_code.sequence_ref];

          stack_.push_back({type_function(), false, -1, "", {}, {}});
        }
        break;
      case Tokenizer::TokenType::kEndObject:  // }
        if (stack_.size() == 0) {
          // Somehow we popped more than we pushed.  Error.
          fprintf(stderr, "Empty stack\n");
          return false;
        } else {
          // End of a nested struct!  Add it.
          const flatbuffers::uoffset_t end = WriteTable(
              stack_.back().typetable, stack_.back().elements, &fbb_);

          // We now want to talk about the parent structure.  Pop the child.
          stack_.pop_back();

          if (stack_.size() == 0) {
            // Instead of queueing it up in the stack, return it through the
            // passed in variable.
            *table_end = end;
          } else {
            // And now we can add it.
            const int field_index = stack_.back().field_index;

            // Do the right thing if we are in a vector.
            if (in_vector()) {
              stack_.back().vector_elements.emplace_back(
                  flatbuffers::Offset<flatbuffers::String>(end));
            } else {
              stack_.back().elements.emplace_back(
                  field_index, flatbuffers::Offset<flatbuffers::String>(end));
            }
          }
        }
        break;

      case Tokenizer::TokenType::kStartArray:  // [
        if (stack_.size() == 0) {
          // We don't support an array of structs at the root level.
          return false;
        }
        // Sanity check that we aren't trying to make a vector of vectors.
        if (in_vector()) {
          return false;
        }
        set_in_vector(true);

        break;
      case Tokenizer::TokenType::kEndArray: {  // ]
        if (!in_vector()) {
          return false;
        }

        const int field_index = stack_.back().field_index;

        if (!FinishVector(field_index)) return false;

        set_in_vector(false);
      } break;

      case Tokenizer::TokenType::kTrueValue:   // true
      case Tokenizer::TokenType::kFalseValue:  // false
      case Tokenizer::TokenType::kNumberValue: {
        bool is_int = true;
        double double_value;
        long long int_value;
        if (token == Tokenizer::TokenType::kTrueValue) {
          int_value = 1;
        } else if (token == Tokenizer::TokenType::kFalseValue) {
          int_value = 0;
        } else if (!t.FieldAsInt(&int_value)) {
          if (t.FieldAsDouble(&double_value)) {
            is_int = false;
          } else {
            fprintf(stderr, "Got a invalid number '%s'\n",
                    t.field_value().c_str());
            return false;
          }
        }

        const int field_index = stack_.back().field_index;

        if (is_int) {
          // No need to get too stressed about bool vs int.  Convert them all.
          int64_t val = int_value;
          if (!AddElement(field_index, val)) return false;
        } else {
          if (!AddElement(field_index, double_value)) return false;
        }
      } break;
      case Tokenizer::TokenType::kStringValue:  // string value
      {
        const int field_index = stack_.back().field_index;

        if (!AddElement(field_index, t.field_value())) return false;
      } break;
      case Tokenizer::TokenType::kField:  // field name
      {
        stack_.back().field_name = t.field_name();
        stack_.back().field_index = FieldIndex(
            stack_.back().typetable, stack_.back().field_name.c_str());

        if (stack_.back().field_index == -1) {
          fprintf(stderr, "Invalid field name '%s'\n",
                  stack_.back().field_name.c_str());
          return false;
        }
      } break;
    }
  }
  return false;
}

bool JsonParser::AddElement(int field_index, int64_t int_value) {
  flatbuffers::TypeCode type_code =
      stack_.back().typetable->type_codes[field_index];

  if (type_code.is_vector != in_vector()) {
    fprintf(stderr, "Type and json disagree on if we are in a vector or not\n");
    return false;
  }

  if (in_vector()) {
    stack_.back().vector_elements.emplace_back(int_value);
  } else {
    stack_.back().elements.emplace_back(field_index, int_value);
  }
  return true;
}

bool JsonParser::AddElement(int field_index, double double_value) {
  flatbuffers::TypeCode type_code =
      stack_.back().typetable->type_codes[field_index];

  if (type_code.is_vector != in_vector()) {
    fprintf(stderr, "Type and json disagree on if we are in a vector or not\n");
    return false;
  }

  if (in_vector()) {
    stack_.back().vector_elements.emplace_back(double_value);
  } else {
    stack_.back().elements.emplace_back(field_index, double_value);
  }
  return true;
}

bool JsonParser::AddElement(int field_index, const ::std::string &data) {
  flatbuffers::TypeCode type_code =
      stack_.back().typetable->type_codes[field_index];

  if (type_code.is_vector != in_vector()) {
    fprintf(stderr, "Type and json disagree on if we are in a vector or not\n");
    return false;
  }

  const flatbuffers::ElementaryType elementary_type =
      static_cast<flatbuffers::ElementaryType>(type_code.base_type);
  switch (elementary_type) {
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
      if (type_code.sequence_ref != -1) {
        // We have an enum.
        const flatbuffers::TypeTable *type_table = stack_.back().typetable;
        flatbuffers::TypeFunction type_function =
            type_table->type_refs[type_code.sequence_ref];

        const flatbuffers::TypeTable *enum_type_table = type_function();

        CHECK_EQ(enum_type_table->st, flatbuffers::ST_ENUM);

        int64_t int_value = 0;
        bool found = false;
        for (size_t i = 0; i < enum_type_table->num_elems; ++i) {
          if (data == enum_type_table->names[i]) {
            int_value = i;
            found = true;
            break;
          }
        }

        if (!found) {
          fprintf(stderr, "Enum value '%s' not found for field '%s'\n",
                  data.c_str(), type_table->names[field_index]);
          return false;
        }

        if (in_vector()) {
          stack_.back().vector_elements.emplace_back(int_value);
        } else {
          stack_.back().elements.emplace_back(field_index, int_value);
        }
        return true;
      }
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_FLOAT:
    case flatbuffers::ET_DOUBLE:
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      break;
  }

  if (in_vector()) {
    stack_.back().vector_elements.emplace_back(fbb_.CreateString(data));
  } else {
    stack_.back().elements.emplace_back(field_index, fbb_.CreateString(data));
  }
  return true;
}

bool AddSingleElement(const flatbuffers::TypeTable *typetable,
                      const FieldElement &field_element,
                      ::std::vector<bool> *fields_in_use,
                      flatbuffers::FlatBufferBuilder *fbb) {
  if ((*fields_in_use)[field_element.field_index]) {
    fprintf(stderr, "Duplicate field: '%s'\n",
            typetable->names[field_element.field_index]);
    return false;
  }

  (*fields_in_use)[field_element.field_index] = true;

  switch (field_element.element.type) {
    case Element::ElementType::INT:
      return AddSingleElement(typetable, field_element.field_index,
                              field_element.element.int_element, fbb);
    case Element::ElementType::DOUBLE:
      return AddSingleElement(typetable, field_element.field_index,
                              field_element.element.double_element, fbb);
    case Element::ElementType::OFFSET:
      return AddSingleElement(typetable, field_element.field_index,
                              field_element.element.offset_element, fbb);
  }
  return false;
}

bool AddSingleElement(const flatbuffers::TypeTable *typetable, int field_index,
                      int64_t int_value, flatbuffers::FlatBufferBuilder *fbb

) {
  flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
      static_cast<flatbuffers::voffset_t>(field_index));

  flatbuffers::TypeCode type_code = typetable->type_codes[field_index];

  const flatbuffers::ElementaryType elementary_type =
      static_cast<flatbuffers::ElementaryType>(type_code.base_type);
  switch (elementary_type) {
    case flatbuffers::ET_BOOL:
      fbb->AddElement<bool>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_CHAR:
      fbb->AddElement<int8_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_UCHAR:
      fbb->AddElement<uint8_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_SHORT:
      fbb->AddElement<int16_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_USHORT:
      fbb->AddElement<uint16_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_INT:
      fbb->AddElement<int32_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_UINT:
      fbb->AddElement<uint32_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_LONG:
      fbb->AddElement<int64_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_ULONG:
      fbb->AddElement<uint64_t>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_FLOAT:
      fbb->AddElement<float>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb->AddElement<double>(field_offset, int_value, 0);
      return true;
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_SEQUENCE:
      fprintf(
          stderr, "Mismatched type for field '%s'. Got: integer, expected %s\n",
          typetable->names[field_index], ElementaryTypeName(elementary_type));
      return false;
  };
  return false;
}

bool AddSingleElement(const flatbuffers::TypeTable *typetable, int field_index,
                      double double_value,
                      flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
      static_cast<flatbuffers::voffset_t>(field_index));

  flatbuffers::TypeCode type_code = typetable->type_codes[field_index];

  const flatbuffers::ElementaryType elementary_type =
      static_cast<flatbuffers::ElementaryType>(type_code.base_type);
  switch (elementary_type) {
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      fprintf(
          stderr, "Mismatched type for field '%s'. Got: double, expected %s\n",
          typetable->names[field_index], ElementaryTypeName(elementary_type));
      return false;
    case flatbuffers::ET_FLOAT:
      fbb->AddElement<float>(field_offset, double_value, 0);
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb->AddElement<double>(field_offset, double_value, 0);
      return true;
  }
  return false;
}
bool AddSingleElement(const flatbuffers::TypeTable *typetable, int field_index,
                      flatbuffers::Offset<flatbuffers::String> offset_element,
                      flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::TypeCode type_code = typetable->type_codes[field_index];

  flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
      static_cast<flatbuffers::voffset_t>(field_index));

  // Vectors will always be Offset<>'s.
  if (type_code.is_vector) {
    fbb->AddOffset(field_offset, offset_element);
    return true;
  }

  const flatbuffers::ElementaryType elementary_type =
      static_cast<flatbuffers::ElementaryType>(type_code.base_type);
  switch (elementary_type) {
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_FLOAT:
    case flatbuffers::ET_DOUBLE:
      fprintf(
          stderr, "Mismatched type for field '%s'. Got: string, expected %s\n",
          typetable->names[field_index], ElementaryTypeName(elementary_type));
      CHECK_EQ(type_code.sequence_ref, -1)
          << ": Field name " << typetable->names[field_index]
          << " Got string expected " << ElementaryTypeName(elementary_type);
      return false;
    case flatbuffers::ET_STRING:
      CHECK_EQ(type_code.sequence_ref, -1);
    case flatbuffers::ET_SEQUENCE:
      fbb->AddOffset(field_offset, offset_element);
      return true;
  }
  return false;
}

bool JsonParser::FinishVector(int field_index) {
  flatbuffers::TypeCode type_code =
      stack_.back().typetable->type_codes[field_index];

  const flatbuffers::ElementaryType elementary_type =
      static_cast<flatbuffers::ElementaryType>(type_code.base_type);

  // Vectors have a start (unfortunately which needs to know the size)
  fbb_.StartVector(
      stack_.back().vector_elements.size(),
      flatbuffers::InlineSize(elementary_type, stack_.back().typetable));

  // Then the data (in reverse order for some reason...)
  for (size_t i = stack_.back().vector_elements.size(); i > 0;) {
    const Element &element = stack_.back().vector_elements[--i];
    switch (element.type) {
      case Element::ElementType::INT:
        if (!PushElement(elementary_type, element.int_element)) return false;
        break;
      case Element::ElementType::DOUBLE:
        CHECK_EQ(type_code.sequence_ref, -1)
            << ": Field index is " << field_index;
        if (!PushElement(elementary_type, element.double_element)) return false;
        break;
      case Element::ElementType::OFFSET:
        if (!PushElement(elementary_type, element.offset_element)) return false;
        break;
    }
  }

  // Then an End which is placed into the buffer the same as any other offset.
  stack_.back().elements.emplace_back(
      field_index, flatbuffers::Offset<flatbuffers::String>(
                       fbb_.EndVector(stack_.back().vector_elements.size())));
  stack_.back().vector_elements.clear();
  return true;
}

bool JsonParser::PushElement(flatbuffers::ElementaryType elementary_type,
                             int64_t int_value) {
  switch (elementary_type) {
    case flatbuffers::ET_BOOL:
      fbb_.PushElement<bool>(int_value);
      return true;
    case flatbuffers::ET_CHAR:
      fbb_.PushElement<int8_t>(int_value);
      return true;
    case flatbuffers::ET_UCHAR:
      fbb_.PushElement<uint8_t>(int_value);
      return true;
    case flatbuffers::ET_SHORT:
      fbb_.PushElement<int16_t>(int_value);
      return true;
    case flatbuffers::ET_USHORT:
      fbb_.PushElement<uint16_t>(int_value);
      return true;
    case flatbuffers::ET_INT:
      fbb_.PushElement<int32_t>(int_value);
      return true;
    case flatbuffers::ET_UINT:
      fbb_.PushElement<uint32_t>(int_value);
      return true;
    case flatbuffers::ET_LONG:
      fbb_.PushElement<int64_t>(int_value);
      return true;
    case flatbuffers::ET_ULONG:
      fbb_.PushElement<uint64_t>(int_value);
      return true;
    case flatbuffers::ET_FLOAT:
      fbb_.PushElement<float>(int_value);
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb_.PushElement<double>(int_value);
      return true;
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_SEQUENCE:
      fprintf(stderr,
              "Mismatched type for field '%s'. Got: integer, expected %s\n",
              stack_.back().field_name.c_str(),
              ElementaryTypeName(elementary_type));
      return false;
  };
  return false;
}

bool JsonParser::PushElement(flatbuffers::ElementaryType elementary_type,
                             double double_value) {
  switch (elementary_type) {
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      fprintf(stderr,
              "Mismatched type for field '%s'. Got: double, expected %s\n",
              stack_.back().field_name.c_str(),
              ElementaryTypeName(elementary_type));
      return false;
    case flatbuffers::ET_FLOAT:
      fbb_.PushElement<float>(double_value);
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb_.PushElement<double>(double_value);
      return true;
  }
  return false;
}

bool JsonParser::PushElement(
    flatbuffers::ElementaryType elementary_type,
    flatbuffers::Offset<flatbuffers::String> offset_value) {
  switch (elementary_type) {
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_FLOAT:
    case flatbuffers::ET_DOUBLE:
      fprintf(stderr,
              "Mismatched type for field '%s'. Got: sequence, expected %s\n",
              stack_.back().field_name.c_str(),
              ElementaryTypeName(elementary_type));
      return false;
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      fbb_.PushElement(offset_value);
      return true;
  }
  return false;
}

}  // namespace

flatbuffers::DetachedBuffer JsonToFlatbuffer(
    const std::string_view data,
    const flatbuffers::TypeTable *typetable) {
  JsonParser p;
  return p.Parse(data, typetable);
}

::std::string BufferFlatbufferToJson(const uint8_t *buffer,
                                     const ::flatbuffers::TypeTable *typetable,
                                     bool multi_line) {
  // It is pretty common to get passed in a nullptr when a test fails.  Rather
  // than CHECK, return a more user friendly result.
  if (buffer == nullptr) {
    return "null";
  }
  return TableFlatbufferToJson(reinterpret_cast<const flatbuffers::Table *>(
                                   flatbuffers::GetRoot<uint8_t>(buffer)),
                               typetable, multi_line);
}

::std::string TableFlatbufferToJson(const flatbuffers::Table *t,
                                    const ::flatbuffers::TypeTable *typetable,
                                    bool multi_line) {
  // It is pretty common to get passed in a nullptr when a test fails.  Rather
  // than CHECK, return a more user friendly result.
  if (t == nullptr) {
    return "null";
  }
  ::flatbuffers::ToStringVisitor tostring_visitor(
      multi_line ? "\n" : " ", true, multi_line ? " " : "", multi_line);
  flatbuffers::IterateObject(reinterpret_cast<const uint8_t *>(t), typetable,
                             &tostring_visitor);
  return tostring_visitor.s;
}

}  // namespace aos
