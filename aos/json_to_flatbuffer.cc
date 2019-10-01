#include "aos/json_to_flatbuffer.h"

#include <cstddef>
#include "stdio.h"

#include "aos/flatbuffer_utils.h"
#include "aos/logging/logging.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/minireflect.h"

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
  ::std::vector<uint8_t> Parse(const char *data,
                               const flatbuffers::TypeTable *typetable) {
    flatbuffers::uoffset_t end = 0;
    bool result = DoParse(typetable, data, &end);

    if (result) {
      // On success, finish the table and build the vector.
      auto o = flatbuffers::Offset<flatbuffers::Table>(end);
      fbb_.Finish(o);

      const uint8_t *buf = fbb_.GetBufferPointer();
      const int size = fbb_.GetSize();
      return ::std::vector<uint8_t>(buf, buf + size);
    } else {
      // Otherwise return an empty vector.
      return ::std::vector<uint8_t>();
    }
  }

 private:
  // Setters and getters for in_vector (at the current level of the stack)
  bool in_vector() const { return stack_.back().in_vector; }
  void set_in_vector(bool in_vector) { stack_.back().in_vector = in_vector; }

  // Parses the flatbuffer.  This is a second method so we can do easier
  // cleanup at the top level.  Returns true on success.
  bool DoParse(const flatbuffers::TypeTable *typetable, const char *data,
               flatbuffers::uoffset_t *table_end);

  // Adds *_value for the provided field.  If we are in a vector, queues the
  // data up in vector_elements_.  Returns true on success.
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
  };
  ::std::vector<FlatBufferContext> stack_;

  // For scalar types (not strings, and not nested tables), the vector ends
  // up being implemented as a start and end, and a block of data.  So we
  // can't just push offsets in as we go.  We either need to reproduce the
  // logic inside flatbuffers, or build up vectors of the data.  Vectors will
  // be a bit of extra stack space, but whatever.
  //
  // Strings and nested structures are vectors of offsets.
  // into the vector. Once you get to the end, you build up a vector and
  // push that into the field.
  ::std::vector<Element> vector_elements_;
};

bool JsonParser::DoParse(const flatbuffers::TypeTable *typetable,
                         const char *data, flatbuffers::uoffset_t *table_end) {
  ::std::vector<const flatbuffers::TypeTable *> stack;

  Tokenizer t(data);

  // Main loop.  Run until we get an end.
  while (true) {
    Tokenizer::TokenType token = t.Next();

    switch (token) {
      case Tokenizer::TokenType::kEnd:
        if (stack_.size() != 0) {
          printf("Failed to unwind stack all the way\n");
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
          stack_.push_back({typetable, false, -1, "", {}});
        } else {
          int field_index = stack_.back().field_index;

          const flatbuffers::TypeCode &type_code =
              stack_.back().typetable->type_codes[field_index];

          if (type_code.base_type != flatbuffers::ET_SEQUENCE) {
            printf("Field '%s' is not a sequence\n",
                   stack_.back().field_name.c_str());
            return false;
          }

          flatbuffers::TypeFunction type_function =
              stack_.back().typetable->type_refs[type_code.sequence_ref];

          stack_.push_back({type_function(), false, -1, "", {}});
        }
        break;
      case Tokenizer::TokenType::kEndObject:  // }
        if (stack_.size() == 0) {
          // Somehow we popped more than we pushed.  Error.
          printf("Empty stack\n");
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
              vector_elements_.emplace_back(
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
      // TODO(austin): Need to detect int vs float.
      /*
      asdf
      {
        const int field_index = stack_.back().field_index;

      } break;
      */
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
          printf("Invalid field name '%s'\n", stack_.back().field_name.c_str());
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
    printf("Type and json disagree on if we are in a vector or not\n");
    return false;
  }

  if (in_vector()) {
    vector_elements_.emplace_back(int_value);
  } else {
    stack_.back().elements.emplace_back(field_index, int_value);
  }
  return true;
}

bool JsonParser::AddElement(int field_index, double double_value) {
  flatbuffers::TypeCode type_code =
      stack_.back().typetable->type_codes[field_index];

  if (type_code.is_vector != in_vector()) {
    printf("Type and json disagree on if we are in a vector or not\n");
    return false;
  }

  if (in_vector()) {
    vector_elements_.emplace_back(double_value);
  } else {
    stack_.back().elements.emplace_back(field_index, double_value);
  }
  return true;
}

bool JsonParser::AddElement(int field_index, const ::std::string &data) {
  flatbuffers::TypeCode type_code =
      stack_.back().typetable->type_codes[field_index];

  if (type_code.is_vector != in_vector()) {
    printf("Type and json disagree on if we are in a vector or not\n");
    return false;
  }

  if (in_vector()) {
    vector_elements_.emplace_back(fbb_.CreateString(data));

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
    printf("Duplicate field: '%s'\n",
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
      printf("Mismatched type for field '%s'. Got: integer, expected %s\n",
             typetable->names[field_index],
             ElementaryTypeName(elementary_type));
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
      printf("Mismatched type for field '%s'. Got: double, expected %s\n",
             typetable->names[field_index],
             ElementaryTypeName(elementary_type));
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
      printf("Mismatched type for field '%s'. Got: string, expected %s\n",
             typetable->names[field_index],
             ElementaryTypeName(elementary_type));
      return false;
    case flatbuffers::ET_SEQUENCE:
    case flatbuffers::ET_STRING:
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
      vector_elements_.size(),
      flatbuffers::InlineSize(elementary_type, stack_.back().typetable));

  // Then the data (in reverse order for some reason...)
  for (size_t i = vector_elements_.size(); i > 0;) {
    const Element &element = vector_elements_[--i];
    switch (element.type) {
      case Element::ElementType::INT:
        if (!PushElement(elementary_type, element.int_element)) return false;
        break;
      case Element::ElementType::DOUBLE:
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
                       fbb_.EndVector(vector_elements_.size())));
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
      printf("Mismatched type for field '%s'. Got: integer, expected %s\n",
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
      printf("Mismatched type for field '%s'. Got: double, expected %s\n",
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
      printf("Mismatched type for field '%s'. Got: sequence, expected %s\n",
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

::std::vector<uint8_t> JsonToFlatbuffer(
    const char *data, const flatbuffers::TypeTable *typetable) {
  JsonParser p;
  return p.Parse(data, typetable);
}

::std::string FlatbufferToJson(const uint8_t *buffer,
                               const ::flatbuffers::TypeTable *typetable,
                               bool multi_line) {
  ::flatbuffers::ToStringVisitor tostring_visitor(
      multi_line ? "\n" : " ", true, multi_line ? " " : "", multi_line);
  IterateFlatBuffer(buffer, typetable, &tostring_visitor);
  return tostring_visitor.s;
}

void Tokenizer::ConsumeWhitespace() {
  while (true) {
    if (*data_ == '\0') {
      return;
    }
    // Skip any whitespace.
    if (*data_ == ' ' || *data_ == '\r' || *data_ == '\t') {
      ++data_;
    } else if (*data_ == '\n') {
      ++data_;
      ++linenumber_;
    } else {
      // There is no fail.  Once we are out of whitespace (including 0 of it),
      // declare success.
      return;
    }
  }
}

bool Tokenizer::Consume(const char *token) {
  const char *original = data_;
  while (true) {
    // Finishing the token is success.
    if (*token == '\0') {
      return true;
    }

    // But finishing the data first is failure.
    if (*data_ == '\0') {
      data_ = original;
      return false;
    }

    // Missmatch is failure.
    if (*token != *data_) {
      data_ = original;
      return false;
    }

    ++data_;
    ++token;
  }
}

bool Tokenizer::ConsumeString(::std::string *s) {
  // Under no conditions is it acceptible to run out of data while parsing a
  // string.  Any '\0' checks should confirm that.
  const char *original = data_;
  if (*data_ == '\0') {
    return false;
  }

  // Expect the leading "
  if (*data_ != '"') {
    return false;
  }

  ++data_;
  const char *last_parsed_data = data_;
  *s = ::std::string();

  while (true) {
    if (*data_ == '\0') {
      data_ = original;
      return false;
    }

    // If we get an end or an escape, do something special.
    if (*data_ == '"' || *data_ == '\\') {
      // Save what we found up until now, not including this character.
      *s += ::std::string(last_parsed_data, data_);

      // Update the pointer.
      last_parsed_data = data_;

      // " is the end, declare victory.
      if (*data_ == '"') {
        ++data_;
        return true;
      } else {
        ++data_;
        // Now consume valid escape characters and add their representation onto
        // the output string.
        if (*data_ == '\0') {
          data_ = original;
          return false;
        } else if (*data_ == '"') {
          *s += "\"";
        } else if (*data_ == '\\') {
          *s += "\\";
        } else if (*data_ == '/') {
          *s += "/";
        } else if (*data_ == 'b') {
          *s += "\b";
        } else if (*data_ == 'f') {
          *s += "\f";
        } else if (*data_ == 'n') {
          *s += "\n";
        } else if (*data_ == 'r') {
          *s += "\r";
        } else if (*data_ == 't') {
          *s += "\t";
        } else if (*data_ == 'u') {
          // TODO(austin): Unicode should be valid, but I really don't care to
          // do this now...
          fprintf(stderr, "Unexpected unicode on line %d\n", linenumber_);
          data_ = original;
          return false;
        }
      }
      // And skip the escaped character.
      last_parsed_data = data_ + 1;
    }

    ++data_;
  }
}

bool Tokenizer::ConsumeNumber(::std::string *s) {
  // Under no conditions is it acceptible to run out of data while parsing a
  // number.  Any '\0' checks should confirm that.
  *s = ::std::string();
  const char *original = data_;

  // Consume the leading - unconditionally.
  Consume("-");

  // Then, we either get a 0, or we get a nonzero.  Only nonzero can be followed
  // by a second number.
  if (!Consume("0")) {
    if (*data_ == '\0') {
      return false;
    } else if (*data_ >= '1' && *data_ <= '9') {
      // This wasn't a zero, but was a valid digit.  Consume it.
      ++data_;
    } else {
      return false;
    }

    // Now consume any number of any digits.
    while (true) {
      if (*data_ == '\0') {
        data_ = original;
        return false;
      }
      if (*data_ < '0' || *data_ > '9') {
        break;
      }
      ++data_;
    }
  }

  // We could now have a decimal.
  if (*data_ == '.') {
    ++data_;
    while (true) {
      if (*data_ == '\0') {
        data_ = original;
        return false;
      }
      // And any number of digits.
      if (*data_ < '0' || *data_ > '9') {
        break;
      }
      ++data_;
    }
  }

  // And now an exponent.
  if (*data_ == 'e' || *data_ == 'E') {
    ++data_;
    if (*data_ == '\0') {
      data_ = original;
      return false;
    }

    // Which could have a +-
    if (*data_ == '+' || *data_ == '-') {
      ++data_;
    }
    int count = 0;
    while (true) {
      if (*data_ == '\0') {
        data_ = original;
        return false;
      }
      // And digits.
      if (*data_ < '0' || *data_ > '9') {
        break;
      }
      ++data_;
      ++count;
    }
    // But, it is an error to have an exponent and nothing following it.
    if (count == 0) {
      data_ = original;
      return false;
    }
  }

  *s = ::std::string(original, data_);
  return true;
}

Tokenizer::TokenType Tokenizer::Next() {
  switch (state_) {
    case State::kExpectObjectStart:
      // We should always start out with a {
      if (!Consume("{")) return TokenType::kError;

      // Document that we just started an object.
      object_type_.push_back(ObjectType::kObject);

      ConsumeWhitespace();

      if (Consume("}")) {
        ConsumeWhitespace();
        state_ = State::kExpectObjectEnd;
      } else {
        state_ = State::kExpectField;
      }
      return TokenType::kStartObject;

    case State::kExpectField: {
      // Fields are built up of strings, whitespace, and then a : (followed by
      // whitespace...)
      ::std::string s;
      if (!ConsumeString(&s)) {
        fprintf(stderr, "Error on line %d, expected string for field name.\n",
                linenumber_);
        return TokenType::kError;
      }
      field_name_ = ::std::move(s);

      ConsumeWhitespace();

      if (!Consume(":")) {
        fprintf(stderr, "Error on line %d\n", linenumber_);
        return TokenType::kError;
      }

      ConsumeWhitespace();

      state_ = State::kExpectValue;

      return TokenType::kField;
    } break;
    case State::kExpectValue: {
      TokenType result = TokenType::kError;

      ::std::string s;
      if (Consume("{")) {
        // Fields are in objects.  Record and recurse.
        object_type_.push_back(ObjectType::kObject);

        ConsumeWhitespace();

        state_ = State::kExpectField;
        return TokenType::kStartObject;
      } else if (Consume("[")) {
        // Values are in arrays.  Record and recurse.
        object_type_.push_back(ObjectType::kArray);

        ConsumeWhitespace();
        state_ = State::kExpectValue;
        return TokenType::kStartArray;
      } else if (ConsumeString(&s)) {
        // Parsed as a string, grab it.
        field_value_ = ::std::move(s);
        result = TokenType::kStringValue;
      } else if (ConsumeNumber(&s)) {
        // Parsed as a number, grab it.
        field_value_ = ::std::move(s);
        result = TokenType::kNumberValue;
      } else if (Consume("true")) {
        // Parsed as a true, grab it.
        field_value_ = "true";
        result = TokenType::kTrueValue;
      } else if (Consume("false")) {
        // Parsed as a false, grab it.
        field_value_ = "false";
        result = TokenType::kFalseValue;
      } else {
        // Couldn't parse, so we have a syntax error.
        fprintf(stderr, "Error line %d, invalid field value.\n", linenumber_);
      }

      ConsumeWhitespace();

      // After a field, we either have a , and another field (or value if we are
      // in an array), or we should be closing out the object (or array).
      if (Consume(",")) {
        ConsumeWhitespace();
        switch (object_type_.back()) {
          case ObjectType::kObject:
            state_ = State::kExpectField;
            break;
          case ObjectType::kArray:
            state_ = State::kExpectValue;
            break;
        }
      } else {
        // Sanity check that the stack is deep enough.
        if (object_type_.size() == 0) {
          fprintf(stderr, "Error on line %d\n", linenumber_);
          return TokenType::kError;
        }

        // And then require closing out the object.
        switch (object_type_.back()) {
          case ObjectType::kObject:
            if (Consume("}")) {
              ConsumeWhitespace();
              state_ = State::kExpectObjectEnd;
            } else {
              return TokenType::kError;
            }
            break;
          case ObjectType::kArray:
            if (Consume("]")) {
              ConsumeWhitespace();
              state_ = State::kExpectArrayEnd;
            } else {
              return TokenType::kError;
            }
            break;
        }
      }
      return result;
    } break;

    case State::kExpectArrayEnd:
    case State::kExpectObjectEnd: {
      const TokenType result = state_ == State::kExpectArrayEnd
                                   ? TokenType::kEndArray
                                   : TokenType::kEndObject;
      // This is a transient state so we can send 2 tokens out in a row.  We
      // discover the object or array end at the end of reading the value.
      object_type_.pop_back();
      if (object_type_.size() == 0) {
        // We unwound the outer object.  We should send kEnd next.
        state_ = State::kExpectEnd;
      } else if (object_type_.back() == ObjectType::kObject) {
        // If we are going into an object, it should either have another field
        // or end.
        if (Consume(",")) {
          ConsumeWhitespace();
          state_ = State::kExpectField;
        } else if (Consume("}")) {
          ConsumeWhitespace();
          state_ = State::kExpectObjectEnd;
        } else {
          return TokenType::kError;
        }
      } else if (object_type_.back() == ObjectType::kArray) {
        // If we are going into an array, it should either have another value
        // or end.
        if (Consume(",")) {
          ConsumeWhitespace();
          state_ = State::kExpectValue;
        } else if (Consume("]")) {
          ConsumeWhitespace();
          state_ = State::kExpectArrayEnd;
        } else {
          return TokenType::kError;
        }
      }
      // And then send out the correct token.
      return result;
    }
    case State::kExpectEnd:
      // If we are supposed to be done, confirm nothing is after the end.
      if (AtEnd()) {
        return TokenType::kEnd;
      } else {
        fprintf(stderr, "Data past end at line %d\n", linenumber_);
        return TokenType::kError;
      }
  }
  return TokenType::kError;
}

bool Tokenizer::FieldAsInt(long long *value) {
  const char *pos = field_value().c_str();
  errno = 0;
  *value = strtoll(field_value().c_str(), const_cast<char **>(&pos), 10);
  if (pos != field_value().c_str() + field_value().size() || errno != 0) {
    return false;
  }
  return true;
}

bool Tokenizer::FieldAsDouble(double *value) {
  const char *pos = field_value().c_str();
  errno = 0;
  *value = strtod(field_value().c_str(), const_cast<char **>(&pos));

  if (pos != field_value().c_str() + field_value().size() || errno != 0) {
    return false;
  }
  return true;
}

}  // namespace aos
