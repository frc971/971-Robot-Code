#include <iostream>
#include <sstream>

#include "aos/json_to_flatbuffer.h"

namespace aos {

namespace {

using reflection::BaseType;

void IntToString(int64_t val, reflection::BaseType type,
                 std::stringstream *out) {
  switch (type) {
    case BaseType::Bool:
      *out << (val ? "true" : "false");
      break;
    case BaseType::UByte:
      *out << std::to_string(static_cast<uint8_t>(val));
      break;
    case BaseType::Byte:
      *out << std::to_string(static_cast<int8_t>(val));
      break;
    case BaseType::Short:
      *out << static_cast<int16_t>(val);
      break;
    case BaseType::UShort:
      *out << static_cast<uint16_t>(val);
      break;
    case BaseType::Int:
      *out << static_cast<int32_t>(val);
      break;
    case BaseType::UInt:
      *out << static_cast<uint32_t>(val);
      break;
    case BaseType::Long:
      *out << static_cast<int64_t>(val);
      break;
    case BaseType::ULong:
      *out << static_cast<uint64_t>(val);
      break;
    default:
      *out << "null";
  }
}

void FloatToString(double val, reflection::BaseType type,
                   std::stringstream *out) {
  switch (type) {
    case BaseType::Float:
      out->precision(std::numeric_limits<float>::digits10);
      *out << static_cast<float>(val);
      break;
    case BaseType::Double:
      out->precision(std::numeric_limits<double>::digits10);
      *out << val;
      break;
    default:
      *out << "null";
  }
}

template <typename ObjT>
void ObjectToString(
    const reflection::Object *obj,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Object>> *objects,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    const ObjT *object, std::stringstream *out);

// Get enum value name
const char *EnumToString(
    int64_t enum_value,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::EnumVal>>
        *values) {
  // Replace with binary search? Enum values are pre-sorted.
  for (auto iter = values->begin(); iter != values->end(); iter++) {
    if (enum_value == iter->value()) {
      return iter->name()->c_str();
    }
  }
  return nullptr;
}

// Convert integer to string, checking if it is an enum.
void IntOrEnumToString(
    int64_t val, const reflection::Type *type,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    std::stringstream *out) {
  // Check if integer is an enum and print string, otherwise fallback to
  // printing as int.
  if (type->index() > -1 && type->index() < (int32_t)enums->size()) {
    const reflection::Enum *enum_props = enums->Get(type->index());
    if (!enum_props->is_union()) {
      const char *value_string = EnumToString(val, enum_props->values());

      if (value_string != nullptr) {
        *out << '"' << value_string << '"';
      }
    }
  } else {
    if (type->base_type() == BaseType::Vector ||
        type->base_type() == BaseType::Array) {
      IntToString(val, type->element(), out);
    } else {
      IntToString(val, type->base_type(), out);
    }
  }
}

// Print field in flatbuffer table. Field must be populated.
template <typename ObjT>
void FieldToString(
    const ObjT *table, const reflection::Field *field,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Object>> *objects,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    std::stringstream *out) {
  const reflection::Type *type = field->type();

  switch (type->base_type()) {
    case BaseType::Bool:
    case BaseType::UByte:
    case BaseType::Byte:
    case BaseType::Short:
    case BaseType::UShort:
    case BaseType::Int:
    case BaseType::UInt:
    case BaseType::Long:
    case BaseType::ULong:
      IntOrEnumToString(GetAnyFieldI(*table, *field), type, enums, out);
      break;
    case BaseType::Float:
    case BaseType::Double:
      FloatToString(GetAnyFieldF(*table, *field), type->base_type(), out);
      break;
    case BaseType::String:
      if constexpr (std::is_same<flatbuffers::Table, ObjT>()) {
        std::string str = flatbuffers::GetFieldS(*table, *field)->str();
        std::string out_str;
        out_str.reserve(str.size());
        for (char c : str) {
          // out_str += c;
          switch (c) {
            case '"':
              out_str += "\\\"";
              break;
            case '\\':
              out_str += "\\\\";
              break;
            case '\b':
              out_str += "\\b";
              break;
            case '\f':
              out_str += "\\f";
              break;
            case '\n':
              out_str += "\\n";
              break;
            case '\r':
              out_str += "\\r";
              break;
            case '\t':
              out_str += "\\t";
              break;
            default:
              out_str += c;
          }
        }
        *out << '"' << out_str << '"';
      } else {
        *out << "null";
      }
      break;
    case BaseType::Vector: {
      if constexpr (std::is_same<flatbuffers::Table, ObjT>()) {
        const flatbuffers::VectorOfAny *vector =
            flatbuffers::GetFieldAnyV(*table, *field);
        reflection::BaseType elem_type = type->element();

        *out << '[';
        for (flatbuffers::uoffset_t i = 0; i < vector->size(); ++i) {
          if (i != 0) {
            *out << ", ";
          }
          if (flatbuffers::IsInteger(elem_type)) {
            IntOrEnumToString(
                flatbuffers::GetAnyVectorElemI(vector, elem_type, i), type,
                enums, out);
          } else if (flatbuffers::IsFloat(elem_type)) {
            FloatToString(flatbuffers::GetAnyVectorElemF(vector, elem_type, i),
                          elem_type, out);
          } else if (elem_type == BaseType::String) {
            *out << '"' << flatbuffers::GetAnyVectorElemS(vector, elem_type, i)
                 << '"';
          } else if (elem_type == BaseType::Obj) {
            if (type->index() > -1 &&
                type->index() < (int32_t)objects->size()) {
              if (objects->Get(type->index())->is_struct()) {
                ObjectToString(
                    objects->Get(type->index()), objects, enums,
                    flatbuffers::GetAnyVectorElemAddressOf<
                        const flatbuffers::Struct>(
                        vector, i, objects->Get(type->index())->bytesize()),
                    out);
              } else {
                ObjectToString(objects->Get(type->index()), objects, enums,
                               flatbuffers::GetAnyVectorElemPointer<
                                   const flatbuffers::Table>(vector, i),
                               out);
              }
            }
          }
        }
        *out << ']';
      } else {
        *out << "null";
      }
    } break;
    case BaseType::Obj: {
      if (type->index() > -1 && type->index() < (int32_t)objects->size()) {
        if (objects->Get(type->index())->is_struct()) {
          ObjectToString(objects->Get(type->index()), objects, enums,
                         flatbuffers::GetFieldStruct(*table, *field), out);
        } else if constexpr (std::is_same<flatbuffers::Table, ObjT>()) {
          ObjectToString(objects->Get(type->index()), objects, enums,
                         flatbuffers::GetFieldT(*table, *field), out);
        }
      } else {
        *out << "null";
      }
    } break;
    default:
      *out << "null";
  }
}

// Prints flatbuffer table or struct given list of possible child objects and
// enums. Prints "null" if the child object type is not found.
template <typename ObjT>
void ObjectToString(
    const reflection::Object *obj,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Object>> *objects,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    const ObjT *object, std::stringstream *out) {
  static_assert(std::is_same<flatbuffers::Table, ObjT>() ||
                    std::is_same<flatbuffers::Struct, ObjT>(),
                "Type must be either flatbuffer table or struct");
  bool print_sep = false;
  *out << '{';
  for (const reflection::Field *field : *obj->fields()) {
    // Check whether this object has the field populated (even for structs,
    // which should have all fields populated)
    if (object->GetAddressOf(field->offset())) {
      if (print_sep) {
        *out << ", ";
      } else {
        print_sep = true;
      }
      *out << '"' << field->name()->c_str() << "\": ";
      FieldToString(object, field, objects, enums, out);
    }
  }
  *out << '}';
}

}  // namespace

std::string FlatbufferToJson(const reflection::Schema *schema,
                             const uint8_t *data) {
  const flatbuffers::Table *table = flatbuffers::GetAnyRoot(data);

  const reflection::Object *obj = schema->root_table();

  std::stringstream out;

  ObjectToString(obj, schema->objects(), schema->enums(), table, &out);

  return out.str();
}
}  // namespace aos
