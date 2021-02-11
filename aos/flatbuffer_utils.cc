#include "aos/flatbuffer_utils.h"

#include "flatbuffers/minireflect.h"
#include "glog/logging.h"

namespace aos {

bool FlatbufferType::IsSequence() const {
  if (type_table_) {
    return type_table_->st != flatbuffers::ST_ENUM;
  }
  LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::IsEnum() const {
  if (type_table_) {
    return type_table_->st == flatbuffers::ST_ENUM;
  }
  LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::FieldIsSequence(int index) const {
  DCHECK(IsSequence());
  if (type_table_) {
    DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    if (type_code.base_type != flatbuffers::ET_SEQUENCE) {
      return false;
    }
    DCHECK(FieldType(index).IsSequence());
    return true;
  }
  LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::FieldIsEnum(int index) const {
  DCHECK(IsSequence());
  if (type_table_) {
    DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    if (type_code.base_type == flatbuffers::ET_SEQUENCE) {
      return false;
    }
    if (type_code.sequence_ref == -1) {
      // Not an enum.
      return false;
    }
    DCHECK(FieldType(index).IsEnum());
    return true;
  }
  LOG(FATAL) << "Unimplemented";
}

std::optional<int64_t> FlatbufferType::EnumValue(std::string_view name) const {
  DCHECK(IsEnum());
  if (type_table_) {
    for (size_t i = 0; i < type_table_->num_elems; ++i) {
      if (name == type_table_->names[i]) {
        if (type_table_->values) {
          return type_table_->values[i];
        } else {
          return i;
        }
      }
    }
    return std::nullopt;
  }
  LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::FieldIsRepeating(int index) const {
  DCHECK(IsSequence());
  if (type_table_) {
    DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    return type_code.is_repeating;
  }
  LOG(FATAL) << "Unimplemented";
}

int FlatbufferType::FieldIndex(std::string_view field_name) const {
  DCHECK(IsSequence());
  if (type_table_) {
    for (size_t i = 0; i < type_table_->num_elems; ++i) {
      if (field_name == std::string_view(type_table_->names[i])) {
        return i;
      }
    }
    return -1;
  }
  LOG(FATAL) << "Unimplemented";
}

std::string_view FlatbufferType::FieldName(int index) const {
  DCHECK(IsSequence());
  if (type_table_) {
    DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    return type_table_->names[index];
  }
  LOG(FATAL) << "Unimplemented";
}

flatbuffers::ElementaryType FlatbufferType::FieldElementaryType(
    int index) const {
  DCHECK(IsSequence());
  if (type_table_) {
    DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    return static_cast<flatbuffers::ElementaryType>(type_code.base_type);
  }
  LOG(FATAL) << "Unimplemented";
}

size_t FlatbufferType::FieldInlineSize(int index) const {
  DCHECK(IsSequence());
  if (type_table_) {
    return flatbuffers::InlineSize(FieldElementaryType(index), type_table_);
  }
  LOG(FATAL) << "Unimplemented";
}

int FlatbufferType::NumberFields() const {
  DCHECK(IsSequence());
  if (type_table_) {
    return type_table_->num_elems;
  }
  LOG(FATAL) << "Unimplemented";
}

FlatbufferType FlatbufferType::FieldType(int index) const {
  DCHECK(IsSequence());
  if (type_table_) {
    DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    CHECK_GE(type_code.sequence_ref, 0);
    // type_refs can be shorter than num_elems, but not longer, so this is still
    // a valid sanity check.
    DCHECK_LT(static_cast<size_t>(type_code.sequence_ref),
              type_table_->num_elems);
    const flatbuffers::TypeFunction type_function =
        type_table_->type_refs[type_code.sequence_ref];
    return FlatbufferType(type_function());
  }
  LOG(FATAL) << "Unimplemented";
}

}  // namespace aos
