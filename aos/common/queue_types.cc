#include "aos/common/queue_types.h"

#include <errno.h>

#include <memory>

#include "aos/common/byteorder.h"

namespace aos {

ssize_t MessageType::Serialize(char *buffer, size_t max_bytes) const {
  char *const buffer_start = buffer;
  size_t name_length = strlen(name);
  ::std::unique_ptr<size_t> field_name_lengths(new size_t[number_fields]);
  size_t fields_size = 0;
  for (int i = 0; i < number_fields; ++i) {
    fields_size += sizeof(fields[i]->type);
    field_name_lengths.get()[i] = strlen(fields[i]->name);
    fields_size += field_name_lengths.get()[i];
  }
  if (max_bytes < sizeof(id) + sizeof(name_length) + sizeof(number_fields) +
                      name_length + fields_size) {
    errno = EOVERFLOW;
    return -1;
  }
  to_network(&id, buffer);
  buffer += sizeof(id);
  to_network(&name_length, buffer);
  buffer += sizeof(name_length);
  to_network(&number_fields, buffer);
  buffer += sizeof(number_fields);
  memcpy(buffer, name, name_length);
  buffer += name_length;

  for (int i = 0; i < number_fields; ++i) {
    to_network(&fields[i]->type, buffer);
    buffer += sizeof(fields[i]->type);
    to_network(&field_name_lengths.get()[i], buffer);
    buffer += sizeof(field_name_lengths.get()[i]);
    memcpy(buffer, fields[i]->name, field_name_lengths.get()[i]);
    buffer += field_name_lengths.get()[i];
  }

  return buffer - buffer_start;
}

MessageType *MessageType::Deserialize(const char *buffer, size_t *bytes) {
  size_t name_length;
  decltype(MessageType::id) id;
  decltype(MessageType::number_fields) number_fields;
  if (*bytes < sizeof(id) + sizeof(name_length) + sizeof(number_fields)) {
    errno = EOVERFLOW;
    return nullptr;
  }
  *bytes -= sizeof(id) + sizeof(name_length) + sizeof(number_fields);

  to_host(buffer, &id);
  buffer += sizeof(id);
  to_host(buffer, &name_length);
  buffer += sizeof(name_length);
  to_host(buffer, &number_fields);
  buffer += sizeof(number_fields);

  if (*bytes < name_length) {
    errno = EOVERFLOW;
    return nullptr;
  }
  *bytes -= name_length;

  Field **fields = new Field *[number_fields];
  ::std::unique_ptr<MessageType> r(new MessageType(number_fields, fields));
  r->id = id;
  ::std::unique_ptr<char> name(new char[name_length + 1]);
  memcpy(name.get(), buffer, name_length);
  buffer += name_length;
  name.get()[name_length] = '\0';
  r->name = name.release();

  for (int i = 0; i < number_fields; ++i) {
    size_t field_name_length;
    if (*bytes < sizeof(fields[i]->type) + sizeof(field_name_length)) {
      errno = EOVERFLOW;
      return nullptr;
    }
    *bytes -= sizeof(fields[i]->type) + sizeof(field_name_length);

    to_host(buffer, &fields[i]->type);
    buffer += sizeof(fields[i]->type);
    to_host(buffer, &field_name_length);
    buffer += sizeof(field_name_length);

    if (*bytes < field_name_length) {
      errno = EOVERFLOW;
      return nullptr;
    }
    *bytes -= field_name_length;
    ::std::unique_ptr<char> field_name(new char[field_name_length + 1]);
    memcpy(field_name.get(), buffer, field_name_length);
    buffer += field_name_length;
    field_name.get()[field_name_length] = '\0';
    fields[i]->name = field_name.release();
  }

  return r.release();
}

}  // namespace aos
