#include "aos/common/queue_types.h"

#include <inttypes.h>

#include <memory>
#include <unordered_map>

#include "aos/common/byteorder.h"
#include "aos/linux_code/ipc_lib/shared_mem.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/ipc_lib/core_lib.h"

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
      return nullptr;
    }
    *bytes -= sizeof(fields[i]->type) + sizeof(field_name_length);

    to_host(buffer, &fields[i]->type);
    buffer += sizeof(fields[i]->type);
    to_host(buffer, &field_name_length);
    buffer += sizeof(field_name_length);

    if (*bytes < field_name_length) {
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

namespace type_cache {
namespace {

struct CacheEntry {
  const MessageType &type;
  bool in_shm;

  CacheEntry(const MessageType &type) : type(type), in_shm(false) {}
};

struct ShmType {
  uint32_t id;
  volatile ShmType *next;

  size_t serialized_size;
  char serialized[];
};

::std::unordered_map<uint32_t, CacheEntry> cache;

}  // namespace

void Add(const MessageType &type) {
  if (cache.count(type.id) == 0) {
    cache.emplace(type.id, type);
  }
}

const MessageType &Get(uint32_t type_id) {
  if (cache.count(type_id) > 0) {
    return cache.at(type_id).type;
  }

  const volatile ShmType *c = static_cast<volatile ShmType *>(
      global_core->mem_struct->queue_types.pointer);
  while (c != nullptr) {
    if (c->id == type_id) {
      size_t bytes = c->serialized_size;
      MessageType *type = MessageType::Deserialize(
          const_cast<const char *>(c->serialized), &bytes);
      cache.emplace(type_id, *type);
      return *type;
    }
    c = c->next;
  }
  LOG(FATAL, "MessageType for id 0x%" PRIx32 " not found\n", type_id);
}

void AddShm(uint32_t type_id) {
  CacheEntry &cached = cache.at(type_id);
  if (cached.in_shm) return;

  if (mutex_lock(&global_core->mem_struct->queue_types.lock) != 0) {
    LOG(FATAL, "locking queue_types lock failed\n");
  }
  volatile ShmType *current = static_cast<volatile ShmType *>(
      global_core->mem_struct->queue_types.pointer);
  if (current != nullptr) {
    while (true) {
      if (current->id == type_id) {
        cached.in_shm = true;
        mutex_unlock(&global_core->mem_struct->queue_types.lock);
        return;
      }
      if (current->next == nullptr) break;
      current = current->next;
    }
  }
  char buffer[512];
  ssize_t size = cached.type.Serialize(buffer, sizeof(buffer));
  if (size == -1) {
    LOG(FATAL, "type %s is too big to fit into %zd bytes\n",
        cached.type.name, sizeof(buffer));
  }

  volatile ShmType *shm =
      static_cast<volatile ShmType *>(shm_malloc(sizeof(ShmType) + size));
  shm->id = type_id;
  shm->next = nullptr;
  shm->serialized_size = size;
  memcpy(const_cast<char *>(shm->serialized), buffer, size);

  if (current == NULL) {
    global_core->mem_struct->queue_types.pointer = const_cast<ShmType *>(shm);
  } else {
    current->next = shm;
  }
  mutex_unlock(&global_core->mem_struct->queue_types.lock);
}

}  // namespace type_cache
}  // namespace aos
