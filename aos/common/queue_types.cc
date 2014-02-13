#include "aos/common/queue_types.h"

#include <inttypes.h>

#include <memory>
#include <unordered_map>

#include "aos/common/byteorder.h"
#include "aos/linux_code/ipc_lib/shared_mem.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/ipc_lib/core_lib.h"
#include "aos/common/mutex.h"

namespace aos {

ssize_t MessageType::Serialize(char *buffer, size_t max_bytes) const {
  char *const buffer_start = buffer;
  size_t fields_size = 0;
  for (int i = 0; i < number_fields; ++i) {
    fields_size += sizeof(fields[i]->type);
    fields_size += sizeof(size_t);
    fields_size += fields[i]->name.size();
  }
  if (max_bytes < sizeof(id) + sizeof(super_size) + sizeof(size_t) +
                      sizeof(number_fields) + name.size() + fields_size) {
    return -1;
  }

  size_t length;

  to_network(&super_size, buffer);
  buffer += sizeof(super_size);
  to_network(&id, buffer);
  buffer += sizeof(id);
  length = name.size();
  to_network(&length, buffer);
  buffer += sizeof(length);
  to_network(&number_fields, buffer);
  buffer += sizeof(number_fields);
  memcpy(buffer, name.data(), name.size());
  buffer += name.size();

  for (int i = 0; i < number_fields; ++i) {
    to_network(&fields[i]->type, buffer);
    buffer += sizeof(fields[i]->type);
    length = fields[i]->name.size();
    to_network(&length, buffer);
    buffer += sizeof(length);
    memcpy(buffer, fields[i]->name.data(), fields[i]->name.size());
    buffer += length;
  }

  return buffer - buffer_start;
}

MessageType *MessageType::Deserialize(const char *buffer, size_t *bytes) {
  size_t name_length;
  decltype(MessageType::super_size) super_size;
  decltype(MessageType::id) id;
  decltype(MessageType::number_fields) number_fields;
  if (*bytes < sizeof(super_size) + sizeof(id) + sizeof(name_length) +
                   sizeof(number_fields)) {
    return nullptr;
  }
  *bytes -= sizeof(super_size) + sizeof(id) + sizeof(name_length) +
            sizeof(number_fields);

  to_host(buffer, &super_size);
  buffer += sizeof(super_size);
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
  ::std::unique_ptr<MessageType> r(
      new MessageType(super_size, id, ::std::string(buffer, name_length),
                      number_fields, fields));
  buffer += name_length;

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
    fields[i]->name = ::std::string(buffer, field_name_length);
    buffer += field_name_length;
  }

  return r.release();
}

bool PrintMessage(char *output, size_t *output_bytes, void *input,
                  size_t *input_bytes, const MessageType &type) {
  *input_bytes -= type.super_size;
  input = static_cast<char *>(input) + type.super_size;

  if (*output_bytes < type.name.size() + 1) return false;
  *output_bytes -= type.name.size() + 1;
  memcpy(output, type.name.data(), type.name.size());
  output += type.name.size();
  *(output++) = '{';

  bool first = true;
  for (int i = 0; i < type.number_fields; ++i) {
    if (first) {
      first = false;
    } else {
      if (*output_bytes < 1) return false;
      *output_bytes -= 1;
      *(output++) = ',';
    }

    if (*output_bytes < type.fields[i]->name.size() + 1) return false;
    *output_bytes -= type.fields[i]->name.size() + 1;
    memcpy(output, type.fields[i]->name.data(), type.fields[i]->name.size());
    output += type.fields[i]->name.size();
    *(output++) = ':';

    const size_t output_bytes_before = *output_bytes,
                 input_bytes_before = *input_bytes;
    if (MessageType::IsPrimitive(type.fields[i]->type)) {
      if (!PrintField(output, output_bytes, input, input_bytes,
                      type.fields[i]->type)) {
        return false;
      }
    } else {
      if (!PrintMessage(output, output_bytes, input, input_bytes,
                        type_cache::Get(type.fields[i]->type))) {
        return false;
      }
    }

    // Update the input and output pointers, ignoring the trailing '\0' that the
    // subcall put on.
    output += output_bytes_before - *output_bytes - 1;
    *output_bytes += 1;
    input = static_cast<char *>(input) + input_bytes_before - *input_bytes;
  }
  if (*output_bytes < 2) return false;
  *output_bytes -= 2;
  *(output++) = '}';
  *(output++) = '\0';
  return true;
}

namespace type_cache {
namespace {

struct CacheEntry {
  const MessageType &type;
  bool in_shm;

  CacheEntry(const MessageType &type, bool in_shm)
      : type(type), in_shm(in_shm) {}
};

struct ShmType {
  uint32_t id;
  volatile ShmType *next;

  size_t serialized_size;
  char serialized[];
};

::std::unordered_map<uint32_t, CacheEntry> cache;
::aos::Mutex cache_lock;

}  // namespace

void Add(const MessageType &type) {
  ::aos::MutexLocker locker(&cache_lock);
  if (cache.count(type.id) == 0) {
    cache.emplace(::std::piecewise_construct, ::std::forward_as_tuple(type.id),
                  ::std::forward_as_tuple(type, false));
  }
}

const MessageType &Get(uint32_t type_id) {
  ::aos::MutexLocker locker(&cache_lock);
  if (cache.count(type_id) > 0) {
    return cache.at(type_id).type;
  }

  // No need to lock because the only thing that happens is somebody adds on to
  // the end, and they shouldn't be adding the one we're looking for.
  const volatile ShmType *c = static_cast<volatile ShmType *>(
      global_core->mem_struct->queue_types.pointer);
  while (c != nullptr) {
    if (c->id == type_id) {
      size_t bytes = c->serialized_size;
      MessageType *type = MessageType::Deserialize(
          const_cast<const char *>(c->serialized), &bytes);
      cache.emplace(::std::piecewise_construct,
                    ::std::forward_as_tuple(type_id),
                    ::std::forward_as_tuple(*type, true));
      return *type;
    }
    c = c->next;
  }
  LOG(FATAL, "MessageType for id 0x%" PRIx32 " not found\n", type_id);
}

void AddShm(uint32_t type_id) {
  ::aos::MutexLocker locker(&cache_lock);
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
        cached.type.name.c_str(), sizeof(buffer));
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

  for (int i = 0; i < cached.type.number_fields; ++i) {
    if (!MessageType::IsPrimitive(cached.type.fields[i]->type)) {
      AddShm(cached.type.fields[i]->type);
    }
  }
}

}  // namespace type_cache
}  // namespace aos
