// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "NetworkLoopQueue.h"

#include <wpi/Logger.h>

using namespace nt::net;

static constexpr size_t kMaxSize = 2 * 1024 * 1024;

void NetworkLoopQueue::SetValue(NT_Publisher pubHandle, const Value& value) {
  std::scoped_lock lock{m_mutex};
  m_size += sizeof(ClientMessage) + value.size();
  if (m_size > kMaxSize) {
    if (!m_sizeErrored) {
      WPI_ERROR(m_logger, "NT: dropping value set due to memory limits");
      m_sizeErrored = true;
    }
    return;  // avoid potential out of memory
  }
  m_queue.emplace_back(ClientMessage{ClientValueMsg{pubHandle, value}});
}
