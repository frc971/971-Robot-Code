#include "frc971/can_logger/can_logger.h"

#include "absl/flags/flag.h"

ABSL_FLAG(bool, poll, false,
          "If true, poll the CAN bus every 100ms.  If false, wake up for "
          "each frame and publish it.");

ABSL_FLAG(int32_t, priority, 10,
          "If --poll is not set, set the realtime priority to this.");
ABSL_FLAG(int32_t, affinity, -1, "If positive, pin to this core.");

namespace frc971::can_logger {

CanLogger::CanLogger(aos::ShmEventLoop *event_loop,
                     std::string_view channel_name,
                     std::string_view interface_name)
    : shm_event_loop_(event_loop),
      fd_(socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW)),
      frames_sender_(shm_event_loop_->MakeSender<CanFrame>(channel_name)) {
  // TODO(max): Figure out a proper priority
  if (!absl::GetFlag(FLAGS_poll)) {
    shm_event_loop_->SetRuntimeRealtimePriority(absl::GetFlag(FLAGS_priority));
  }
  if (absl::GetFlag(FLAGS_affinity) >= 0) {
    shm_event_loop_->SetRuntimeAffinity(
        aos::MakeCpusetFromCpus({absl::GetFlag(FLAGS_affinity)}));
  }
  struct ifreq ifr;
  strcpy(ifr.ifr_name, interface_name.data());
  PCHECK(ioctl(fd_.get(), SIOCGIFINDEX, &ifr) == 0)
      << "Failed to get index for interface " << interface_name;

  int enable_canfd = true;
  PCHECK(setsockopt(fd_.get(), SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                    sizeof(enable_canfd)) == 0)
      << "Failed to enable CAN FD";

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  PCHECK(bind(fd_.get(), reinterpret_cast<struct sockaddr *>(&addr),
              sizeof(addr)) == 0)
      << "Failed to bind socket to interface " << interface_name;

  int recieve_buffer_size;
  socklen_t opt_size = sizeof(recieve_buffer_size);
  PCHECK(getsockopt(fd_.get(), SOL_SOCKET, SO_RCVBUF, &recieve_buffer_size,
                    &opt_size) == 0);
  CHECK_EQ(opt_size, sizeof(recieve_buffer_size));
  VLOG(0) << "CAN recieve bufffer is " << recieve_buffer_size << " bytes large";

  if (absl::GetFlag(FLAGS_poll)) {
    aos::TimerHandler *timer_handler =
        shm_event_loop_->AddTimer([this]() { Poll(); });
    timer_handler->set_name("CAN logging Loop");
    timer_handler->Schedule(event_loop->monotonic_now(), kPollPeriod);
  } else {
    shm_event_loop_->epoll()->OnReadable(fd_.get(), [this]() { Poll(); });
  }
}

void CanLogger::Poll() {
  VLOG(2) << "Polling";
  int frames_read = 0;
  while (ReadFrame()) {
    frames_read++;
  }
  VLOG(1) << "Read " << frames_read << " frames to end of buffer";
}

bool CanLogger::ReadFrame() {
  errno = 0;
  struct canfd_frame frame;
  ssize_t bytes_read = read(fd_.get(), &frame, sizeof(struct canfd_frame));

  if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    // There are no more frames sitting in the recieve buffer.
    return false;
  }

  VLOG(2) << "Read " << bytes_read << " bytes";
  PCHECK(bytes_read > 0);
  PCHECK(bytes_read == static_cast<ssize_t>(CAN_MTU) ||
         bytes_read == static_cast<ssize_t>(CANFD_MTU))
      << "Incomplete can frame";

  struct timeval tv;
  PCHECK(ioctl(fd_.get(), SIOCGSTAMP, &tv) == 0)
      << "Failed to get timestamp of CAN frame";

  aos::Sender<CanFrame>::Builder builder = frames_sender_.MakeBuilder();

  auto frame_data = builder.fbb()->CreateVector<uint8_t>(frame.data, frame.len);

  CanFrame::Builder can_frame_builder = builder.MakeBuilder<CanFrame>();
  can_frame_builder.add_can_id(frame.can_id);
  can_frame_builder.add_flags(frame.flags);
  can_frame_builder.add_data(frame_data);
  can_frame_builder.add_realtime_timestamp_ns(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::seconds(tv.tv_sec) +
          std::chrono::microseconds(tv.tv_usec))
          .count());

  builder.CheckOk(builder.Send(can_frame_builder.Finish()));

  return true;
}

}  // namespace frc971::can_logger
