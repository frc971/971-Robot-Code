#include "starterd_lib.h"

#include <fcntl.h>
#include <grp.h>
#include <pwd.h>
#include <sys/fsuid.h>
#include <sys/prctl.h>

#include <algorithm>
#include <utility>

#include "absl/strings/str_format.h"
#include "aos/json_to_flatbuffer.h"
#include "glog/logging.h"
#include "glog/stl_logging.h"

namespace aos {
namespace starter {

Application::Application(const aos::Application *application,
                         aos::ShmEventLoop *event_loop,
                         std::function<void()> on_change)
    : name_(application->name()->string_view()),
      path_(application->has_executable_name()
                ? application->executable_name()->string_view()
                : application->name()->string_view()),
      args_(1),
      user_name_(application->has_user() ? application->user()->str() : ""),
      user_(application->has_user() ? FindUid(user_name_.c_str())
                                    : std::nullopt),
      group_(application->has_user() ? FindPrimaryGidForUser(user_name_.c_str())
                                     : std::nullopt),
      autostart_(application->autostart()),
      autorestart_(application->autorestart()),
      event_loop_(event_loop),
      start_timer_(event_loop_->AddTimer([this] {
        status_ = aos::starter::State::RUNNING;
        LOG(INFO) << "Started '" << name_ << "' pid: " << pid_;
      })),
      restart_timer_(event_loop_->AddTimer([this] { DoStart(); })),
      stop_timer_(event_loop_->AddTimer([this] {
        if (kill(pid_, SIGKILL) == 0) {
          LOG(WARNING) << "Failed to stop, sending SIGKILL to '" << name_
                       << "' pid: " << pid_;
        }
      })),
      on_change_(on_change) {}

void Application::DoStart() {
  if (status_ != aos::starter::State::WAITING) {
    return;
  }

  start_timer_->Disable();
  restart_timer_->Disable();

  std::tie(read_pipe_, write_pipe_) = ScopedPipe::MakePipe();

  const pid_t pid = fork();

  if (pid != 0) {
    if (pid == -1) {
      PLOG(WARNING) << "Failed to fork '" << name_ << "'";
      stop_reason_ = aos::starter::LastStopReason::FORK_ERR;
      status_ = aos::starter::State::STOPPED;
    } else {
      pid_ = pid;
      id_ = next_id_++;
      start_time_ = event_loop_->monotonic_now();
      status_ = aos::starter::State::STARTING;
      LOG(INFO) << "Starting '" << name_ << "' pid " << pid_;

      // Setup timer which moves application to RUNNING state if it is still
      // alive in 1 second.
      start_timer_->Setup(event_loop_->monotonic_now() +
                          std::chrono::seconds(1));
    }
    on_change_();
    return;
  }

  // Clear out signal mask of parent so forked process receives all signals
  // normally.
  sigset_t empty_mask;
  sigemptyset(&empty_mask);
  sigprocmask(SIG_SETMASK, &empty_mask, nullptr);

  // Cleanup children if starter dies in a way that is not handled gracefully.
  if (prctl(PR_SET_PDEATHSIG, SIGKILL) == -1) {
    write_pipe_.Write(
        static_cast<uint32_t>(aos::starter::LastStopReason::SET_PRCTL_ERR));
    PLOG(FATAL) << "Could not set PR_SET_PDEATHSIG to SIGKILL";
  }

  if (group_) {
    CHECK(!user_name_.empty());
    // The manpage for setgroups says we just need CAP_SETGID, but empirically
    // we also need the effective UID to be 0 to make it work. user_ must also
    // be set so we change this effective UID back later.
    CHECK(user_);
    if (seteuid(0) == -1) {
      write_pipe_.Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_GRP_ERR));
      PLOG(FATAL) << "Could not seteuid(0) for " << name_
                  << " in preparation for setting groups";
    }
    if (initgroups(user_name_.c_str(), *group_) == -1) {
      write_pipe_.Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_GRP_ERR));
      PLOG(FATAL) << "Could not initialize normal groups for " << name_
                  << " as " << user_name_ << " with " << *group_;
    }
    if (setgid(*group_) == -1) {
      write_pipe_.Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_GRP_ERR));
      PLOG(FATAL) << "Could not set group for " << name_ << " to " << *group_;
    }
  }

  if (user_) {
    if (setuid(*user_) == -1) {
      write_pipe_.Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_USR_ERR));
      PLOG(FATAL) << "Could not set user for " << name_ << " to " << *user_;
    }
  }

  // argv[0] should be the program name
  args_.insert(args_.begin(), path_.data());

  execvp(path_.c_str(), args_.data());

  // If we got here, something went wrong
  write_pipe_.Write(
      static_cast<uint32_t>(aos::starter::LastStopReason::EXECV_ERR));
  PLOG(WARNING) << "Could not execute " << name_ << " (" << path_ << ')';

  _exit(EXIT_FAILURE);
}

void Application::DoStop(bool restart) {
  // If stop or restart received, the old state of these is no longer applicable
  // so cancel both.
  restart_timer_->Disable();
  start_timer_->Disable();

  switch (status_) {
    case aos::starter::State::STARTING:
    case aos::starter::State::RUNNING: {
      LOG(INFO) << "Stopping '" << name_ << "' pid: " << pid_ << " with signal "
                << SIGINT;
      status_ = aos::starter::State::STOPPING;

      kill(pid_, SIGINT);

      // Watchdog timer to SIGKILL application if it is still running 1 second
      // after SIGINT
      stop_timer_->Setup(event_loop_->monotonic_now() +
                         std::chrono::seconds(1));
      queue_restart_ = restart;
      on_change_();
      break;
    }
    case aos::starter::State::WAITING: {
      // If waiting to restart, and receives restart, skip the waiting period
      // and restart immediately. If stop received, all we have to do is move
      // to the STOPPED state.
      if (restart) {
        DoStart();
      } else {
        status_ = aos::starter::State::STOPPED;
        on_change_();
      }
      break;
    }
    case aos::starter::State::STOPPING: {
      // If the application is already stopping, then we just need to update the
      // restart flag to the most recent status.
      queue_restart_ = restart;
      break;
    }
    case aos::starter::State::STOPPED: {
      // Restart immediately if the application is already stopped
      if (restart) {
        status_ = aos::starter::State::WAITING;
        DoStart();
      }
      break;
    }
  }
}

void Application::QueueStart() {
  status_ = aos::starter::State::WAITING;

  LOG(INFO) << "Restarting " << name_ << " in 3 seconds";
  restart_timer_->Setup(event_loop_->monotonic_now() + std::chrono::seconds(3));
  start_timer_->Disable();
  stop_timer_->Disable();
  on_change_();
}

void Application::set_args(
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> &v) {
  args_.clear();
  std::transform(v.begin(), v.end(), std::back_inserter(args_),
                 [](const flatbuffers::String *str) {
                   return const_cast<char *>(str->c_str());
                 });
  args_.push_back(nullptr);
}

std::optional<uid_t> Application::FindUid(const char *name) {
  // TODO(austin): Use the reentrant version.  This should be safe.
  struct passwd *user_data = getpwnam(name);
  if (user_data != nullptr) {
    return user_data->pw_uid;
  } else {
    LOG(FATAL) << "Could not find user " << name;
    return std::nullopt;
  }
}

std::optional<gid_t> Application::FindPrimaryGidForUser(const char *name) {
  // TODO(austin): Use the reentrant version.  This should be safe.
  struct passwd *user_data = getpwnam(name);
  if (user_data != nullptr) {
    return user_data->pw_gid;
  } else {
    LOG(FATAL) << "Could not find user " << name;
    return std::nullopt;
  }
}

flatbuffers::Offset<aos::starter::ApplicationStatus>
Application::PopulateStatus(flatbuffers::FlatBufferBuilder *builder) {
  CHECK_NOTNULL(builder);
  auto name_fbs = builder->CreateString(name_);

  aos::starter::ApplicationStatus::Builder status_builder(*builder);
  status_builder.add_name(name_fbs);
  status_builder.add_state(status_);
  status_builder.add_last_exit_code(exit_code_);
  status_builder.add_last_stop_reason(stop_reason_);
  if (pid_ != -1) {
    status_builder.add_pid(pid_);
    status_builder.add_id(id_);
  }
  status_builder.add_last_start_time(start_time_.time_since_epoch().count());
  return status_builder.Finish();
}

void Application::Terminate() {
  stop_reason_ = aos::starter::LastStopReason::TERMINATE;
  DoStop(false);
  terminating_ = true;
}

void Application::HandleCommand(aos::starter::Command cmd) {
  switch (cmd) {
    case aos::starter::Command::START: {
      switch (status_) {
        case aos::starter::State::WAITING: {
          restart_timer_->Disable();
          DoStart();
          break;
        }
        case aos::starter::State::STARTING: {
          break;
        }
        case aos::starter::State::RUNNING: {
          break;
        }
        case aos::starter::State::STOPPING: {
          queue_restart_ = true;
          break;
        }
        case aos::starter::State::STOPPED: {
          status_ = aos::starter::State::WAITING;
          DoStart();
          break;
        }
      }
      break;
    }
    case aos::starter::Command::STOP: {
      stop_reason_ = aos::starter::LastStopReason::STOP_REQUESTED;
      DoStop(false);
      break;
    }
    case aos::starter::Command::RESTART: {
      stop_reason_ = aos::starter::LastStopReason::RESTART_REQUESTED;
      DoStop(true);
      break;
    }
  }
}

bool Application::MaybeHandleSignal() {
  int status;

  // Check if the status of this process has changed
  if (pid_ == -1 || waitpid(pid_, &status, WNOHANG) != pid_) {
    return false;
  }

  // Check that the event was the process exiting
  if (!WIFEXITED(status) && !WIFSIGNALED(status)) {
    return false;
  }

  exit_time_ = event_loop_->monotonic_now();
  exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : WTERMSIG(status);

  if (auto read_result = read_pipe_.Read()) {
    stop_reason_ = static_cast<aos::starter::LastStopReason>(*read_result);
  }

  switch (status_) {
    case aos::starter::State::STARTING: {
      LOG(WARNING) << "Failed to start '" << name_ << "' on pid " << pid_
                   << " : Exited with status " << exit_code_;
      if (autorestart()) {
        QueueStart();
      }
      break;
    }
    case aos::starter::State::RUNNING: {
      if (exit_code_ == 0) {
        LOG(INFO) << "Application '" << name_ << "' pid " << pid_
                  << " exited with status " << exit_code_;
      } else {
        LOG(WARNING) << "Application '" << name_ << "' pid " << pid_
                     << " exited unexpectedly with status " << exit_code_;
      }
      if (autorestart()) {
        QueueStart();
      }
      break;
    }
    case aos::starter::State::STOPPING: {
      LOG(INFO) << "Successfully stopped '" << name_ << "' pid: " << pid_
                << " with status " << exit_code_;
      status_ = aos::starter::State::STOPPED;

      // Disable force stop timer since the process already died
      stop_timer_->Disable();

      on_change_();
      if (terminating_) {
        return true;
      }

      if (queue_restart_) {
        queue_restart_ = false;
        status_ = aos::starter::State::WAITING;
        DoStart();
      }
      break;
    }
    case aos::starter::State::WAITING:
    case aos::starter::State::STOPPED: {
      LOG(FATAL)
          << "Received signal on process that was already stopped : name: '"
          << name_ << "' pid: " << pid_;
      break;
    }
  }

  return false;
}

ScopedPipe::ScopedPipe(int fd) : fd_(fd) {}

ScopedPipe::~ScopedPipe() {
  if (fd_ != -1) {
    PCHECK(close(fd_) != -1);
  }
}

ScopedPipe::ScopedPipe(ScopedPipe &&scoped_pipe) : fd_(scoped_pipe.fd_) {
  scoped_pipe.fd_ = -1;
}

ScopedPipe &ScopedPipe::operator=(ScopedPipe &&scoped_pipe) {
  if (fd_ != -1) {
    PCHECK(close(fd_) != -1);
  }
  fd_ = scoped_pipe.fd_;
  scoped_pipe.fd_ = -1;
  return *this;
}

std::tuple<ScopedPipe::ScopedReadPipe, ScopedPipe::ScopedWritePipe>
ScopedPipe::MakePipe() {
  int fds[2];
  PCHECK(pipe(fds) != -1);
  PCHECK(fcntl(fds[0], F_SETFL, fcntl(fds[0], F_GETFL) | O_NONBLOCK) != -1);
  PCHECK(fcntl(fds[1], F_SETFL, fcntl(fds[1], F_GETFL) | O_NONBLOCK) != -1);
  return {ScopedReadPipe(fds[0]), ScopedWritePipe(fds[1])};
}

std::optional<uint32_t> ScopedPipe::ScopedReadPipe::Read() {
  uint32_t buf;
  ssize_t result = read(fd(), &buf, sizeof(buf));
  if (result == sizeof(buf)) {
    return buf;
  } else {
    return std::nullopt;
  }
}

void ScopedPipe::ScopedWritePipe::Write(uint32_t data) {
  ssize_t result = write(fd(), &data, sizeof(data));
  PCHECK(result != -1);
  CHECK(result == sizeof(data));
}

SignalListener::SignalListener(aos::ShmEventLoop *loop,
                               std::function<void(signalfd_siginfo)> callback)
    : loop_(loop),
      callback_(std::move(callback)),
      signalfd_({SIGHUP, SIGINT, SIGQUIT, SIGABRT, SIGFPE, SIGSEGV, SIGPIPE,
                 SIGTERM, SIGBUS, SIGXCPU, SIGCHLD}) {
  loop->epoll()->OnReadable(signalfd_.fd(), [this] {
    signalfd_siginfo info = signalfd_.Read();

    if (info.ssi_signo == 0) {
      LOG(WARNING) << "Could not read " << sizeof(signalfd_siginfo) << " bytes";
      return;
    }

    callback_(info);
  });
}

SignalListener::~SignalListener() { loop_->epoll()->DeleteFd(signalfd_.fd()); }

const aos::Channel *StatusChannelForNode(const aos::Configuration *config,
                                         const aos::Node *node) {
  return configuration::GetChannel<Status>(config, "/aos", "", node);
}
const aos::Channel *StarterRpcChannelForNode(const aos::Configuration *config,
                                             const aos::Node *node) {
  return configuration::GetChannel<StarterRpc>(config, "/aos", "", node);
}

Starter::Starter(const aos::Configuration *event_loop_config)
    : config_msg_(event_loop_config),
      event_loop_(event_loop_config),
      status_sender_(event_loop_.MakeSender<aos::starter::Status>("/aos")),
      status_timer_(event_loop_.AddTimer([this] {
        SendStatus();
        status_count_ = 0;
      })),
      cleanup_timer_(event_loop_.AddTimer([this] { event_loop_.Exit(); })),
      max_status_count_(
          event_loop_.GetChannel<aos::starter::Status>("/aos")->frequency() -
          1),
      listener_(&event_loop_,
                [this](signalfd_siginfo signal) { OnSignal(signal); }) {
  event_loop_.SkipAosLog();

  event_loop_.OnRun([this] {
    status_timer_->Setup(event_loop_.monotonic_now(),
                         std::chrono::milliseconds(1000));
  });

  if (!aos::configuration::MultiNode(config_msg_)) {
    event_loop_.MakeWatcher(
        "/aos",
        [this](const aos::starter::StarterRpc &cmd) { HandleStarterRpc(cmd); });
  } else {
    for (const aos::Node *node : aos::configuration::GetNodes(config_msg_)) {
      const Channel *channel = StarterRpcChannelForNode(config_msg_, node);
      CHECK(channel != nullptr) << ": Failed to find channel /aos for "
                                << StarterRpc::GetFullyQualifiedName() << " on "
                                << node->name()->string_view();
      if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                       event_loop_.node())) {
        LOG(INFO) << "StarterRpc channel "
                  << aos::configuration::StrippedChannelToString(channel)
                  << " is not readable on "
                  << event_loop_.node()->name()->string_view();
      } else {
        event_loop_.MakeWatcher(channel->name()->string_view(),
                                [this](const aos::starter::StarterRpc &cmd) {
                                  HandleStarterRpc(cmd);
                                });
      }
    }
  }

  if (config_msg_->has_applications()) {
    const flatbuffers::Vector<flatbuffers::Offset<aos::Application>>
        *applications = config_msg_->applications();

    if (aos::configuration::MultiNode(config_msg_)) {
      std::string_view current_node = event_loop_.node()->name()->string_view();
      for (const aos::Application *application : *applications) {
        CHECK(application->has_nodes());
        for (const flatbuffers::String *node : *application->nodes()) {
          if (node->string_view() == current_node) {
            AddApplication(application);
            break;
          }
        }
      }
    } else {
      for (const aos::Application *application : *applications) {
        AddApplication(application);
      }
    }
  }
}

void Starter::HandleStarterRpc(const StarterRpc &command) {
  if (!command.has_command() || !command.has_name() || exiting_) {
    return;
  }

  LOG(INFO) << "Received " << aos::FlatbufferToJson(&command);

  if (command.has_nodes()) {
    CHECK(aos::configuration::MultiNode(config_msg_));
    bool relevant_to_this_node = false;
    for (const flatbuffers::String *node : *command.nodes()) {
      if (node->string_view() == event_loop_.node()->name()->string_view()) {
        relevant_to_this_node = true;
      }
    }
    if (!relevant_to_this_node) {
      return;
    }
  }
  // If not populated, restart regardless of node.

  auto search = applications_.find(command.name()->str());
  if (search != applications_.end()) {
    // If an applicatione exists by the given name, dispatch the command
    search->second.HandleCommand(command.command());
  }
}

void Starter::MaybeSendStatus() {
  if (status_count_ < max_status_count_) {
    SendStatus();
    ++status_count_;
  } else {
    VLOG(1) << "That's enough " << status_count_ << " " << max_status_count_;
  }
}

void Starter::Cleanup() {
  if (exiting_) {
    return;
  }
  exiting_ = true;
  for (auto &application : applications_) {
    application.second.Terminate();
  }
  cleanup_timer_->Setup(event_loop_.monotonic_now() +
                        std::chrono::milliseconds(1500));
}

void Starter::OnSignal(signalfd_siginfo info) {
  if (info.ssi_signo == SIGCHLD) {
    // SIGCHLD messages can be collapsed if multiple are received, so all
    // applications must check their status.
    for (auto iter = applications_.begin(); iter != applications_.end();) {
      if (iter->second.MaybeHandleSignal()) {
        iter = applications_.erase(iter);
      } else {
        ++iter;
      }
    }

    if (exiting_ && applications_.empty()) {
      event_loop_.Exit();
    }
  } else {
    LOG(INFO) << "Received signal '" << strsignal(info.ssi_signo) << "'";

    if (std::find(kStarterDeath.begin(), kStarterDeath.end(), info.ssi_signo) !=
        kStarterDeath.end()) {
      LOG(WARNING) << "Starter shutting down";
      Cleanup();
    }
  }
}

Application *Starter::AddApplication(const aos::Application *application) {
  auto [iter, success] =
      applications_.try_emplace(application->name()->str(), application,
                                &event_loop_, [this]() { MaybeSendStatus(); });
  if (success) {
    if (application->has_args()) {
      iter->second.set_args(*application->args());
    }
    return &(iter->second);
  }
  return nullptr;
}

void Starter::Run() {
#ifdef AOS_ARCHITECTURE_arm_frc
  PCHECK(setuid(0) == 0) << "Failed to change user to root";
#endif

  for (auto &application : applications_) {
    if (application.second.autostart()) {
      application.second.Start();
    }
  }

  event_loop_.Run();
}

void Starter::SendStatus() {
  aos::Sender<aos::starter::Status>::Builder builder =
      status_sender_.MakeBuilder();

  std::vector<flatbuffers::Offset<aos::starter::ApplicationStatus>> statuses;

  for (auto &application : applications_) {
    statuses.push_back(application.second.PopulateStatus(builder.fbb()));
  }

  auto statuses_fbs = builder.fbb()->CreateVector(statuses);

  aos::starter::Status::Builder status_builder(*builder.fbb());
  status_builder.add_statuses(statuses_fbs);
  builder.CheckOk(builder.Send(status_builder.Finish()));
}

}  // namespace starter
}  // namespace aos
