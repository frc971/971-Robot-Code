#include "starterd_lib.h"

#include <fcntl.h>
#include <pwd.h>
#include <sys/fsuid.h>
#include <sys/prctl.h>

#include <algorithm>
#include <utility>

#include "glog/logging.h"
#include "glog/stl_logging.h"

namespace aos {
namespace starter {

Application::Application(const aos::Application *application,
                         aos::ShmEventLoop *event_loop)
    : name_(application->name()->string_view()),
      path_(application->has_executable_name()
                ? application->executable_name()->string_view()
                : application->name()->string_view()),
      args_(1),
      user_(application->has_user() ? FindUid(application->user()->c_str())
                                    : std::nullopt),
      event_loop_(event_loop),
      start_timer_(event_loop_->AddTimer([this] {
        status_ = aos::starter::State::RUNNING;
        LOG(INFO) << "Started " << name_;
      })),
      restart_timer_(event_loop_->AddTimer([this] { DoStart(); })),
      stop_timer_(event_loop_->AddTimer([this] {
        if (kill(pid_, SIGKILL) == 0) {
          LOG(WARNING) << "Sent SIGKILL to " << name_ << " pid: " << pid_;
        }
      }))

{}

void Application::DoStart() {
  if (status_ != aos::starter::State::WAITING) {
    return;
  }

  start_timer_->Disable();
  restart_timer_->Disable();

  LOG(INFO) << "Starting " << name_;

  std::tie(read_pipe_, write_pipe_) = ScopedPipe::MakePipe();

  const pid_t pid = fork();

  if (pid != 0) {
    if (pid == -1) {
      PLOG(WARNING) << "Failed to fork";
      stop_reason_ = aos::starter::LastStopReason::FORK_ERR;
      status_ = aos::starter::State::STOPPED;
    } else {
      pid_ = pid;
      id_ = next_id_++;
      start_time_ = event_loop_->monotonic_now();
      status_ = aos::starter::State::STARTING;

      // Setup timer which moves application to RUNNING state if it is still
      // alive in 1 second.
      start_timer_->Setup(event_loop_->monotonic_now() +
                          std::chrono::seconds(1));
    }
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

  if (user_) {
    if (seteuid(*user_) == -1 || setfsuid(*user_) == -1) {
      write_pipe_.Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_USR_ERR));
      PLOG(FATAL) << "Could not set user for " << name_ << " to " << *user_;
    }
  }

  // argv[0] should be the program name
  args_.insert(args_.begin(), path_.data());

  execv(path_.c_str(), args_.data());

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
      LOG(INFO) << "Killing " << name_ << " pid: " << pid_;
      status_ = aos::starter::State::STOPPING;

      kill(pid_, SIGINT);

      // Watchdog timer to SIGKILL application if it is still running 1 second
      // after SIGINT
      stop_timer_->Setup(event_loop_->monotonic_now() +
                         std::chrono::seconds(1));
      queue_restart_ = restart;
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

  LOG(INFO) << "Restarting " << name_ << " in 1 second";
  restart_timer_->Setup(event_loop_->monotonic_now() + std::chrono::seconds(1));
  start_timer_->Disable();
  stop_timer_->Disable();
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
  struct passwd *user_data = getpwnam(name);
  if (user_data != nullptr) {
    return user_data->pw_uid;
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
      LOG(WARNING) << "Failed to start " << name_ << " on pid " << pid_
                   << " : Exited with status " << exit_code_;
      QueueStart();
      break;
    }
    case aos::starter::State::RUNNING: {
      QueueStart();
      break;
    }
    case aos::starter::State::STOPPING: {
      LOG(INFO) << "Successfully stopped " << name_;
      status_ = aos::starter::State::STOPPED;

      // Disable force stop timer since the process already died
      stop_timer_->Disable();

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
          << "Received signal on process that was already stopped : name: "
          << name_ << " pid: " << pid_;
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

Starter::Starter(const aos::Configuration *event_loop_config)
    : config_msg_(event_loop_config),
      event_loop_(event_loop_config),
      status_sender_(event_loop_.MakeSender<aos::starter::Status>("/aos")),
      status_timer_(event_loop_.AddTimer([this] { SendStatus(); })),
      cleanup_timer_(event_loop_.AddTimer([this] { event_loop_.Exit(); })),
      listener_(&event_loop_,
                [this](signalfd_siginfo signal) { OnSignal(signal); }) {
  event_loop_.SkipTimingReport();
  event_loop_.SkipAosLog();

  event_loop_.OnRun([this] {
    status_timer_->Setup(event_loop_.monotonic_now(),
                         std::chrono::milliseconds(500));
  });

  event_loop_.MakeWatcher("/aos", [this](const aos::starter::StarterRpc &cmd) {
    if (!cmd.has_command() || !cmd.has_name() || exiting_) {
      return;
    }
    LOG(INFO) << "Received command "
              << aos::starter::EnumNameCommand(cmd.command()) << ' '
              << cmd.name()->string_view();

    auto search = applications_.find(cmd.name()->str());
    if (search != applications_.end()) {
      // If an applicatione exists by the given name, dispatch the command
      search->second.HandleCommand(cmd.command());
    }
  });

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
  LOG(INFO) << "Received signal " << strsignal(info.ssi_signo);

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
  } else if (std::find(kStarterDeath.begin(), kStarterDeath.end(),
                       info.ssi_signo) != kStarterDeath.end()) {
    LOG(WARNING) << "Starter shutting down";
    Cleanup();
  }
}

Application *Starter::AddApplication(const aos::Application *application) {
  auto [iter, success] = applications_.try_emplace(application->name()->str(),
                                                   application, &event_loop_);
  if (success) {
    if (application->has_args()) {
      iter->second.set_args(*application->args());
    }
    return &(iter->second);
  }
  return nullptr;
}

void Starter::Run() {
  for (auto &application : applications_) {
    application.second.Start();
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
  CHECK(builder.Send(status_builder.Finish()));
}

}  // namespace starter
}  // namespace aos
