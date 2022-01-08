#include "aos/starter/subprocess.h"

#include <grp.h>
#include <pwd.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "glog/logging.h"

namespace aos::starter {

SignalListener::SignalListener(aos::ShmEventLoop *loop,
                               std::function<void(signalfd_siginfo)> callback)
    : SignalListener(loop, callback,
                     {SIGHUP, SIGINT, SIGQUIT, SIGABRT, SIGFPE, SIGSEGV,
                      SIGPIPE, SIGTERM, SIGBUS, SIGXCPU, SIGCHLD}) {}

SignalListener::SignalListener(aos::ShmEventLoop *loop,
                               std::function<void(signalfd_siginfo)> callback,
                               std::initializer_list<unsigned int> signals)
    : loop_(loop), callback_(std::move(callback)), signalfd_(signals) {
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

Application::Application(const aos::Application *application,
                         aos::EventLoop *event_loop,
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

  std::tie(read_pipe_, write_pipe_) = util::ScopedPipe::MakePipe();

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

}  // namespace aos::starter
