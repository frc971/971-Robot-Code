#include "aos/atom_code/starter/starter.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/signalfd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <sys/stat.h>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/atom_code/init.h"

void niceexit(int status);

pid_t start(const char *cmd, uint8_t times) {
  char *which_cmd, *which_cmd_stm;
  if (asprintf(&which_cmd, "which %s", cmd) == -1) {
    LOG(ERROR, "creating \"which %s\" failed with %d: %s\n",
        cmd, errno, strerror(errno));
    niceexit(EXIT_FAILURE);
  }
  if (asprintf(&which_cmd_stm, "which %s.stm", cmd) == -1) {
    LOG(ERROR, "creating \"which %s.stm\" failed with %d: %s\n",
        cmd, errno, strerror(errno));
    niceexit(EXIT_FAILURE);
  }
  FILE *which = popen(which_cmd, "r");
  char exe[CMDLEN + 5], orig_exe[CMDLEN];
  size_t ret;
  if ((ret = fread(orig_exe, 1, sizeof(orig_exe), which)) == CMDLEN) {
    LOG(ERROR, "`which %s` was too long. not starting '%s'\n", cmd, cmd);
    return 0;
  }
  orig_exe[ret] = '\0';
  if (pclose(which) == -1) {
    LOG(WARNING, "pclose failed with %d: %s\n", errno, strerror(errno));
  }
  free(which_cmd);
  if (strlen(orig_exe) == 0) { // which returned nothing; check if stm exists
    LOG(INFO, "%s didn't exist. trying %s.stm\n", cmd, cmd);
    FILE *which_stm = popen(which_cmd_stm, "r");
    if ((ret = fread(orig_exe, 1, sizeof(orig_exe), which_stm)) == CMDLEN) {
      LOG(ERROR, "`which %s.stm` was too long. not starting %s\n", cmd, cmd);
      return 0;
    }
    orig_exe[ret] = '\0';
    if (pclose(which) == -1) {
      LOG(WARNING, "pclose failed with %d: %s\n", errno, strerror(errno));
    }
  }
  if (strlen(orig_exe) == 0) {
    LOG(WARNING, "couldn't find file '%s[.stm]'. not going to start it\n",
        cmd);
    return 0;
  }
  if (orig_exe[strlen(orig_exe) - 1] != '\n') {
    LOG(WARNING, "no \\n on the end of `which %s[.stm]` output ('%s')\n",
        cmd, orig_exe);
  } else {
    orig_exe[strlen(orig_exe) - 1] = '\0'; // get rid of the \n
  }
  strncpy(exe, orig_exe, CMDLEN);

  strcat(exe, ".stm");
  struct stat st;
  errno = 0;
  if (stat(orig_exe, &st) != 0 && errno != ENOENT) {
    LOG(ERROR, "killing everything because stat('%s') failed with %d: %s\n",
        orig_exe, errno, strerror(errno));
    niceexit(EXIT_FAILURE);
  } else if (errno == ENOENT) {
    LOG(WARNING, "binary '%s' doesn't exist. not starting it\n", orig_exe);
    return 0;
  }
  struct stat st2;
  // if we can confirm it's already 0 size
  bool orig_zero = stat(orig_exe, &st2) == 0 && st2.st_size == 0;
  if (!orig_zero) {
    // if it failed and it wasn't because it was missing
    if (unlink(exe) != 0 && (errno != EROFS && errno != ENOENT)) {
      LOG(ERROR,
          "killing everything because unlink('%s') failed with %d: %s\n",
          exe, errno, strerror(errno));
      niceexit(EXIT_FAILURE);
    }
    if (link(orig_exe, exe) != 0) {
      LOG(ERROR,
          "killing everything because link('%s', '%s') failed with %d: %s\n",
          orig_exe, exe, errno, strerror(errno));
      niceexit(EXIT_FAILURE);
    }
  }
  if (errno == EEXIST) {
    LOG(INFO, "exe ('%s') already existed\n", exe);
  }

  pid_t child;
  if ((child = fork()) == 0) {
    execlp(exe, orig_exe, static_cast<char *>(NULL));
    LOG(ERROR,
        "killing everything because execlp('%s', '%s', NULL) "
        "failed with %d: %s\n",
        exe, cmd, errno, strerror(errno));
    _exit(EXIT_FAILURE); // don't niceexit or anything because this is the child!!
  }
  if (child == -1) {
    LOG(WARNING, "fork on '%s' failed with %d: %s",
        cmd, errno, strerror(errno));
    if (times < 100) {
      return start(cmd, times + 1);
    } else {
      LOG(ERROR, "tried to start '%s' too many times. giving up\n", cmd);
      return 0;
    }
  } else {
    children[child] = cmd;
    files[child] = orig_exe;
    int ret = inotify_add_watch(notifyfd, orig_exe, IN_ATTRIB | IN_MODIFY);
    if (ret < 0) {
      LOG(WARNING, "inotify_add_watch('%s') failed: "
          "not going to watch for changes to it because of %d: %s\n",
          orig_exe, errno, strerror(errno));
    } else {
      watches[ret] = child;
      mtimes[ret] = st2.st_mtime;
    }
    return child;
  }
}

static bool exited = false;
void exit_handler() {
  if(exited) {
    return;
  } else {
    exited = true;
  }
  fputs("starter: killing all children for exit\n", stdout);
  for (auto it = children.begin(); it != children.end(); ++it) {
    printf("starter: killing child %d ('%s') for exit\n", it->first, it->second);
    kill(it->first, SIGKILL);
  }
  if (sigfd != 0) {
    close(sigfd);
  }
  if (notifyfd != 0) {
    close(notifyfd);
  }
}
void niceexit(int status) {
  printf("starter: niceexit(%d) EXIT_SUCCESS=%d EXIT_FAILURE=%d\n",
         status, EXIT_SUCCESS, EXIT_FAILURE);
  exit_handler();
  exit(status);
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    fputs("starter: error: need an argument specifying what file to use\n", stderr);
    niceexit(EXIT_FAILURE);
  } else if(argc > 2) {
    fputs("starter: warning: too many arguments\n", stderr);
  }

  atexit(exit_handler);

  aos::logging::Init();

  notifyfd = inotify_init1(IN_NONBLOCK);

  pid_t core = start("core", 0);
  if (core == 0) {
    fputs("starter: error: core didn't exist\n", stderr);
    niceexit(EXIT_FAILURE);
  }
  fprintf(stderr, "starter: info: core's pid is %jd\n", static_cast<intmax_t>(core));
  FILE *pid_file = fopen("/tmp/starter.pid", "w");
  if (pid_file == NULL) {
    perror("fopen(/tmp/starter.pid)");
  } else {
    if (fprintf(pid_file, "%d", core) == -1) {
      fprintf(stderr, "starter: error: fprintf(pid_file, core(=%d)) failed "
              "with %d: %s",
              core, errno, strerror(errno));
    }
    fclose(pid_file);
  }
  sleep(1);
  if (kill(core, 0) != 0) {
    fprintf(stderr, "starter: couldn't kill(%jd(=core), 0) because of %d: %s\n",
            static_cast<intmax_t>(core), errno, strerror(errno));
    niceexit(EXIT_FAILURE);
  }
  fputs("starter: before init\n", stdout);
  aos::InitNRT();
  fputs("starter: after init\n", stdout);

  FILE *list = fopen(argv[1], "re");
  char line[CMDLEN + 1];
  char *line_copy;
  uint8_t too_long = 0;
  while (fgets(line, sizeof(line), list) != NULL) {
    if (line[strlen(line) - 1] != '\n') {
      LOG(WARNING, "command segment '%s' is too long. "
          "increase the size of the line char[] above " __FILE__ ": %d\n",
          line, __LINE__);
      too_long = 1;
      continue;
    }
    if (too_long) {
      too_long = 0;
      LOG(WARNING, "\tgot last chunk of too long line: '%s'\n", line);
      continue; // don't try running the last little chunk
    }
    line[strlen(line) - 1] = '\0'; // get rid of the \n
    line_copy = new char[strlen(line) + 1];
    memcpy(line_copy, line, strlen(line) + 1);
    fprintf(stderr, "starter: info: going to start \"%s\"\n", line_copy);
    start(line_copy, 0);
  }
  fclose(list);
  LOG(INFO, "started everything\n");

  sigset_t mask;
  sigemptyset (&mask);
  sigaddset (&mask, SIGCHLD);
  sigprocmask (SIG_BLOCK, &mask, NULL);
  sigfd = signalfd (-1, &mask, O_NONBLOCK);

  fd_set readfds;
  FD_ZERO(&readfds);
  siginfo_t infop;
  signalfd_siginfo fdsi;
  inotify_event notifyevt;
  int ret;
  while (1) {
    FD_SET(sigfd, &readfds);
    FD_SET(notifyfd, &readfds);
    timeval timeout;
    timeout.tv_sec = restarts.empty() ? 2 : 0;
    timeout.tv_usec = 100000;
    ret = select (FD_SETSIZE, &readfds, NULL, NULL, &timeout);

    if (ret == 0) { // timeout
      auto it = restarts.begin();
      // WARNING because the message about it dying will be
      for (; it != restarts.end(); it++) {
        LOG(WARNING, "restarting process %d ('%s') by giving it a SIGKILL(%d)\n",
            *it, children[*it], SIGKILL);
        kill(*it, SIGKILL);
      }
      restarts.clear();
    }

    if (FD_ISSET(notifyfd, &readfds)) {
      if ((ret = read(notifyfd, &notifyevt, sizeof(notifyevt))) ==
          sizeof(notifyevt)) {
        if (watches.count(notifyevt.wd)) {
          struct stat st;
          if (!children.count(watches[notifyevt.wd]) ||
              stat(files[watches[notifyevt.wd]], &st) == 0) {
            if (mtimes[notifyevt.wd] == st.st_mtime) {
              LOG(DEBUG, "ignoring trigger of watch id %d (file '%s')"
                  " because mtime didn't change\n",
                  notifyevt.wd, files[watches[notifyevt.wd]]);
            } else if (children.count(watches[notifyevt.wd])) {
              LOG(DEBUG, "adding process %d to the restart list\n",
                  watches[notifyevt.wd]);
              restarts.insert(watches[notifyevt.wd]);
            } else {
              LOG(DEBUG, "children doesn't have entry for PID %d\n",
                  watches[notifyevt.wd]);
            }
          } else {
            LOG(ERROR, "stat('%s') failed with %d: %s\n",
                files[watches[notifyevt.wd]], errno, strerror(errno));
          }
        } else {
          LOG(WARNING, "no PID for watch id %d\n", notifyevt.wd);
        }
      } else {
        if (ret == -1) {
          LOG(WARNING, "read(notifyfd) failed with %d: %s", errno, strerror(errno));
        } else {
          LOG(WARNING, "couldn't get a whole inotify_event(%d) (only got %d)\n",
              sizeof(notifyevt), ret);
        }
      }
    }

    if (FD_ISSET(sigfd, &readfds)) {
      while(read (sigfd, &fdsi, sizeof fdsi) > 0);
    }
    while (1) {
      infop.si_pid = 0;
      if (waitid(P_ALL, 0, &infop, WEXITED | WSTOPPED | WNOHANG) == 0) {
        if (infop.si_pid == 0) {
          goto after_loop; // no more child process changes pending
        }
        switch (infop.si_code) {
          case CLD_EXITED:
            LOG(WARNING, "child %d (%s) exited with status %d\n",
                infop.si_pid, children[infop.si_pid], infop.si_status);
            break;
          case CLD_DUMPED:
            LOG(INFO, "child %d actually dumped core. "
                "falling through to killed by signal case\n", infop.si_pid);
          case CLD_KILLED:
            LOG(WARNING, "child %d (%s) was killed by signal %d (%s)\n",
                infop.si_pid, children[infop.si_pid], infop.si_status,
                strsignal(infop.si_status));
            break;
          case CLD_STOPPED:
            LOG(WARNING, "child %d (%s) was stopped by signal %d "
                "(giving it a SIGCONT(%d))\n",
                infop.si_pid, children[infop.si_pid], infop.si_status, SIGCONT);
            kill(infop.si_pid, SIGCONT);
            continue;
          default:
            LOG(WARNING, "something happened to child %d (%s) (killing it)\n",
                infop.si_pid, children[infop.si_pid]);
            kill(infop.si_pid, SIGKILL);
            continue;
        }
        if (infop.si_pid == core) {
          fprintf(stderr, "starter: si_code=%d CLD_EXITED=%d CLD_DUMPED=%d "
                  "CLD_KILLED=%d CLD_STOPPED=%d si_status=%d (sig '%s')\n",
                  infop.si_code, CLD_EXITED, CLD_DUMPED, CLD_KILLED,
                  CLD_STOPPED, infop.si_status, strsignal(infop.si_status));
          // core has died. logging is down too
          fputs("starter: error: core died. exiting\n", stderr);
          niceexit(EXIT_FAILURE);
        }

        start(children[infop.si_pid], 0);
        LOG(DEBUG, "erasing %d from children\n", infop.si_pid);
        children.erase(infop.si_pid);
      } else {
        LOG(WARNING, "waitid failed with %d: %s", errno, strerror(errno));
      }
    }
after_loop: ;
  }
}
