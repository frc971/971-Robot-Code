// defines a fixture (SharedMemTestSetup) that sets up shared memory

extern "C" {
#include "shared_mem.h"
  extern struct aos_core *global_core;
}

#include <signal.h>

#include <gtest/gtest.h>
#include <sys/types.h>

// TODO(brians) read logs from here
class SharedMemTestSetup : public testing::Test{
 protected:
  pid_t core;
  int start[2];
  int memcheck[2];
  static void signal_handler(int){
     if(aos_core_free_shared_mem()){
      exit(- 1);
    }
    exit(0);
  }
  static int get_mem_usage(){
    return global_core->size - ((uint8_t *)global_core->mem_struct->msg_alloc - (uint8_t *)SHM_START);
  }
  bool checking_mem;

  virtual void BeforeLocalShmSetup() {}
  virtual void SetUp(){
    ASSERT_EQ(0, pipe(start)) << "couldn't create start pipes";
    ASSERT_EQ(0, pipe(memcheck)) << "couldn't create memcheck pipes";
    checking_mem = false;
    if((core = fork()) == 0){
      close(start[0]);
      close(memcheck[1]);
      struct sigaction act;
      act.sa_handler = signal_handler;
      sigaction(SIGINT, &act, NULL);
      if(aos_core_create_shared_mem(create)){
        exit(-1);
      }
      write_pipe(start[1], "a", 1);
      int usage = 0;
      while(1){
        char buf1;
        read_pipe(memcheck[0], &buf1, 1);
        if(usage == 0)
          usage = get_mem_usage();
        if(usage == get_mem_usage())
          buf1 = 1;
        else
          buf1 = 0;
        write_pipe(start[1], &buf1, 1);
      }
    }
    close(start[1]);
    close(memcheck[0]);
    ASSERT_NE(-1, core) << "fork failed";
    char buf;
    read_pipe(start[0], &buf, 1);

    BeforeLocalShmSetup();

    ASSERT_EQ(0, aos_core_create_shared_mem(reference)) << "couldn't create shared mem reference";
  }
  virtual void TearDown(){
    if(checking_mem){
      write_pipe(memcheck[1], "a", 1);
      char buf;
      read_pipe(start[0], &buf, 1);
      EXPECT_EQ(1, buf) << "memory got leaked";
    }
    EXPECT_EQ(0, aos_core_free_shared_mem()) << "issues freeing shared mem";
    if(core > 0){
      kill(core, SIGINT);
      siginfo_t status;
      ASSERT_EQ(0, waitid(P_PID, core, &status, WEXITED)) << "waiting for the core to finish failed";
      EXPECT_EQ(CLD_EXITED, status.si_code) << "core died";
      EXPECT_EQ(0, status.si_status) << "core exited with an error";
    }
  }
  // if any more shared memory gets allocated after calling this and not freed by the end, it's an error
  void AllSharedMemAllocated(){
    checking_mem = true;
    write_pipe(memcheck[1], "a", 1);
    char buf;
    read_pipe(start[0], &buf, 1);
  }
 private:
  // Wrapper functions for pipes because they should never have errors.
  void read_pipe(int fd, void *buf, size_t count) {
    if (read(fd, buf, count) < 0) abort();
  }
  void write_pipe(int fd, const void *buf, size_t count) {
    if (write(fd, buf, count) < 0) abort();
  }
};
class ExecVeTestSetup : public SharedMemTestSetup {
 protected:
  std::vector<std::string> files;
  std::vector<pid_t> pids;
  virtual void BeforeLocalShmSetup(){
    std::vector<std::string>::iterator it;
    pid_t child;
    for(it = files.begin(); it < files.end(); ++it){
      if((child = fork()) == 0){
        char *null = NULL;
        execve(it->c_str(), &null, &null);
        ADD_FAILURE() << "execve failed";
        perror("execve");
        exit(0);
      }
      if(child > 0)
        pids.push_back(child);
      else
        ADD_FAILURE() << "fork failed return=" << child;
    }
    usleep(150000);
  }
  virtual void TearDown(){
    std::vector<pid_t>::iterator it;
    siginfo_t status;
    for(it = pids.begin(); it < pids.end(); ++it){
      printf("attempting to SIGINT(%d) %d\n", SIGINT, *it);
      if(*it > 0){
        kill(*it, SIGINT);
        ASSERT_EQ(0, waitid(P_PID, *it, &status, WEXITED)) << "waiting for the AsyncAction(pid=" << *it << ") to finish failed";
        EXPECT_EQ(CLD_EXITED, status.si_code) << "child died (killed by signal is " << (int)CLD_KILLED << ")";
        EXPECT_EQ(0, status.si_status) << "child exited with an error";
      }else{
        FAIL();
      }
    }

    SharedMemTestSetup::TearDown();
  }
  // call this _before_ ExecVeTestSetup::SetUp()
  void AddProcess(const std::string file){
    files.push_back(file);
  }
  void PercolatePause(){
    usleep(50000);
  }
};

