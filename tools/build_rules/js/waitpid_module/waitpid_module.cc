#include <errno.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

#include <iostream>

#include <node.h>
#include <v8.h>

namespace tools::build_rules::js {

auto CreateString(v8::Isolate *isolate, const char *text) {
  return v8::String::NewFromUtf8(isolate, text).ToLocalChecked();
}

void WaitPid(const v8::FunctionCallbackInfo<v8::Value> &args) {
  v8::Isolate *isolate = args.GetIsolate();

  if (args.Length() != 1 || !args[0]->IsInt32()) {
    isolate->ThrowException(v8::Exception::TypeError(
        CreateString(isolate, "Need a single integer argument")));
    return;
  }

  int pid = args[0].As<v8::Int32>()->Value();
  std::cout << "Waiting on PID " << pid << std::endl;

  int status;

  while (waitpid(pid, &status, 0) == -1) {
    if (errno == EINTR) {
      continue;
    }
    isolate->ThrowException(
        v8::Exception::Error(CreateString(isolate, "waitpid() failed")));
    return;
  }

  // The expectation is for the child process to exit with 0. Anything else is
  // an error that can be debugged separately.
  if (WIFEXITED(status)) {
    if (WEXITSTATUS(status) != 0) {
      isolate->ThrowException(v8::Exception::Error(
          CreateString(isolate, "child exited with error code")));
      return;
    }
  } else if (WIFSIGNALED(status)) {
    isolate->ThrowException(v8::Exception::Error(
        CreateString(isolate, "child exited because of signal")));
    return;
  } else {
    isolate->ThrowException(v8::Exception::Error(
        CreateString(isolate, "unhandled child state change")));
    return;
  }

  std::cout << "Successfully waited on PID " << pid << std::endl;
}

void Initialize(v8::Local<v8::Object> exports) {
  NODE_SET_METHOD(exports, "waitpid", WaitPid);
}

NODE_MODULE(waitpid_module, Initialize)

}  // namespace tools::build_rules::js
