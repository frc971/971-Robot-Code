// This file provides a Python module for reading logfiles. See
// log_reader_test.py for usage.
//
// This reader works by having the user specify exactly what channels they want
// data for. We then process the logfile and store all the data on that channel
// into a list of timestamps + JSON message data. The user can then use an
// accessor method (get_data_for_channel) to retrieve the cached data.

// Defining PY_SSIZE_T_CLEAN seems to be suggested by most of the Python
// documentation.
#define PY_SSIZE_T_CLEAN
// Note that Python.h needs to be included before anything else.
#include <Python.h>

#include <memory>

#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"

namespace frc971 {
namespace analysis {
namespace {

// All the data corresponding to a single message.
struct MessageData {
  aos::monotonic_clock::time_point monotonic_sent_time;
  aos::realtime_clock::time_point realtime_sent_time;
  // JSON representation of the message.
  std::string json_data;
};

// Data corresponding to an entire channel.
struct ChannelData {
  std::string name;
  std::string type;
  // Each message published on the channel, in order by monotonic time.
  std::vector<MessageData> messages;
};

// All the objects that we need for managing reading a logfile.
struct LogReaderTools {
  std::unique_ptr<aos::logger::LogReader> reader;
  std::unique_ptr<aos::SimulatedEventLoopFactory> event_loop_factory;
  // Event loop to use for subscribing to buses.
  std::unique_ptr<aos::EventLoop> event_loop;
  std::vector<ChannelData> channel_data;
  // Whether we have called process() on the reader yet.
  bool processed = false;
};

struct LogReaderType {
  PyObject_HEAD;
  LogReaderTools *tools = nullptr;
};

void LogReader_dealloc(LogReaderType *self) {
  LogReaderTools *tools = self->tools;
  if (!tools->processed) {
    tools->reader->Deregister();
  }
  delete tools;
  Py_TYPE(self)->tp_free((PyObject *)self);
}

PyObject *LogReader_new(PyTypeObject *type, PyObject * /*args*/,
                            PyObject * /*kwds*/) {
  LogReaderType *self;
  self = (LogReaderType *)type->tp_alloc(type, 0);
  if (self != nullptr) {
    self->tools = new LogReaderTools();
    if (self->tools == nullptr) {
      return nullptr;
    }
  }
  return (PyObject *)self;
}

int LogReader_init(LogReaderType *self, PyObject *args, PyObject *kwds) {
  const char *kwlist[] = {"log_file_name", nullptr};

  const char *log_file_name;
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", const_cast<char **>(kwlist),
                                   &log_file_name)) {
    return -1;
  }

  LogReaderTools *tools = CHECK_NOTNULL(self->tools);
  tools->reader = std::make_unique<aos::logger::LogReader>(log_file_name);
  tools->event_loop_factory = std::make_unique<aos::SimulatedEventLoopFactory>(
      tools->reader->configuration());
  tools->reader->Register(tools->event_loop_factory.get());

  tools->event_loop = tools->event_loop_factory->MakeEventLoop("data_fetcher");
  tools->event_loop->SkipTimingReport();

  return 0;
}

PyObject *LogReader_get_data_for_channel(LogReaderType *self,
                                                PyObject *args,
                                                PyObject *kwds) {
  const char *kwlist[] = {"name", "type", nullptr};

  const char *name;
  const char *type;
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "ss",
                                   const_cast<char **>(kwlist), &name, &type)) {
    return nullptr;
  }

  LogReaderTools *tools = CHECK_NOTNULL(self->tools);

  if (!tools->processed) {
    PyErr_SetString(PyExc_RuntimeError,
                    "Called get_data_for_bus before calling process().");
    return nullptr;
  }

  for (const auto &channel : tools->channel_data) {
    if (channel.name == name && channel.type == type) {
      PyObject *list = PyList_New(channel.messages.size());
      for (size_t ii = 0; ii < channel.messages.size(); ++ii)
      {
        const auto &message = channel.messages[ii];
        PyObject *monotonic_time = PyLong_FromLongLong(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                message.monotonic_sent_time.time_since_epoch())
                .count());
        PyObject *realtime_time = PyLong_FromLongLong(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                message.realtime_sent_time.time_since_epoch())
                .count());
        PyObject *json_data = PyUnicode_FromStringAndSize(
            message.json_data.data(), message.json_data.size());
        PyObject *entry =
            PyTuple_Pack(3, monotonic_time, realtime_time, json_data);
        if (PyList_SetItem(list, ii, entry) != 0) {
          return nullptr;
        }
      }
      return list;
    }
  }
  PyErr_SetString(PyExc_ValueError,
                  "The provided channel was never subscribed to.");
  return nullptr;
}

PyObject *LogReader_subscribe(LogReaderType *self, PyObject *args,
                                     PyObject *kwds) {
  const char *kwlist[] = {"name", "type", nullptr};

  const char *name;
  const char *type;
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "ss",
                                   const_cast<char **>(kwlist), &name, &type)) {
    return nullptr;
  }

  LogReaderTools *tools = CHECK_NOTNULL(self->tools);

  if (tools->processed) {
    PyErr_SetString(PyExc_RuntimeError,
                    "Called subscribe after calling process().");
    return nullptr;
  }

  const aos::Channel *const channel = aos::configuration::GetChannel(
      tools->reader->configuration(), name, type, "", nullptr);
  if (channel == nullptr) {
    return Py_False;
  }
  const int index = tools->channel_data.size();
  tools->channel_data.push_back(
      {.name = name, .type = type, .messages = {}});
  tools->event_loop->MakeRawWatcher(
      channel, [channel, index, tools](const aos::Context &context,
                                       const void *message) {
        tools->channel_data[index].messages.push_back(
            {.monotonic_sent_time = context.monotonic_event_time,
             .realtime_sent_time = context.realtime_event_time,
             .json_data = aos::FlatbufferToJson(
                 channel->schema(), static_cast<const uint8_t *>(message))});
      });
  return Py_True;
}

static PyObject *LogReader_process(LogReaderType *self,
                                   PyObject *Py_UNUSED(ignored)) {
  LogReaderTools *tools = CHECK_NOTNULL(self->tools);

  if (tools->processed) {
    PyErr_SetString(PyExc_RuntimeError, "process() may only be called once.");
    return nullptr;
  }

  tools->processed = true;

  tools->event_loop_factory->Run();
  tools->reader->Deregister();

  Py_RETURN_NONE;
}

static PyObject *LogReader_configuration(LogReaderType *self,
                                         PyObject *Py_UNUSED(ignored)) {
  LogReaderTools *tools = CHECK_NOTNULL(self->tools);

  // I have no clue if the Configuration that we get from the log reader is in a
  // contiguous chunk of memory, and I'm too lazy to either figure it out or
  // figure out how to extract the actual data buffer + offset.
  // Instead, copy the flatbuffer and return a copy of the new buffer.
  aos::FlatbufferDetachedBuffer<aos::Configuration> buffer =
      aos::CopyFlatBuffer(tools->reader->configuration());

  return PyBytes_FromStringAndSize(
      reinterpret_cast<const char *>(buffer.data()), buffer.size());
}

static PyMethodDef LogReader_methods[] = {
    {"configuration", (PyCFunction)LogReader_configuration, METH_NOARGS,
     "Return a bytes buffer for the Configuration of the logfile."},
    {"process", (PyCFunction)LogReader_process, METH_NOARGS,
     "Processes the logfile and all the subscribed to channels."},
    {"subscribe", (PyCFunction)LogReader_subscribe,
     METH_VARARGS | METH_KEYWORDS,
     "Attempts to subscribe to the provided channel name + type. Returns True "
     "if successful."},
    {"get_data_for_channel", (PyCFunction)LogReader_get_data_for_channel,
     METH_VARARGS | METH_KEYWORDS,
     "Returns the logged data for a given channel. Raises an exception if you "
     "did not subscribe to the provided channel. Returned data is a list of "
     "tuples where each tuple is of the form (monotonic_nsec, realtime_nsec, "
     "json_message_data)."},
    {nullptr, 0, 0, nullptr} /* Sentinel */
};

static PyTypeObject LogReaderType = {
    PyVarObject_HEAD_INIT(NULL, 0).tp_name = "py_log_reader.LogReader",
    .tp_doc = "LogReader objects",
    .tp_basicsize = sizeof(LogReaderType),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_new = LogReader_new,
    .tp_init = (initproc)LogReader_init,
    .tp_dealloc = (destructor)LogReader_dealloc,
    .tp_methods = LogReader_methods,
};

static PyModuleDef log_reader_module = {
    PyModuleDef_HEAD_INIT,
    .m_name = "py_log_reader",
    .m_doc = "Example module that creates an extension type.",
    .m_size = -1,
};

PyObject *InitModule() {
  PyObject *m;
  if (PyType_Ready(&LogReaderType) < 0) return nullptr;

  m = PyModule_Create(&log_reader_module);
  if (m == nullptr) return nullptr;

  Py_INCREF(&LogReaderType);
  if (PyModule_AddObject(m, "LogReader", (PyObject *)&LogReaderType) < 0) {
    Py_DECREF(&LogReaderType);
    Py_DECREF(m);
    return nullptr;
  }

  return m;
}

}  // namespace
}  // namespace analysis
}  // namespace frc971

PyMODINIT_FUNC PyInit_py_log_reader(void) {
  return frc971::analysis::InitModule();
}
