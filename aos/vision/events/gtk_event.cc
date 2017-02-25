#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <sys/epoll.h>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "aos/vision/events/epoll_events.h"

namespace aos {
namespace events {

void EpollLoop::RunWithGtkMain() {
  int timeout;
  static constexpr size_t kNumberOfEvents = 64;
  epoll_event events[kNumberOfEvents];
  int number_events = 0;

  std::mutex m;
  std::condition_variable cv;
  bool all_events_handled = false;
  auto handle_cb = [&]() {
    {
      std::unique_lock<std::mutex> lk(m);

      for (int i = 0; i < number_events; i++) {
        EpollEvent *event = static_cast<EpollEvent *>(events[i].data.ptr);
        if ((events[i].events & ~(EPOLLIN | EPOLLPRI)) != 0) {
          LOG(FATAL, "unexpected epoll events set in %x on %d\n",
              events[i].events, event->fd());
        }
        event->ReadEvent();
      }
      timeout = CalculateTimeout();

      all_events_handled = true;
    }
    cv.notify_one();
  };
  handle_cb();
  using HandleCBType = decltype(handle_cb);

  std::thread t([&]() {
    std::unique_lock<std::mutex> lk(m);
    while (true) {
      cv.wait(lk, [&all_events_handled] { return all_events_handled; });
      // Wait for handle_cb to be done.
      number_events =
          PCHECK(epoll_wait(epoll_fd(), events, kNumberOfEvents, timeout));
      all_events_handled = false;
      // Trigger handle_cb on main_thread to avoid concurrency.
      gdk_threads_add_idle(
          +[](gpointer user_data) -> gboolean {
            auto &handle_cb = *reinterpret_cast<HandleCBType *>(user_data);
            handle_cb();
            return G_SOURCE_REMOVE;
          },
          &handle_cb);
    }
  });
  gtk_main();

  // TODO(parker): Allow concurrent proxy onto the normal thread just like Gtk
  // in order to allow event loop fusion, and make event addition thread-safe.

  // Avoid stack destructors (explicitly shutting down of the thread.)
  exit(EXIT_SUCCESS);
}

}  // namespace events
}  // namespace aos
