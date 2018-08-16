# How to send and receive messages on a Queue

Let's say we have a `//y2018:basic_queue.q` file that has the following
contents:

```cpp
package y2018;

message BasicMessage {
  float value;
};

queue BasicMessage basic_queue;
```

and the corresponding entry in `//y2018/BUILD`

```python
queue_library(
    name = "basic_queue",
    srcs = [
        "basic_queue.q",
    ],
    visibility = ["//visibility:public"],
)
```

The following examples assume that you've declared a dependency on
`//y2018:basic_queue` in the relevant `BUILD` file.

## Sending a message
First, include the queue definition that is created from the `.q` file. This
will make the `basic_queue` variable available.

```cpp
#include "y2018/basic_queue.q.h"
```

```cpp

// Allocate a new message pointer to use. basic_queue is from the include
// statement.
auto new_basic_message = basic_queue.MakeMessage();
new_goal.value = 0.5;

// Send the message on the queue
if (!new_basic_message.Send()) {
  LOG(ERROR, "Failed to send.\n");
}
```

## Receiving a message

Once again, we must include the queue header file to get access to the queue.


```cpp
#include "y2018/basic_queue.q.h"
```

Then we can receive a message.


```cpp
basic_queue.FetchLatest();
if (basic_queue.get()) {
  // We have a valid message ready to be used
  LOG(INFO, "Got a BasicMessage from basic_queue with value %f\n",
      basic_queue->value);
} else {
  // No new message was available
  LOG(ERROR, "No message was received\n");
}
```
