package background_task

import (
	"time"
)

// A helper to run a function in the background at a specified interval.
// Can be used for a lot of different things.
type backgroundTask struct {
	ticker        *time.Ticker
	stopRequested chan bool
	done          chan bool
}

func New(interval time.Duration) backgroundTask {
	return backgroundTask{
		ticker:        time.NewTicker(interval),
		stopRequested: make(chan bool, 1),
		done:          make(chan bool, 1),
	}
}

func (task *backgroundTask) Start(taskFunc func()) {
	go func() {
		// Signal the Stop() function below when the goroutine has
		// finished executing.
		defer func() { task.done <- true }()

		// time.Ticker doesn't perform an immediate invocation.
		// Instead, it waits for the specified duration before
		// triggering the first tick. We pretend that there's a tick
		// here by invoking the callback manually.
		taskFunc()

		for {
			select {
			case <-task.stopRequested:
				return
			case <-task.ticker.C:
				taskFunc()
			}
		}
	}()
}

func (task *backgroundTask) Stop() {
	task.stopRequested <- true
	task.ticker.Stop()
	<-task.done
	close(task.stopRequested)
	close(task.done)
}
