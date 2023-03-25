package background_task

import (
	"time"
)

// A helper to run a function in the background at a specified interval.
// Can be used for a lot of different things.
type backgroundTask struct {
	doneChan     chan<- bool
	checkStopped chan<- bool
	interval     time.Duration
}

func New(interval time.Duration) backgroundTask {
	return backgroundTask{
		doneChan:     make(chan bool, 1),
		checkStopped: make(chan bool, 1),
		interval:     interval,
	}
}

func (task *backgroundTask) Start(taskFunc func()) {
	go func() {
		// Setting start time to a time prior so the function gets
		// called instantly when Start() called
		startTime := time.Now().Add(-task.interval - time.Minute)
		for {
			curTime := time.Now()
			diff := curTime.Sub(startTime)

			if diff > task.interval {
				taskFunc()
				startTime = curTime
			}

			if len(task.doneChan) != 0 {
				break
			}

			time.Sleep(time.Second)
		}

		task.checkStopped <- true
	}()
}

func (task *backgroundTask) Stop() {
	task.doneChan <- true

	for {
		if len(task.checkStopped) != 0 {
			close(task.doneChan)
			close(task.checkStopped)
			break
		}
	}
}
