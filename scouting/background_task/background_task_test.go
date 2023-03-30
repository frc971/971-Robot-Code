package background_task

import (
	"testing"
	"time"
)

func TestBackgroundTask(t *testing.T) {
	task := New(100 * time.Millisecond)
	defer task.Stop()

	counter := 0
	task.Start(func() {
		counter += 1
	})

	// Block until we've seeen 10 timer ticks.
	for counter < 10 {
		time.Sleep(100 * time.Millisecond)
	}
}

func TestSelfCancellation(t *testing.T) {
	task := New(100 * time.Millisecond)

	done := false
	counter := 0
	task.Start(func() {
		counter += 1

		if done {
			t.Fatal("callback should not be called after cancellation")
		}

		if counter == 10 {
			task.StopFromWithinTask()
			done = true
		}
	})

	// Block until the background task has cancelled itself.
	for !done {
		time.Sleep(100 * time.Millisecond)
	}

	// Then sleep for a little longer to make sure that the task won't
	// invoke the t.Fatal().
	time.Sleep(time.Second)
	task.Stop()
}
