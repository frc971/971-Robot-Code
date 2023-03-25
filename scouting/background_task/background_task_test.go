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
