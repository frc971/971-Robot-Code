package background_task

import (
	"time"
)

// A helper to run a function in the background every ~10 minutes. Technically
// can be used for a lot of different things, but is primarily geared towards
// scraping thebluealliance.com.
type BackgroundTask struct {
	doneChan     chan<- bool
	checkStopped chan<- bool
}

func (scraper *BackgroundTask) Start(scrape func()) {
	scraper.doneChan = make(chan bool, 1)
	scraper.checkStopped = make(chan bool, 1)

	go func() {
		// Setting start time to 11 minutes prior so getRankings called instantly when Start() called
		startTime := time.Now().Add(-11 * time.Minute)
		for {
			curTime := time.Now()
			diff := curTime.Sub(startTime)

			if diff.Minutes() > 10 {
				scrape()
				startTime = curTime
			}

			if len(scraper.doneChan) != 0 {
				break
			}

			time.Sleep(time.Second)
		}

		scraper.checkStopped <- true
	}()
}

func (scraper *BackgroundTask) Stop() {
	scraper.doneChan <- true

	for {
		if len(scraper.checkStopped) != 0 {
			close(scraper.doneChan)
			close(scraper.checkStopped)
			break
		}
	}
}
