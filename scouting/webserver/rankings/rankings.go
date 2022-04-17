package rankings

import (
	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
	"log"
	"strconv"
	"strings"
	"time"
)

type rankingScraper struct {
	doneChan     chan<- bool
	checkStopped chan<- bool
}

type Database interface {
	AddOrUpdateRankings(db.Ranking) error
}

func parseTeamKey(teamKey string) (int, error) {
	// TBA prefixes teams with "frc". Not sure why. Get rid of that.
	teamKey = strings.TrimPrefix(teamKey, "frc")
	return strconv.Atoi(teamKey)
}

func getRankings(database Database, year int32, eventCode string, blueAllianceConfig string) {
	rankings, err := scraping.AllRankings(year, eventCode, blueAllianceConfig)
	if err != nil {
		log.Println("Failed to scrape ranking list: ", err)
		return
	}

	for _, rank := range rankings.Rankings {
		teamKey, err := parseTeamKey(rank.TeamKey)

		if err != nil {
			log.Println("Failed to parse team key: ", err)
			continue
		}

		rankingInfo := db.Ranking{
			TeamNumber: teamKey,
			Losses:     rank.Records.Losses, Wins: rank.Records.Wins, Ties: rank.Records.Ties,
			Rank: rank.Rank, Dq: rank.Dq,
		}
		err = database.AddOrUpdateRankings(rankingInfo)

		if err != nil {
			log.Println("Failed to add or update database: ", err)
		}
	}
}

func (scraper *rankingScraper) Start(database Database, year int32, eventCode string, blueAllianceConfig string) {
	scraper.doneChan = make(chan bool, 1)
	scraper.checkStopped = make(chan bool, 1)

	go func(database Database, year int32, eventCode string) {
		// Setting start time to 11 minutes prior so getRankings called instantly when Start() called
		startTime := time.Now().Add(-11 * time.Minute)
		for {
			curTime := time.Now()
			diff := curTime.Sub(startTime)

			if diff.Minutes() > 10 {
				getRankings(database, year, eventCode, blueAllianceConfig)
				startTime = curTime
			}

			if len(scraper.doneChan) != 0 {
				break
			}

			time.Sleep(time.Second)
		}

		scraper.checkStopped <- true
	}(database, year, eventCode)
}

func (scraper *rankingScraper) Stop() {
	scraper.doneChan <- true

	for {
		if len(scraper.checkStopped) != 0 {
			close(scraper.doneChan)
			close(scraper.checkStopped)
			break
		}
	}
}
