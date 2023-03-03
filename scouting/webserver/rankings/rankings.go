package rankings

import (
	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
	"log"
	"strconv"
	"strings"
)

type Database interface {
	AddOrUpdateRankings(db.Ranking) error
}

func parseTeamKey(teamKey string) (int, error) {
	// TBA prefixes teams with "frc". Not sure why. Get rid of that.
	teamKey = strings.TrimPrefix(teamKey, "frc")
	return strconv.Atoi(teamKey)
}

func GetRankings(database Database, year int32, eventCode string, blueAllianceConfig string) {
	rankings, err := scraping.GetAllData[scraping.EventRanking](year, eventCode, blueAllianceConfig, "rankings")
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
