package match_list

import (
	"errors"
	"fmt"
	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
	"log"
	"strconv"
	"strings"
)

type Database interface {
	AddToMatch2025(db.TeamMatch2025) error
}

func parseTeamKey(teamKey string) string {
	// TBA prefixes teams with "frc". Not sure why. Get rid of that.
	teamKey = strings.TrimPrefix(teamKey, "frc")
	return teamKey
}

// Parses the alliance data from the specified match and returns the three red
// teams and the three blue teams.
func parseTeamKeys(match *scraping.Match) ([3]string, [3]string, error) {
	redKeys := match.Alliances.Red.TeamKeys
	blueKeys := match.Alliances.Blue.TeamKeys

	if len(redKeys) != 3 || len(blueKeys) != 3 {
		return [3]string{}, [3]string{}, errors.New(fmt.Sprintf(
			"Found %d red teams and %d blue teams.", len(redKeys), len(blueKeys)))
	}

	var red [3]string
	for i, key := range redKeys {
		team := parseTeamKey(key)
		red[i] = team
	}
	var blue [3]string
	for i, key := range blueKeys {
		team := parseTeamKey(key)
		blue[i] = team
	}
	return red, blue, nil
}

func GetMatchList(database Database, year int32, eventCode string, blueAllianceConfig string) {
	matches, err := scraping.GetAllData[[]scraping.Match](year, eventCode, blueAllianceConfig, "matches")
	if err != nil {
		log.Println("Failed to scrape match list: ", err)
		return
	}

	for _, match := range matches {
		// Make sure the data is valid.
		red, blue, err := parseTeamKeys(&match)
		if err != nil {
			log.Println("TheBlueAlliance data for match %d is malformed: %v", match.MatchNumber, err)
			return
		}

		team_matches := []db.TeamMatch2025{
			{
				MatchNumber: int32(match.MatchNumber),
				CompCode:    strconv.Itoa(int(year)) + eventCode,
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "R", AlliancePosition: 1, TeamNumber: red[0],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				CompCode:    strconv.Itoa(int(year)) + eventCode,
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "R", AlliancePosition: 2, TeamNumber: red[1],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				CompCode:    strconv.Itoa(int(year)) + eventCode,
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "R", AlliancePosition: 3, TeamNumber: red[2],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				CompCode:    strconv.Itoa(int(year)) + eventCode,
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "B", AlliancePosition: 1, TeamNumber: blue[0],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				CompCode:    strconv.Itoa(int(year)) + eventCode,
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "B", AlliancePosition: 2, TeamNumber: blue[1],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				CompCode:    strconv.Itoa(int(year)) + eventCode,
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "B", AlliancePosition: 3, TeamNumber: blue[2],
			},
		}

		for _, match := range team_matches {
			// Iterate through matches to check they can be added to database.
			err = database.AddToMatch2025(match)
			if err != nil {
				log.Println("Failed to add team %d from match %d to the database: %v", match.TeamNumber, match.MatchNumber, err)
				return
			}
		}
	}
}
