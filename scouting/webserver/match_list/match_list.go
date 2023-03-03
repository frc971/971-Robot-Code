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
	AddToMatch(db.TeamMatch) error
}

func parseTeamKey(teamKey string) (int, error) {
	// TBA prefixes teams with "frc". Not sure why. Get rid of that.
	teamKey = strings.TrimPrefix(teamKey, "frc")
	magnitude := 0
	if strings.HasSuffix(teamKey, "A") {
		magnitude = 0
		teamKey = strings.TrimSuffix(teamKey, "A")
	} else if strings.HasSuffix(teamKey, "B") {
		magnitude = 9
		teamKey = strings.TrimSuffix(teamKey, "B")
	} else if strings.HasSuffix(teamKey, "C") {
		magnitude = 8
		teamKey = strings.TrimSuffix(teamKey, "C")
	} else if strings.HasSuffix(teamKey, "D") {
		magnitude = 7
		teamKey = strings.TrimSuffix(teamKey, "D")
	} else if strings.HasSuffix(teamKey, "E") {
		magnitude = 6
		teamKey = strings.TrimSuffix(teamKey, "E")
	} else if strings.HasSuffix(teamKey, "F") {
		magnitude = 5
		teamKey = strings.TrimSuffix(teamKey, "F")
	}

	if magnitude != 0 {
		teamKey = strconv.Itoa(magnitude) + teamKey
	}

	result, err := strconv.Atoi(teamKey)
	return result, err
}

// Parses the alliance data from the specified match and returns the three red
// teams and the three blue teams.
func parseTeamKeys(match *scraping.Match) ([3]int32, [3]int32, error) {
	redKeys := match.Alliances.Red.TeamKeys
	blueKeys := match.Alliances.Blue.TeamKeys

	if len(redKeys) != 3 || len(blueKeys) != 3 {
		return [3]int32{}, [3]int32{}, errors.New(fmt.Sprintf(
			"Found %d red teams and %d blue teams.", len(redKeys), len(blueKeys)))
	}

	var red [3]int32
	for i, key := range redKeys {
		team, err := parseTeamKey(key)
		if err != nil {
			return [3]int32{}, [3]int32{}, errors.New(fmt.Sprintf(
				"Failed to parse red %d team '%s' as integer: %v", i+1, key, err))
		}
		red[i] = int32(team)
	}
	var blue [3]int32
	for i, key := range blueKeys {
		team, err := parseTeamKey(key)
		if err != nil {
			return [3]int32{}, [3]int32{}, errors.New(fmt.Sprintf(
				"Failed to parse blue %d team '%s' as integer: %v", i+1, key, err))
		}
		blue[i] = int32(team)
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

		team_matches := []db.TeamMatch{
			{
				MatchNumber: int32(match.MatchNumber),
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "R", AlliancePosition: 1, TeamNumber: red[0],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "R", AlliancePosition: 2, TeamNumber: red[1],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "R", AlliancePosition: 3, TeamNumber: red[2],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "B", AlliancePosition: 1, TeamNumber: blue[0],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "B", AlliancePosition: 2, TeamNumber: blue[1],
			},
			{
				MatchNumber: int32(match.MatchNumber),
				SetNumber:   int32(match.SetNumber), CompLevel: match.CompLevel,
				Alliance: "B", AlliancePosition: 3, TeamNumber: blue[2],
			},
		}

		for _, match := range team_matches {
			// Iterate through matches to check they can be added to database.
			err = database.AddToMatch(match)
			if err != nil {
				log.Println("Failed to add team %d from match %d to the database: %v", match.TeamNumber, match.MatchNumber, err)
				return
			}
		}
	}
}
