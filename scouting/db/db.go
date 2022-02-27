package db

import (
	"database/sql"
	"fmt"

	_ "github.com/mattn/go-sqlite3"
)

type Database struct {
	*sql.DB
}

type Match struct {
	matchNumber, round     int
	compLevel              string
	r1, r2, r3, b1, b2, b3 int
	// Each of these variables holds the matchID of the corresponding Stats row
	r1ID, r2ID, r3ID, b1ID, b2ID, b3ID int
}

type Stats struct {
	teamNumber, matchNumber                                      int
	shotsMissed, upperGoalShots, lowerGoalShots                  int
	shotsMissedAuto, upperGoalAuto, lowerGoalAuto, playedDefense int
	climbing                                                     int
}

// Opens a database at the specified path. If the path refers to a non-existent
// file, the database will be created and initialized with empty tables.
func NewDatabase(path string) (*Database, error) {
	database := new(Database)
	database.DB, _ = sql.Open("sqlite3", path)
	statement, error_ := database.Prepare("CREATE TABLE IF NOT EXISTS matches " +
		"(id INTEGER PRIMARY KEY, matchNumber INTEGER, round INTEGER, compLevel INTEGER, r1 INTEGER, r2 INTEGER, r3 INTEGER, b1 INTEGER, b2 INTEGER, b3 INTEGER, r1ID INTEGER, r2ID INTEGER, r3ID INTEGER, b1ID INTEGER, b2ID INTEGER, b3ID INTEGER)")
	defer statement.Close()
	if error_ != nil {
		fmt.Println(error_)
		return nil, error_
	}
	_, error_ = statement.Exec()
	statement, error_ = database.Prepare("CREATE TABLE IF NOT EXISTS team_match_stats (id INTEGER PRIMARY KEY, teamNumber INTEGER, matchNumber DOUBLE, shotsMissed INTEGER, upperGoalShots INTEGER, lowerGoalShots INTEGER, shotsMissedAuto INTEGER, upperGoalAuto INTEGER, lowerGoalAuto INTEGER, playedDefense INTEGER, climbing INTEGER)")
	defer statement.Close()
	if error_ != nil {
		fmt.Println(error_)
		return nil, error_
	}
	_, error_ = statement.Exec()
	return database, nil
}

func (database *Database) Delete() error {
	statement, error_ := database.Prepare("DROP TABLE IF EXISTS matches")
	if error_ != nil {
		fmt.Println(error_)
		return (error_)
	}
	_, error_ = statement.Exec()
	statement, error_ = database.Prepare("DROP TABLE IF EXISTS team_match_stats")
	if error_ != nil {
		fmt.Println(error_)
		return (error_)
	}
	_, error_ = statement.Exec()
	return nil
}

// This function will also populate the Stats table with six empty rows every time a match is added
func (database *Database) AddToMatch(m Match) error {
	statement, error_ := database.Prepare("INSERT INTO team_match_stats(teamNumber, matchNumber, shotsMissed, upperGoalShots, lowerGoalShots, shotsMissedAuto, upperGoalAuto, lowerGoalAuto, playedDefense, climbing) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)")
	defer statement.Close()
	if error_ != nil {
		fmt.Println("failed to prepare stats database:", error_)
		return (error_)
	}
	var rowIds [6]int64
	for i, teamNumber := range []int{m.r1, m.r2, m.r3, m.b1, m.b2, m.b3} {
		result, error_ := statement.Exec(teamNumber, m.matchNumber, 0, 0, 0, 0, 0, 0, 0, 0)
		if error_ != nil {
			fmt.Println("failed to execute statement 2:", error_)
			return (error_)
		}
		rowIds[i], error_ = result.LastInsertId()
	}
	statement, error_ = database.Prepare("INSERT INTO matches(matchNumber, round, compLevel, r1, r2, r3, b1, b2, b3, r1ID, r2ID, r3ID, b1ID, b2ID, b3ID) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)")
	defer statement.Close()
	if error_ != nil {
		fmt.Println("failed to prepare match database:", error_)
		return (error_)
	}
	_, error_ = statement.Exec(m.matchNumber, m.round, m.compLevel, m.r1, m.r2, m.r3, m.b1, m.b2, m.b3, rowIds[0], rowIds[1], rowIds[2], rowIds[3], rowIds[4], rowIds[5])
	if error_ != nil {
		fmt.Println(error_)
		return (error_)
	}
	return nil
}

func (database *Database) AddToStats(s Stats) error {
	statement, error_ := database.Prepare("UPDATE team_match_stats SET teamNumber = ?, matchNumber = ?, shotsMissed = ?, upperGoalShots = ?, lowerGoalShots = ?, shotsMissedAuto = ?, upperGoalAuto = ?, lowerGoalAuto = ?, playedDefense = ?, climbing = ? WHERE matchNumber = ? AND teamNumber = ?")
	if error_ != nil {
		fmt.Println(error_)
		return (error_)
	}
	_, error_ = statement.Exec(s.teamNumber, s.matchNumber, s.shotsMissed, s.upperGoalShots, s.lowerGoalShots, s.shotsMissedAuto, s.upperGoalAuto, s.lowerGoalAuto, s.playedDefense, s.climbing, s.matchNumber, s.teamNumber)
	if error_ != nil {
		fmt.Println(error_)
		return (error_)
	}
	return nil
}

func (database *Database) ReturnMatches() ([]Match, error) {
	matches := make([]Match, 0)
	rows, _ := database.Query("SELECT * FROM matches")
	defer rows.Close()
	for rows.Next() {
		var match Match
		var id int
		error_ := rows.Scan(&id, &match.matchNumber, &match.round, &match.compLevel, &match.r1, &match.r2, &match.r3, &match.b1, &match.b2, &match.b3, &match.r1ID, &match.r2ID, &match.r3ID, &match.b1ID, &match.b2ID, &match.b3ID)
		if error_ != nil {
			fmt.Println(nil, error_)
			return nil, error_
		}
		matches = append(matches, match)
	}
	return matches, nil
}

func (database *Database) ReturnStats() ([]Stats, error) {
	rows, _ := database.Query("SELECT * FROM team_match_stats")
	defer rows.Close()
	teams := make([]Stats, 0)
	var id int
	for rows.Next() {
		var team Stats
		error_ := rows.Scan(&id, &team.teamNumber, &team.matchNumber, &team.shotsMissed, &team.upperGoalShots, &team.lowerGoalShots, &team.shotsMissedAuto, &team.upperGoalAuto, &team.lowerGoalAuto, &team.playedDefense, &team.climbing)
		if error_ != nil {
			fmt.Println(error_)
			return nil, error_
		}
		teams = append(teams, team)
	}
	return teams, nil
}

func (database *Database) QueryMatches(teamNumber_ int) ([]Match, error) {
	rows, error_ := database.Query("SELECT * FROM matches WHERE r1 = ? OR r2 = ? OR r3 = ? OR b1 = ? OR b2 = ? OR b3 = ?", teamNumber_, teamNumber_, teamNumber_, teamNumber_, teamNumber_, teamNumber_)
	if error_ != nil {
		fmt.Println("failed to execute statement 1:", error_)
		return nil, error_
	}
	defer rows.Close()
	var matches []Match
	var id int
	for rows.Next() {
		var match Match
		rows.Scan(&id, &match.matchNumber, &match.round, &match.compLevel, &match.r1, &match.r2, &match.r3, &match.b1, &match.b2, &match.b3, &match.r1ID, &match.r2ID, &match.r3ID, &match.b1ID, &match.b2ID, &match.b3ID)
		matches = append(matches, match)
	}
	return matches, nil
}

func (database *Database) QueryStats(teamNumber_ int) ([]Stats, error) {
	rows, error_ := database.Query("SELECT * FROM team_match_stats WHERE teamNumber = ?", teamNumber_)
	if error_ != nil {
		fmt.Println("failed to execute statement 3:", error_)
		return nil, error_
	}
	defer rows.Close()
	var teams []Stats
	for rows.Next() {
		var team Stats
		var id int
		error_ = rows.Scan(&id, &team.teamNumber, &team.matchNumber, &team.shotsMissed,
			&team.upperGoalShots, &team.lowerGoalShots, &team.shotsMissedAuto, &team.upperGoalAuto,
			&team.lowerGoalAuto, &team.playedDefense, &team.climbing)
		teams = append(teams, team)
	}
	if error_ != nil {
		fmt.Println("failed to execute statement 3:", error_)
		return nil, error_
	}
	return teams, nil
}
