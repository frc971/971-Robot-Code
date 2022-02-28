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
	MatchNumber, Round     int
	CompLevel              string
	R1, R2, R3, B1, B2, B3 int
	// Each of these variables holds the matchID of the corresponding Stats row
	r1ID, r2ID, r3ID, b1ID, b2ID, b3ID int
}

type Stats struct {
	teamNumber, MatchNumber                                      int
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
		"(id INTEGER PRIMARY KEY, MatchNumber INTEGER, Round INTEGER, CompLevel INTEGER, R1 INTEGER, R2 INTEGER, R3 INTEGER, B1 INTEGER, B2 INTEGER, B3 INTEGER, r1ID INTEGER, r2ID INTEGER, r3ID INTEGER, b1ID INTEGER, b2ID INTEGER, b3ID INTEGER)")
	defer statement.Close()
	if error_ != nil {
		fmt.Println(error_)
		return nil, error_
	}
	_, error_ = statement.Exec()
	statement, error_ = database.Prepare("CREATE TABLE IF NOT EXISTS team_match_stats (id INTEGER PRIMARY KEY, teamNumber INTEGER, MatchNumber DOUBLE, shotsMissed INTEGER, upperGoalShots INTEGER, lowerGoalShots INTEGER, shotsMissedAuto INTEGER, upperGoalAuto INTEGER, lowerGoalAuto INTEGER, playedDefense INTEGER, climbing INTEGER)")
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
	statement, error_ := database.Prepare("INSERT INTO team_match_stats(teamNumber, MatchNumber, shotsMissed, upperGoalShots, lowerGoalShots, shotsMissedAuto, upperGoalAuto, lowerGoalAuto, playedDefense, climbing) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)")
	defer statement.Close()
	if error_ != nil {
		fmt.Println("failed to prepare stats database:", error_)
		return (error_)
	}
	var rowIds [6]int64
	for i, teamNumber := range []int{m.R1, m.R2, m.R3, m.B1, m.B2, m.B3} {
		result, error_ := statement.Exec(teamNumber, m.MatchNumber, 0, 0, 0, 0, 0, 0, 0, 0)
		if error_ != nil {
			fmt.Println("failed to execute statement 2:", error_)
			return (error_)
		}
		rowIds[i], error_ = result.LastInsertId()
	}
	statement, error_ = database.Prepare("INSERT INTO matches(MatchNumber, Round, CompLevel, R1, R2, R3, B1, B2, B3, r1ID, r2ID, r3ID, b1ID, b2ID, b3ID) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)")
	defer statement.Close()
	if error_ != nil {
		fmt.Println("failed to prepare match database:", error_)
		return (error_)
	}
	_, error_ = statement.Exec(m.MatchNumber, m.Round, m.CompLevel, m.R1, m.R2, m.R3, m.B1, m.B2, m.B3, rowIds[0], rowIds[1], rowIds[2], rowIds[3], rowIds[4], rowIds[5])
	if error_ != nil {
		fmt.Println(error_)
		return (error_)
	}
	return nil
}

func (database *Database) AddToStats(s Stats) error {
	statement, error_ := database.Prepare("UPDATE team_match_stats SET teamNumber = ?, MatchNumber = ?, shotsMissed = ?, upperGoalShots = ?, lowerGoalShots = ?, shotsMissedAuto = ?, upperGoalAuto = ?, lowerGoalAuto = ?, playedDefense = ?, climbing = ? WHERE MatchNumber = ? AND teamNumber = ?")
	if error_ != nil {
		fmt.Println(error_)
		return (error_)
	}
	_, error_ = statement.Exec(s.teamNumber, s.MatchNumber, s.shotsMissed, s.upperGoalShots, s.lowerGoalShots, s.shotsMissedAuto, s.upperGoalAuto, s.lowerGoalAuto, s.playedDefense, s.climbing, s.MatchNumber, s.teamNumber)
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
		error_ := rows.Scan(&id, &match.MatchNumber, &match.Round, &match.CompLevel, &match.R1, &match.R2, &match.R3, &match.B1, &match.B2, &match.B3, &match.r1ID, &match.r2ID, &match.r3ID, &match.b1ID, &match.b2ID, &match.b3ID)
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
		error_ := rows.Scan(&id, &team.teamNumber, &team.MatchNumber, &team.shotsMissed, &team.upperGoalShots, &team.lowerGoalShots, &team.shotsMissedAuto, &team.upperGoalAuto, &team.lowerGoalAuto, &team.playedDefense, &team.climbing)
		if error_ != nil {
			fmt.Println(error_)
			return nil, error_
		}
		teams = append(teams, team)
	}
	return teams, nil
}

func (database *Database) QueryMatches(teamNumber_ int) ([]Match, error) {
	rows, error_ := database.Query("SELECT * FROM matches WHERE R1 = ? OR R2 = ? OR R3 = ? OR B1 = ? OR B2 = ? OR B3 = ?", teamNumber_, teamNumber_, teamNumber_, teamNumber_, teamNumber_, teamNumber_)
	if error_ != nil {
		fmt.Println("failed to execute statement 1:", error_)
		return nil, error_
	}
	defer rows.Close()
	var matches []Match
	var id int
	for rows.Next() {
		var match Match
		rows.Scan(&id, &match.MatchNumber, &match.Round, &match.CompLevel, &match.R1, &match.R2, &match.R3, &match.B1, &match.B2, &match.B3, &match.r1ID, &match.r2ID, &match.r3ID, &match.b1ID, &match.b2ID, &match.b3ID)
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
		error_ = rows.Scan(&id, &team.teamNumber, &team.MatchNumber, &team.shotsMissed,
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
