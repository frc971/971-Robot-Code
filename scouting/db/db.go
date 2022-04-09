package db

import (
	"database/sql"
	"errors"
	"fmt"

	_ "github.com/jackc/pgx/stdlib"
)

type Database struct {
	*sql.DB
}

type Match struct {
	MatchNumber, Round     int32
	CompLevel              string
	R1, R2, R3, B1, B2, B3 int32
}

type Stats struct {
	TeamNumber, MatchNumber int32
	StartingQuadrant        int32
	AutoBallPickedUp        [5]bool
	// TODO(phil): Re-order auto and teleop fields so auto comes first.
	ShotsMissed, UpperGoalShots, LowerGoalShots   int32
	ShotsMissedAuto, UpperGoalAuto, LowerGoalAuto int32
	PlayedDefense, DefenseReceivedScore           int32
	// Climbing level:
	// 0 -> "NoAttempt"
	// 1 -> "Failed"
	// 2 -> "FailedWithPlentyOfTime"
	// 3 -> "Low"
	// 4 -> "Medium"
	// 5 -> "High"
	// 6 -> "Transversal"
	Climbing int32
	// Some non-numerical data that the scout felt worth noting.
	Comment string
	// The username of the person who collected these statistics.
	// "unknown" if submitted without logging in.
	// Empty if the stats have not yet been collected.
	CollectedBy string
}

type NotesData struct {
	TeamNumber int32
	Notes      []string
}

type Ranking struct {
	TeamNumber         int
	Losses, Wins, Ties int32
	Rank, Dq           int32
}

// Opens a database at the specified port on localhost. We currently don't
// support connecting to databases on other hosts.
func NewDatabase(user string, password string, port int) (*Database, error) {
	var err error
	database := new(Database)

	psqlInfo := fmt.Sprintf("postgres://%s:%s@localhost:%d/postgres", user, password, port)
	database.DB, err = sql.Open("pgx", psqlInfo)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to connect to postgres: ", err))
	}

	statement, err := database.Prepare("CREATE TABLE IF NOT EXISTS matches (" +
		"MatchNumber INTEGER, " +
		"Round INTEGER, " +
		"CompLevel VARCHAR, " +
		"R1 INTEGER, " +
		"R2 INTEGER, " +
		"R3 INTEGER, " +
		"B1 INTEGER, " +
		"B2 INTEGER, " +
		"B3 INTEGER, " +
		"PRIMARY KEY (MatchNumber, Round, CompLevel))")
	if err != nil {
		database.Close()
		return nil, errors.New(fmt.Sprint("Failed to prepare matches table creation: ", err))
	}
	defer statement.Close()

	_, err = statement.Exec()
	if err != nil {
		database.Close()
		return nil, errors.New(fmt.Sprint("Failed to create matches table: ", err))
	}

	statement, err = database.Prepare("CREATE TABLE IF NOT EXISTS team_match_stats (" +
		"id SERIAL PRIMARY KEY, " +
		"TeamNumber INTEGER, " +
		"MatchNumber INTEGER, " +
		"StartingQuadrant INTEGER, " +
		"AutoBall1PickedUp BOOLEAN, " +
		"AutoBall2PickedUp BOOLEAN, " +
		"AutoBall3PickedUp BOOLEAN, " +
		"AutoBall4PickedUp BOOLEAN, " +
		"AutoBall5PickedUp BOOLEAN, " +
		"ShotsMissed INTEGER, " +
		"UpperGoalShots INTEGER, " +
		"LowerGoalShots INTEGER, " +
		"ShotsMissedAuto INTEGER, " +
		"UpperGoalAuto INTEGER, " +
		"LowerGoalAuto INTEGER, " +
		"PlayedDefense INTEGER, " +
		"DefenseReceivedScore INTEGER, " +
		"Climbing INTEGER, " +
		"Comment VARCHAR, " +
		"CollectedBy VARCHAR)")
	if err != nil {
		database.Close()
		return nil, errors.New(fmt.Sprint("Failed to prepare stats table creation: ", err))
	}
	defer statement.Close()

	_, err = statement.Exec()
	if err != nil {
		database.Close()
		return nil, errors.New(fmt.Sprint("Failed to create team_match_stats table: ", err))
	}

	statement, err = database.Prepare("CREATE TABLE IF NOT EXISTS team_notes (" +
		"id SERIAL PRIMARY KEY, " +
		"TeamNumber INTEGER, " +
		"Notes TEXT)")
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to prepare notes table creation: ", err))
	}
	defer statement.Close()

	_, err = statement.Exec()
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to create notes table: ", err))
	}

	statement, err = database.Prepare("CREATE TABLE IF NOT EXISTS rankings (" +
		"id SERIAL PRIMARY KEY, " +
		"Losses INTEGER, " +
		"Wins INTEGER, " +
		"Ties INTEGER, " +
		"Rank INTEGER, " +
		"Dq INTEGER, " +
		"TeamNumber INTEGER)")
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to prepare rankings table creation: ", err))
	}
	defer statement.Close()

	_, err = statement.Exec()
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to create rankings table: ", err))
	}

	return database, nil
}

func (database *Database) Delete() error {
	statement, err := database.Prepare("DROP TABLE IF EXISTS matches")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare dropping matches table: ", err))
	}
	_, err = statement.Exec()
	if err != nil {
		return errors.New(fmt.Sprint("Failed to drop matches table: ", err))
	}

	statement, err = database.Prepare("DROP TABLE IF EXISTS team_match_stats")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare dropping stats table: ", err))
	}
	_, err = statement.Exec()
	if err != nil {
		return errors.New(fmt.Sprint("Failed to drop stats table: ", err))
	}

	statement, err = database.Prepare("DROP TABLE IF EXISTS team_notes")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare dropping notes table: ", err))
	}
	_, err = statement.Exec()
	if err != nil {
		return errors.New(fmt.Sprint("Failed to drop notes table: ", err))
	}
	return nil

	statement, err = database.Prepare("DROP TABLE IF EXISTS rankings")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare dropping rankings table: ", err))
	}
	_, err = statement.Exec()
	if err != nil {
		return errors.New(fmt.Sprint("Failed to drop rankings table: ", err))
	}
	return nil
}

// This function will also populate the Stats table with six empty rows every time a match is added
func (database *Database) AddToMatch(m Match) error {
	statement, err := database.Prepare("INSERT INTO matches(" +
		"MatchNumber, Round, CompLevel, " +
		"R1, R2, R3, B1, B2, B3) " +
		"VALUES (" +
		"$1, $2, $3, " +
		"$4, $5, $6, $7, $8, $9)")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare insertion into match database: ", err))
	}
	defer statement.Close()

	_, err = statement.Exec(m.MatchNumber, m.Round, m.CompLevel,
		m.R1, m.R2, m.R3, m.B1, m.B2, m.B3)
	if err != nil {
		return errors.New(fmt.Sprint("Failed to insert into match database: ", err))
	}
	return nil
}

func (database *Database) AddToStats(s Stats) error {
	matches, err := database.QueryMatches(s.TeamNumber)
	if err != nil {
		return err
	}
	foundMatch := false
	for _, match := range matches {
		if match.MatchNumber == s.MatchNumber {
			foundMatch = true
			break
		}
	}
	if !foundMatch {
		return errors.New(fmt.Sprint(
			"Failed to find team ", s.TeamNumber,
			" in match ", s.MatchNumber, " in the schedule."))
	}

	statement, err := database.Prepare("INSERT INTO team_match_stats(" +
		"TeamNumber, MatchNumber, " +
		"StartingQuadrant, " +
		"AutoBall1PickedUp, AutoBall2PickedUp, AutoBall3PickedUp, " +
		"AutoBall4PickedUp, AutoBall5PickedUp, " +
		"ShotsMissed, UpperGoalShots, LowerGoalShots, " +
		"ShotsMissedAuto, UpperGoalAuto, LowerGoalAuto, " +
		"PlayedDefense, DefenseReceivedScore, Climbing, " +
		"Comment, CollectedBy) " +
		"VALUES (" +
		"$1, $2, " +
		"$3, " +
		"$4, $5, $6, " +
		"$7, $8, " +
		"$9, $10, $11, " +
		"$12, $13, $14, " +
		"$15, $16, $17, " +
		"$18, $19)")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare stats update statement: ", err))
	}
	defer statement.Close()

	_, err = statement.Exec(
		s.TeamNumber, s.MatchNumber,
		s.StartingQuadrant,
		s.AutoBallPickedUp[0], s.AutoBallPickedUp[1], s.AutoBallPickedUp[2],
		s.AutoBallPickedUp[3], s.AutoBallPickedUp[4],
		s.ShotsMissed, s.UpperGoalShots, s.LowerGoalShots,
		s.ShotsMissedAuto, s.UpperGoalAuto, s.LowerGoalAuto,
		s.PlayedDefense, s.DefenseReceivedScore, s.Climbing,
		s.Comment, s.CollectedBy)
	if err != nil {
		return errors.New(fmt.Sprint("Failed to update stats database: ", err))
	}

	return nil
}

func (database *Database) AddOrUpdateRankings(r Ranking) error {
	statement, err := database.Prepare("UPDATE rankings SET " +
		"Losses = $1, Wins = $2, Ties = $3, " +
		"Rank = $4, Dq = $5, TeamNumber = $6 " +
		"WHERE TeamNumber = $6")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare rankings database update: ", err))
	}
	defer statement.Close()

	result, err := statement.Exec(r.Losses, r.Wins, r.Ties,
		r.Rank, r.Dq, r.TeamNumber)
	if err != nil {
		return errors.New(fmt.Sprint("Failed to update rankings database: ", err))
	}

	numRowsAffected, err := result.RowsAffected()
	if err != nil {
		return errors.New(fmt.Sprint("Failed to query rows affected: ", err))
	}
	if numRowsAffected == 0 {
		statement, err := database.Prepare("INSERT INTO rankings(" +
			"Losses, Wins, Ties, " +
			"Rank, Dq, TeamNumber) " +
			"VALUES (" +
			"$1, $2, $3, " +
			"$4, $5, $6)")
		if err != nil {
			return errors.New(fmt.Sprint("Failed to prepare insertion into rankings database: ", err))
		}
		defer statement.Close()

		_, err = statement.Exec(r.Losses, r.Wins, r.Ties,
			r.Rank, r.Dq, r.TeamNumber)
		if err != nil {
			return errors.New(fmt.Sprint("Failed to insert into rankings database: ", err))
		}
	}

	return nil
}

func (database *Database) ReturnMatches() ([]Match, error) {
	rows, err := database.Query("SELECT * FROM matches")
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to select from matches: ", err))
	}
	defer rows.Close()

	matches := make([]Match, 0)
	for rows.Next() {
		var match Match
		err := rows.Scan(&match.MatchNumber, &match.Round, &match.CompLevel,
			&match.R1, &match.R2, &match.R3, &match.B1, &match.B2, &match.B3)
		if err != nil {
			return nil, errors.New(fmt.Sprint("Failed to scan from matches: ", err))
		}
		matches = append(matches, match)
	}
	return matches, nil
}

func (database *Database) ReturnStats() ([]Stats, error) {
	rows, err := database.Query("SELECT * FROM team_match_stats")
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to SELECT * FROM team_match_stats: ", err))
	}
	defer rows.Close()

	teams := make([]Stats, 0)
	for rows.Next() {
		var team Stats
		var id int
		err = rows.Scan(&id,
			&team.TeamNumber, &team.MatchNumber,
			&team.StartingQuadrant,
			&team.AutoBallPickedUp[0], &team.AutoBallPickedUp[1], &team.AutoBallPickedUp[2],
			&team.AutoBallPickedUp[3], &team.AutoBallPickedUp[4],
			&team.ShotsMissed, &team.UpperGoalShots, &team.LowerGoalShots,
			&team.ShotsMissedAuto, &team.UpperGoalAuto, &team.LowerGoalAuto,
			&team.PlayedDefense, &team.DefenseReceivedScore, &team.Climbing,
			&team.Comment, &team.CollectedBy)
		if err != nil {
			return nil, errors.New(fmt.Sprint("Failed to scan from stats: ", err))
		}
		teams = append(teams, team)
	}
	return teams, nil
}

func (database *Database) ReturnRankings() ([]Ranking, error) {
	rows, err := database.Query("SELECT * FROM rankings")
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to SELECT * FROM rankings: ", err))
	}
	defer rows.Close()

	all_rankings := make([]Ranking, 0)
	for rows.Next() {
		var ranking Ranking
		var id int
		err = rows.Scan(&id,
			&ranking.Losses, &ranking.Wins, &ranking.Ties,
			&ranking.Rank, &ranking.Dq, &ranking.TeamNumber)
		if err != nil {
			return nil, errors.New(fmt.Sprint("Failed to scan from rankings: ", err))
		}
		all_rankings = append(all_rankings, ranking)
	}
	return all_rankings, nil
}

func (database *Database) QueryMatches(teamNumber_ int32) ([]Match, error) {
	rows, err := database.Query("SELECT * FROM matches WHERE "+
		"R1 = $1 OR R2 = $2 OR R3 = $3 OR B1 = $4 OR B2 = $5 OR B3 = $6",
		teamNumber_, teamNumber_, teamNumber_, teamNumber_, teamNumber_, teamNumber_)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to select from matches for team: ", err))
	}
	defer rows.Close()

	var matches []Match
	for rows.Next() {
		var match Match
		err = rows.Scan(&match.MatchNumber, &match.Round, &match.CompLevel,
			&match.R1, &match.R2, &match.R3, &match.B1, &match.B2, &match.B3)
		if err != nil {
			return nil, errors.New(fmt.Sprint("Failed to scan from matches: ", err))
		}
		matches = append(matches, match)
	}
	return matches, nil
}

func (database *Database) QueryStats(teamNumber_ int) ([]Stats, error) {
	rows, err := database.Query("SELECT * FROM team_match_stats WHERE TeamNumber = $1", teamNumber_)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to select from stats: ", err))
	}
	defer rows.Close()

	var teams []Stats
	for rows.Next() {
		var team Stats
		var id int
		err = rows.Scan(&id,
			&team.TeamNumber, &team.MatchNumber,
			&team.StartingQuadrant,
			&team.AutoBallPickedUp[0], &team.AutoBallPickedUp[1], &team.AutoBallPickedUp[2],
			&team.AutoBallPickedUp[3], &team.AutoBallPickedUp[4],
			&team.ShotsMissed, &team.UpperGoalShots, &team.LowerGoalShots,
			&team.ShotsMissedAuto, &team.UpperGoalAuto, &team.LowerGoalAuto,
			&team.PlayedDefense, &team.DefenseReceivedScore, &team.Climbing,
			&team.Comment, &team.CollectedBy)
		if err != nil {
			return nil, errors.New(fmt.Sprint("Failed to scan from stats: ", err))
		}
		teams = append(teams, team)
	}
	return teams, nil
}

func (database *Database) QueryNotes(TeamNumber int32) (NotesData, error) {
	rows, err := database.Query("SELECT * FROM team_notes WHERE TeamNumber = $1", TeamNumber)
	if err != nil {
		return NotesData{}, errors.New(fmt.Sprint("Failed to select from notes: ", err))
	}
	defer rows.Close()

	var notes []string
	for rows.Next() {
		var id int32
		var data string
		err = rows.Scan(&id, &TeamNumber, &data)
		if err != nil {
			return NotesData{}, errors.New(fmt.Sprint("Failed to scan from notes: ", err))
		}
		notes = append(notes, data)
	}
	return NotesData{TeamNumber, notes}, nil
}

func (database *Database) QueryRankings(TeamNumber int) ([]Ranking, error) {
	rows, err := database.Query("SELECT * FROM rankings WHERE TeamNumber = $1", TeamNumber)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to select from rankings: ", err))
	}
	defer rows.Close()

	all_rankings := make([]Ranking, 0)
	for rows.Next() {
		var ranking Ranking
		var id int
		err = rows.Scan(&id,
			&ranking.Losses, &ranking.Wins, &ranking.Ties,
			&ranking.Rank, &ranking.Dq, &ranking.TeamNumber)
		if err != nil {
			return nil, errors.New(fmt.Sprint("Failed to scan from rankings: ", err))
		}
		all_rankings = append(all_rankings, ranking)
	}
	return all_rankings, nil
}

func (database *Database) AddNotes(data NotesData) error {
	if len(data.Notes) > 1 {
		return errors.New("Can only insert one row of notes at a time")
	}
	statement, err := database.Prepare("INSERT INTO " +
		"team_notes(TeamNumber, Notes)" +
		"VALUES ($1, $2)")
	if err != nil {
		return errors.New(fmt.Sprint("Failed to prepare insertion into notes table: ", err))
	}
	defer statement.Close()

	_, err = statement.Exec(data.TeamNumber, data.Notes[0])
	if err != nil {
		return errors.New(fmt.Sprint("Failed to insert into Notes database: ", err))
	}
	return nil
}
