package db

import (
	"errors"
	"fmt"
	"gorm.io/driver/postgres"
	"gorm.io/gorm"
	"gorm.io/gorm/clause"
	"gorm.io/gorm/logger"
)

type Database struct {
	*gorm.DB
}

type TeamMatch struct {
	MatchNumber      int32  `gorm:"primaryKey"`
	SetNumber        int32  `gorm:"primaryKey"`
	CompLevel        string `gorm:"primaryKey"`
	Alliance         string `gorm:"primaryKey"` // "R" or "B"
	AlliancePosition int32  `gorm:"primaryKey"` // 1, 2, or 3
	TeamNumber       int32
}

type Shift struct {
	MatchNumber                                                      int32 `gorm:"primaryKey"`
	R1scouter, R2scouter, R3scouter, B1scouter, B2scouter, B3scouter string
}

type Stats struct {
	TeamNumber       int32  `gorm:"primaryKey"`
	MatchNumber      int32  `gorm:"primaryKey"`
	SetNumber        int32  `gorm:"primaryKey"`
	CompLevel        string `gorm:"primaryKey"`
	StartingQuadrant int32
	// This field is for the balls picked up during auto. Use this field
	// when using this library. Ignore the AutoBallPickedUpX fields below.
	AutoBallPickedUp [5]bool `gorm:"-:all"`
	// These fields are internal implementation details. Do not use these.
	// TODO(phil): Figure out how to use the JSON gorm serializer instead
	// of manually serializing/deserializing these.
	AutoBallPickedUp1 bool
	AutoBallPickedUp2 bool
	AutoBallPickedUp3 bool
	AutoBallPickedUp4 bool
	AutoBallPickedUp5 bool
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
	// 6 -> "Traversal"
	Climbing int32
	// Some non-numerical data that the scout felt worth noting.
	Comment string
	// The username of the person who collected these statistics.
	// "unknown" if submitted without logging in.
	// Empty if the stats have not yet been collected.
	CollectedBy string
}

type Stats2023 struct {
	TeamNumber                                                     string `gorm:"primaryKey"`
	MatchNumber                                                    int32  `gorm:"primaryKey"`
	SetNumber                                                      int32  `gorm:"primaryKey"`
	CompLevel                                                      string `gorm:"primaryKey"`
	StartingQuadrant                                               int32
	LowCubesAuto, MiddleCubesAuto, HighCubesAuto, CubesDroppedAuto int32
	LowConesAuto, MiddleConesAuto, HighConesAuto, ConesDroppedAuto int32
	LowCubes, MiddleCubes, HighCubes, CubesDropped                 int32
	LowCones, MiddleCones, HighCones, ConesDropped                 int32
	AvgCycle                                                       int32
	// The username of the person who collected these statistics.
	// "unknown" if submitted without logging in.
	// Empty if the stats have not yet been collected.
	CollectedBy string
}

type Action struct {
	TeamNumber      string `gorm:"primaryKey"`
	MatchNumber     int32  `gorm:"primaryKey"`
	SetNumber       int32  `gorm:"primaryKey"`
	CompLevel       string `gorm:"primaryKey"`
	CompletedAction []byte
	// This contains a serialized scouting.webserver.requests.ActionType flatbuffer.
	TimeStamp   int32 `gorm:"primaryKey"`
	CollectedBy string
}

type NotesData struct {
	ID           uint `gorm:"primaryKey"`
	TeamNumber   int32
	Notes        string
	GoodDriving  bool
	BadDriving   bool
	SketchyClimb bool
	SolidClimb   bool
	GoodDefense  bool
	BadDefense   bool
}

type Ranking struct {
	TeamNumber         int `gorm:"primaryKey"`
	Losses, Wins, Ties int32
	Rank, Dq           int32
}

type DriverRankingData struct {
	// Each entry in the table is a single scout's ranking.
	// Multiple scouts can submit a driver ranking for the same
	// teams in the same match.
	// The teams being ranked are stored in Rank1, Rank2, Rank3,
	// Rank1 being the best driving and Rank3 being the worst driving.

	ID          uint `gorm:"primaryKey"`
	MatchNumber int32
	Rank1       int32
	Rank2       int32
	Rank3       int32
}

// Opens a database at the specified port on localhost. We currently don't
// support connecting to databases on other hosts.
func NewDatabase(user string, password string, port int) (*Database, error) {
	var err error
	database := new(Database)

	dsn := fmt.Sprintf("host=localhost user=%s password=%s dbname=postgres port=%d sslmode=disable", user, password, port)
	database.DB, err = gorm.Open(postgres.Open(dsn), &gorm.Config{
		Logger: logger.Default.LogMode(logger.Silent),
	})
	if err != nil {
		database.Delete()
		return nil, errors.New(fmt.Sprint("Failed to connect to postgres: ", err))
	}

	err = database.AutoMigrate(&TeamMatch{}, &Shift{}, &Stats{}, &Stats2023{}, &Action{}, &NotesData{}, &Ranking{}, &DriverRankingData{})
	if err != nil {
		database.Delete()
		return nil, errors.New(fmt.Sprint("Failed to create/migrate tables: ", err))
	}

	return database, nil
}

func (database *Database) Delete() error {
	sql, err := database.DB.DB()
	if err != nil {
		return err
	}
	return sql.Close()
}

func (database *Database) SetDebugLogLevel() {
	database.DB.Logger = database.DB.Logger.LogMode(logger.Info)
}

func (database *Database) AddToMatch(m TeamMatch) error {
	result := database.Clauses(clause.OnConflict{
		UpdateAll: true,
	}).Create(&m)
	return result.Error
}

func (database *Database) AddToShift(sh Shift) error {
	result := database.Clauses(clause.OnConflict{
		UpdateAll: true,
	}).Create(&sh)
	return result.Error
}

func (database *Database) AddAction(a Action) error {
	result := database.Clauses(clause.OnConflict{
		UpdateAll: true,
	}).Create(&a)
	return result.Error
}

func (database *Database) AddToStats(s Stats) error {
	matches, err := database.queryMatches(s.TeamNumber)
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

	// Unpack the auto balls array.
	s.AutoBallPickedUp1 = s.AutoBallPickedUp[0]
	s.AutoBallPickedUp2 = s.AutoBallPickedUp[1]
	s.AutoBallPickedUp3 = s.AutoBallPickedUp[2]
	s.AutoBallPickedUp4 = s.AutoBallPickedUp[3]
	s.AutoBallPickedUp5 = s.AutoBallPickedUp[4]
	result := database.Create(&s)
	return result.Error
}

func (database *Database) AddToStats2023(s Stats2023) error {
	matches, err := database.QueryMatchesString(s.TeamNumber)
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

	result := database.Create(&s)
	return result.Error
}

func (database *Database) DeleteFromStats(compLevel_ string, matchNumber_ int32, setNumber_ int32, teamNumber_ string) error {
	var stats2023 []Stats2023
	result := database.
		Where("comp_level = ? AND match_number = ? AND set_number = ? AND team_number = ?", compLevel_, matchNumber_, setNumber_, teamNumber_).
		Delete(&stats2023)
	return result.Error
}

func (database *Database) AddOrUpdateRankings(r Ranking) error {
	result := database.Clauses(clause.OnConflict{
		UpdateAll: true,
	}).Create(&r)
	return result.Error
}

func (database *Database) ReturnMatches() ([]TeamMatch, error) {
	var matches []TeamMatch
	result := database.Find(&matches)
	return matches, result.Error
}

func (database *Database) ReturnAllNotes() ([]NotesData, error) {
	var notes []NotesData
	result := database.Find(&notes)
	return notes, result.Error
}

func (database *Database) ReturnAllDriverRankings() ([]DriverRankingData, error) {
	var rankings []DriverRankingData
	result := database.Find(&rankings)
	return rankings, result.Error
}

func (database *Database) ReturnAllShifts() ([]Shift, error) {
	var shifts []Shift
	result := database.Find(&shifts)
	return shifts, result.Error
}

func (database *Database) ReturnActions() ([]Action, error) {
	var actions []Action
	result := database.Find(&actions)
	return actions, result.Error
}

// Packs the stats. This really just consists of taking the individual auto
// ball booleans and turning them into an array. The individual booleans are
// cleared so that they don't affect struct comparisons.
func packStats(stats *Stats) {
	stats.AutoBallPickedUp = [5]bool{
		stats.AutoBallPickedUp1,
		stats.AutoBallPickedUp2,
		stats.AutoBallPickedUp3,
		stats.AutoBallPickedUp4,
		stats.AutoBallPickedUp5,
	}
	stats.AutoBallPickedUp1 = false
	stats.AutoBallPickedUp2 = false
	stats.AutoBallPickedUp3 = false
	stats.AutoBallPickedUp4 = false
	stats.AutoBallPickedUp5 = false
}

func (database *Database) ReturnStats() ([]Stats, error) {
	var stats []Stats
	result := database.Find(&stats)
	// Pack the auto balls array.
	for i := range stats {
		packStats(&stats[i])
	}
	return stats, result.Error
}

func (database *Database) ReturnStats2023() ([]Stats2023, error) {
	var stats2023 []Stats2023
	result := database.Find(&stats2023)
	return stats2023, result.Error
}

func (database *Database) ReturnRankings() ([]Ranking, error) {
	var rankins []Ranking
	result := database.Find(&rankins)
	return rankins, result.Error
}

func (database *Database) queryMatches(teamNumber_ int32) ([]TeamMatch, error) {
	var matches []TeamMatch
	result := database.
		Where("team_number = $1", teamNumber_).
		Find(&matches)
	return matches, result.Error
}

func (database *Database) QueryMatchesString(teamNumber_ string) ([]TeamMatch, error) {
	var matches []TeamMatch
	result := database.
		Where("team_number = $1", teamNumber_).
		Find(&matches)
	return matches, result.Error
}

func (database *Database) QueryAllShifts(matchNumber_ int) ([]Shift, error) {
	var shifts []Shift
	result := database.Where("match_number = ?", matchNumber_).Find(&shifts)
	return shifts, result.Error
}

func (database *Database) QueryStats(teamNumber_ int) ([]Stats, error) {
	var stats []Stats
	result := database.Where("team_number = ?", teamNumber_).Find(&stats)
	// Pack the auto balls array.
	for i := range stats {
		packStats(&stats[i])
	}
	return stats, result.Error
}

func (database *Database) QueryActions(teamNumber_ int) ([]Action, error) {
	var actions []Action
	result := database.
		Where("team_number = ?", teamNumber_).Find(&actions)
	return actions, result.Error
}

func (database *Database) QueryNotes(TeamNumber int32) ([]string, error) {
	var rawNotes []NotesData
	result := database.Where("team_number = ?", TeamNumber).Find(&rawNotes)
	if result.Error != nil {
		return nil, result.Error
	}

	notes := make([]string, len(rawNotes))
	for i := range rawNotes {
		notes[i] = rawNotes[i].Notes
	}
	return notes, nil
}

func (database *Database) QueryRankings(TeamNumber int) ([]Ranking, error) {
	var rankins []Ranking
	result := database.Where("team_number = ?", TeamNumber).Find(&rankins)
	return rankins, result.Error
}

func (database *Database) AddNotes(data NotesData) error {
	result := database.Create(&NotesData{
		TeamNumber:   data.TeamNumber,
		Notes:        data.Notes,
		GoodDriving:  data.GoodDriving,
		BadDriving:   data.BadDriving,
		SketchyClimb: data.SketchyClimb,
		SolidClimb:   data.SolidClimb,
		GoodDefense:  data.GoodDefense,
		BadDefense:   data.BadDefense,
	})
	return result.Error
}

func (database *Database) AddDriverRanking(data DriverRankingData) error {
	result := database.Create(&DriverRankingData{
		MatchNumber: data.MatchNumber,
		Rank1:       data.Rank1,
		Rank2:       data.Rank2,
		Rank3:       data.Rank3,
	})
	return result.Error
}

func (database *Database) QueryDriverRanking(MatchNumber int) ([]DriverRankingData, error) {
	var data []DriverRankingData
	result := database.Where("match_number = ?", MatchNumber).Find(&data)
	return data, result.Error
}
