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

type Match struct {
	// TODO(phil): Rework this be be one team per row.
	// Makes queries much simpler.
	MatchNumber            int32  `gorm:"primaryKey"`
	SetNumber              int32  `gorm:"primaryKey"`
	CompLevel              string `gorm:"primaryKey"`
	R1, R2, R3, B1, B2, B3 int32
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

	err = database.AutoMigrate(&Match{}, &Shift{}, &Stats{}, &NotesData{}, &Ranking{})
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

func (database *Database) AddToMatch(m Match) error {
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

	// Unpack the auto balls array.
	s.AutoBallPickedUp1 = s.AutoBallPickedUp[0]
	s.AutoBallPickedUp2 = s.AutoBallPickedUp[1]
	s.AutoBallPickedUp3 = s.AutoBallPickedUp[2]
	s.AutoBallPickedUp4 = s.AutoBallPickedUp[3]
	s.AutoBallPickedUp5 = s.AutoBallPickedUp[4]
	result := database.Create(&s)
	return result.Error
}

func (database *Database) AddOrUpdateRankings(r Ranking) error {
	result := database.Clauses(clause.OnConflict{
		UpdateAll: true,
	}).Create(&r)
	return result.Error
}

func (database *Database) ReturnMatches() ([]Match, error) {
	var matches []Match
	result := database.Find(&matches)
	return matches, result.Error
}

func (database *Database) ReturnAllShifts() ([]Shift, error) {
	var shifts []Shift
	result := database.Find(&shifts)
	return shifts, result.Error
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

func (database *Database) ReturnRankings() ([]Ranking, error) {
	var rankins []Ranking
	result := database.Find(&rankins)
	return rankins, result.Error
}

func (database *Database) QueryMatches(teamNumber_ int32) ([]Match, error) {
	var matches []Match
	result := database.
		Where("r1 = $1 OR r2 = $1 OR r3 = $1 OR b1 = $1 OR b2 = $1 OR b3 = $1", teamNumber_).
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
