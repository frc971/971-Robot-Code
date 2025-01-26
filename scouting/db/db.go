package db

import (
	"crypto/sha256"
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
	TeamNumber       string
}

type TeamMatch2025 struct {
	MatchNumber      int32  `gorm:"primaryKey"`
	CompCode         string `gorm:"primaryKey"`
	SetNumber        int32  `gorm:"primaryKey"`
	CompLevel        string `gorm:"primaryKey"`
	Alliance         string `gorm:"primaryKey"` // "R" or "B"
	AlliancePosition int32  `gorm:"primaryKey"` // 1, 2, or 3
	TeamNumber       string
}

type Shift struct {
	MatchNumber                                                      int32 `gorm:"primaryKey"`
	R1scouter, R2scouter, R3scouter, B1scouter, B2scouter, B3scouter string
}

type PitImage struct {
	TeamNumber string `gorm:"primaryKey"`
	CheckSum   string `gorm:"primaryKey"`
	ImagePath  string
	ImageData  []byte
}

type RequestedPitImage struct {
	TeamNumber string
	CheckSum   string `gorm:"primaryKey"`
	ImagePath  string
}

type Stats2024 struct {
	TeamNumber                                  string `gorm:"primaryKey"`
	MatchNumber                                 int32  `gorm:"primaryKey"`
	SetNumber                                   int32  `gorm:"primaryKey"`
	CompLevel                                   string `gorm:"primaryKey"`
	CompType                                    string `gorm:"primaryKey"`
	StartingQuadrant                            int32
	SpeakerAuto, AmpAuto                        int32
	NotesDroppedAuto                            int32
	MobilityAuto                                bool
	Speaker, Amp, SpeakerAmplified              int32
	NotesDropped                                int32
	Shuttled, OutOfField                        int32
	Penalties                                   int32
	AvgCycle                                    int64
	RobotDied                                   bool
	Park, OnStage, Harmony, TrapNote, Spotlight bool
	NoShow                                      bool

	// The username of the person who collected these statistics.
	// "unknown" if submitted without logging in.
	// Empty if the stats have not yet been collected.
	CollectedBy string
}

type Stats2025 struct {
	CompCode                           string `gorm:"primaryKey"`
	TeamNumber                         string `gorm:"primaryKey"`
	MatchNumber                        int32  `gorm:"primaryKey"`
	SetNumber                          int32  `gorm:"primaryKey"`
	CompLevel                          string `gorm:"primaryKey"`
	CompType                           string `gorm:"primaryKey"`
	StartingQuadrant                   int32
	L1Auto, L2Auto, L3Auto, L4Auto     int32
	NetAuto, ProcessorAuto             int32
	CoralDroppedAuto, AlgaeDroppedAuto int32
	CoralMissedAuto, AlgaeMissedAuto   int32
	MobilityAuto                       bool
	//teleop
	L1Teleop, L2Teleop, L3Teleop, L4Teleop   int32
	ProcessorTeleop, NetTeleop               int32
	CoralDroppedTeleop, AlgaeDroppedTeleop   int32
	CoralMissedTeleop, AlgaeMissedTeleop     int32
	AvgCycle                                 int64
	RobotDied                                bool
	Park, ShallowCage, DeepCage, BuddieClimb bool
	NoShow                                   bool
	CollectedBy                              string
}

type Action struct {
	TeamNumber  string `gorm:"primaryKey"`
	MatchNumber int32  `gorm:"primaryKey"`
	SetNumber   int32  `gorm:"primaryKey"`
	CompLevel   string `gorm:"primaryKey"`
	CompType    string `gorm:"primaryKey"`
	// This contains a serialized scouting.webserver.requests.ActionType flatbuffer.
	CompletedAction []byte
	Timestamp       int64 `gorm:"primaryKey"`
	CollectedBy     string
}

type NotesData struct {
	ID             uint `gorm:"primaryKey"`
	TeamNumber     string
	MatchNumber    int32
	SetNumber      int32
	CompLevel      string
	Notes          string
	GoodDriving    bool
	BadDriving     bool
	SolidPlacing   bool
	SketchyPlacing bool
	GoodDefense    bool
	BadDefense     bool
	EasilyDefended bool
	NoShow         bool
}

type Ranking struct {
	TeamNumber         string `gorm:"primaryKey"`
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
	Rank1       string
	Rank2       string
	Rank3       string
}

type DriverRanking2025 struct {
	// Each Driver Ranking data is the ranking for one team
	ID          uint `gorm:"primaryKey"`
	CompCode    string
	MatchNumber int32
	TeamNumber  string
	Score       int32
}

type HumanRanking2025 struct {
	// Each Human Ranking data is the ranking for one team
	ID          uint `gorm:"primaryKey"`
	CompCode    string
	MatchNumber int32
	TeamNumber  string
	Score       int32
}

type ParsedDriverRankingData struct {
	// This data stores the output of DriverRank.jl.

	TeamNumber string `gorm:"primaryKey"`

	// The score of the team. A difference of 100 in two team's scores
	// indicates that one team will outperform the other in 90% of the
	// matches.
	Score float32
}

type NotesData2025 struct {
	ID                   uint `gorm:"primaryKey"`
	CompCode             string
	TeamNumber           string
	MatchNumber          int32
	SetNumber            int32
	CompLevel            string
	Notes                string
	GoodDriving          bool
	BadDriving           bool
	CoralGroundIntake    bool
	CoralHpIntake        bool
	AlgaeGroundIntake    bool
	SolidAlgaeShooting   bool
	SketchyAlgaeShooting bool
	SolidCoralShooting   bool
	SketchyCoralShooting bool
	ShuffleCoral         bool
	Penalties            bool
	GoodDefense          bool
	BadDefense           bool
	EasilyDefended       bool
	NoShow               bool
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

	err = database.AutoMigrate(&TeamMatch{}, &TeamMatch2025{}, &Shift{}, &Stats2024{}, &Stats2025{}, &Action{}, &PitImage{}, &NotesData{}, &NotesData2025{}, &Ranking{}, &DriverRankingData{}, &DriverRanking2025{}, &HumanRanking2025{}, &ParsedDriverRankingData{})
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

func (database *Database) AddToMatch2025(m TeamMatch2025) error {
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
	// TODO(phil): Add check for a corresponding match in the `TeamMatch`
	// table.
	result := database.Create(&a)
	return result.Error
}

func (database *Database) AddPitImage(p PitImage) error {
	result := database.Create(&p)
	return result.Error
}

func (database *Database) AddToStats2024(s Stats2024) error {
	if s.CompType == "Regular" {
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
	}

	result := database.Create(&s)
	return result.Error
}

func (database *Database) AddToStats2025(s Stats2025) error {
	if s.CompType == "Regular" {
		matches, err := database.queryMatches2025(s.TeamNumber, s.CompCode)
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
	}

	result := database.Create(&s)
	return result.Error
}

func (database *Database) DeleteFromStats2024(compLevel_ string, matchNumber_ int32, setNumber_ int32, teamNumber_ string) error {
	var stats2024 []Stats2024
	result := database.
		Where("comp_level = ? AND match_number = ? AND set_number = ? AND team_number = ?", compLevel_, matchNumber_, setNumber_, teamNumber_).
		Delete(&stats2024)
	return result.Error
}

func (database *Database) DeleteFromStats2025(compCode_ string, compLevel_ string, matchNumber_ int32, setNumber_ int32, teamNumber_ string) error {
	var stats2025 []Stats2025
	result := database.
		Where("comp_code = ? AND comp_level = ? AND match_number = ? AND set_number = ? AND team_number = ?", compCode_, compLevel_, matchNumber_, setNumber_, teamNumber_).
		Delete(&stats2025)
	return result.Error
}

func (database *Database) DeleteFromActions(compLevel_ string, matchNumber_ int32, setNumber_ int32, teamNumber_ string) error {
	var actions []Action
	result := database.
		Where("comp_level = ? AND match_number = ? AND set_number = ? AND team_number = ?", compLevel_, matchNumber_, setNumber_, teamNumber_).
		Delete(&actions)
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

func (database *Database) ReturnAllNotes2025(CompCode string) ([]NotesData2025, error) {
	var notes []NotesData2025
	result := database.Where("comp_code = ?", CompCode).Order("match_number").Find(&notes)
	return notes, result.Error
}

func (database *Database) ReturnAllDriverRankings() ([]DriverRankingData, error) {
	var rankings []DriverRankingData
	result := database.Find(&rankings)
	return rankings, result.Error
}

func (database *Database) ReturnAllParsedDriverRankings() ([]ParsedDriverRankingData, error) {
	var rankings []ParsedDriverRankingData
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

func (database *Database) ReturnPitImages() ([]PitImage, error) {
	var images []PitImage
	result := database.Find(&images)
	return images, result.Error
}

func (database *Database) ReturnStats2024() ([]Stats2024, error) {
	var stats2024 []Stats2024
	result := database.Find(&stats2024)
	return stats2024, result.Error
}

func (database *Database) QueryStats2025(CompCode string) ([]Stats2025, error) {
	var stats2025 []Stats2025
	result := database.
		Where("comp_code = ? ",
			CompCode).
		Find(&stats2025).Order("team_number").Order("match_number")
	return stats2025, result.Error
}

func (database *Database) ReturnStats2024ForTeam(teamNumber string, matchNumber int32, setNumber int32, compLevel string, compType string) ([]Stats2024, error) {
	var stats2024 []Stats2024
	result := database.
		Where("team_number = ? AND match_number = ? AND set_number = ? AND comp_level = ? AND comp_type = ?",
			teamNumber, matchNumber, setNumber, compLevel, compType).
		Find(&stats2024)
	return stats2024, result.Error
}

func (database *Database) ReturnStats2025ForTeam(compCode string, teamNumber string, matchNumber int32, setNumber int32, compLevel string, compType string) ([]Stats2025, error) {
	var stats2025 []Stats2025
	result := database.
		Where(" comp_code = ? AND team_number = ? AND match_number = ? AND set_number = ? AND comp_level = ? AND comp_type = ?",
			compCode, teamNumber, matchNumber, setNumber, compLevel, compType).
		Find(&stats2025)
	return stats2025, result.Error
}

func (database *Database) ReturnRankings() ([]Ranking, error) {
	var rankins []Ranking
	result := database.Find(&rankins)
	return rankins, result.Error
}

func (database *Database) queryMatches(teamNumber_ string) ([]TeamMatch, error) {
	var matches []TeamMatch
	result := database.
		Where("team_number = $1", teamNumber_).
		Find(&matches)
	return matches, result.Error
}

func (database *Database) queryMatches2025(teamNumber_ string, compCode string) ([]TeamMatch2025, error) {
	var matches []TeamMatch2025
	result := database.
		Where("team_number = $1 AND comp_code = $2", teamNumber_, compCode).Order("comp_code").Order("match_number").
		Find(&matches)
	return matches, result.Error
}

func (database *Database) QueryPitImages(teamNumber_ string) ([]RequestedPitImage, error) {
	var requestedPitImages []RequestedPitImage
	result := database.Model(&PitImage{}).
		Where("team_number = $1", teamNumber_).
		Find(&requestedPitImages)

	return requestedPitImages, result.Error
}

func (database *Database) QueryPitImageByChecksum(checksum_ string) (PitImage, error) {
	var pitImage PitImage
	result := database.
		Where("check_sum = $1", checksum_).
		Find(&pitImage)
	return pitImage, result.Error
}

func ComputeSha256FromByteArray(arr []byte) string {
	sum := sha256.Sum256(arr)
	return fmt.Sprintf("%x", sum)
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

func (database *Database) QueryActions(teamNumber_ string) ([]Action, error) {
	var actions []Action
	result := database.
		Where("team_number = ?", teamNumber_).Find(&actions)
	return actions, result.Error
}

func (database *Database) QueryNotes(TeamNumber string) ([]string, error) {
	var rawNotes []NotesData
	result := database.Where("team_number = ?", TeamNumber).Order("match_number").Find(&rawNotes)
	if result.Error != nil {
		return nil, result.Error
	}

	notes := make([]string, len(rawNotes))
	for i := range rawNotes {
		notes[i] = rawNotes[i].Notes
	}
	return notes, nil
}

func (database *Database) QueryNotes2025(CompCode string, TeamNumber string) ([]string, error) {
	var rawNotes []NotesData2025
	result := database.Where("comp_code = ? AND team_number = ?", CompCode, TeamNumber).Order("match_number").Find(&rawNotes)
	if result.Error != nil {
		return nil, result.Error
	}
	notes := make([]string, len(rawNotes))
	for i := range rawNotes {
		notes[i] = rawNotes[i].Notes
	}
	return notes, nil
}

func (database *Database) QueryRankings(TeamNumber string) ([]Ranking, error) {
	var rankins []Ranking
	result := database.Where("team_number = ?", TeamNumber).Find(&rankins)
	return rankins, result.Error
}

func (database *Database) AddNotes(data NotesData) error {
	result := database.Create(&NotesData{
		TeamNumber:     data.TeamNumber,
		MatchNumber:    data.MatchNumber,
		SetNumber:      data.SetNumber,
		CompLevel:      data.CompLevel,
		Notes:          data.Notes,
		GoodDriving:    data.GoodDriving,
		BadDriving:     data.BadDriving,
		SolidPlacing:   data.SolidPlacing,
		SketchyPlacing: data.SketchyPlacing,
		GoodDefense:    data.GoodDefense,
		BadDefense:     data.BadDefense,
		EasilyDefended: data.EasilyDefended,
		NoShow:         data.NoShow,
	})
	return result.Error
}

func (database *Database) AddNotes2025(data NotesData2025) error {
	result := database.Create(&NotesData2025{
		CompCode:             data.CompCode,
		TeamNumber:           data.TeamNumber,
		MatchNumber:          data.MatchNumber,
		SetNumber:            data.SetNumber,
		CompLevel:            data.CompLevel,
		Notes:                data.Notes,
		GoodDriving:          data.GoodDriving,
		BadDriving:           data.BadDriving,
		CoralGroundIntake:    data.CoralGroundIntake,
		CoralHpIntake:        data.CoralHpIntake,
		AlgaeGroundIntake:    data.AlgaeGroundIntake,
		SolidAlgaeShooting:   data.SolidAlgaeShooting,
		SketchyAlgaeShooting: data.SketchyAlgaeShooting,
		SolidCoralShooting:   data.SolidCoralShooting,
		ShuffleCoral:         data.ShuffleCoral,
		SketchyCoralShooting: data.SketchyCoralShooting,
		Penalties:            data.Penalties,
		GoodDefense:          data.GoodDefense,
		BadDefense:           data.BadDefense,
		EasilyDefended:       data.EasilyDefended,
		NoShow:               data.NoShow,
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

func (database *Database) AddDriverRanking2025(data DriverRanking2025) error {
	result := database.Create(&DriverRanking2025{
		CompCode:    data.CompCode,
		MatchNumber: data.MatchNumber,
		TeamNumber:  data.TeamNumber,
		Score:       data.Score,
	})
	return result.Error
}

func (database *Database) AddHumanRanking2025(data HumanRanking2025) error {
	result := database.Create(&HumanRanking2025{
		CompCode:    data.CompCode,
		MatchNumber: data.MatchNumber,
		TeamNumber:  data.TeamNumber,
		Score:       data.Score,
	})
	return result.Error
}

func (database *Database) AddParsedDriverRanking(data ParsedDriverRankingData) error {
	result := database.Clauses(clause.OnConflict{
		UpdateAll: true,
	}).Create(&data)
	return result.Error
}

func (database *Database) QueryDriverRanking(MatchNumber int) ([]DriverRankingData, error) {
	var data []DriverRankingData
	result := database.Where("match_number = ?", MatchNumber).Find(&data)
	return data, result.Error
}

func (database *Database) QueryDriverRanking2025(CompCode string) ([]DriverRanking2025, error) {
	var data []DriverRanking2025
	result := database.Where("comp_code = ?", CompCode).Order("team_number").Find(&data)
	return data, result.Error
}

func (database *Database) QueryHumanRanking2025(CompCode string) ([]HumanRanking2025, error) {
	var data []HumanRanking2025
	result := database.Where("comp_code = ?", CompCode).Order("team_number").Find(&data)
	return data, result.Error
}
