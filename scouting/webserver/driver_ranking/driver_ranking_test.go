package driver_ranking

import (
	"math"
	"testing"

	"github.com/davecgh/go-spew/spew"
	"github.com/frc971/971-Robot-Code/scouting/db"
)

type MockDatabase struct {
	rawRankings    []db.DriverRankingData
	parsedRankings []db.ParsedDriverRankingData
}

func (database *MockDatabase) ReturnAllDriverRankings() ([]db.DriverRankingData, error) {
	return database.rawRankings, nil
}

func (database *MockDatabase) AddParsedDriverRanking(data db.ParsedDriverRankingData) error {
	database.parsedRankings = append(database.parsedRankings, data)
	return nil
}

// Validates that we can call out to an external script to parse the raw driver
// rankings and turn them into meaningful driver rankings. We don't call the
// real DriverRank.jl script here because we don't have Julia support.
func TestDriverRankingRun(t *testing.T) {
	var database MockDatabase
	database.rawRankings = []db.DriverRankingData{
		db.DriverRankingData{MatchNumber: 1, Rank1: 1234, Rank2: 1235, Rank3: 1236},
		db.DriverRankingData{MatchNumber: 2, Rank1: 971, Rank2: 972, Rank3: 973},
	}

	GenerateFullDriverRanking(&database, "./fake_driver_rank_script")

	// This is the data that fake_driver_rank_script generates.
	expected := []db.ParsedDriverRankingData{
		db.ParsedDriverRankingData{TeamNumber: "1234", Score: 1.5},
		db.ParsedDriverRankingData{TeamNumber: "1235", Score: 2.75},
		db.ParsedDriverRankingData{TeamNumber: "1236", Score: 4.0},
		db.ParsedDriverRankingData{TeamNumber: "971", Score: 5.25},
		db.ParsedDriverRankingData{TeamNumber: "972", Score: 6.5},
		db.ParsedDriverRankingData{TeamNumber: "973", Score: 7.75},
	}
	if len(expected) != len(database.parsedRankings) {
		t.Fatalf(spew.Sprintf("Got %#v,\nbut expected %#v.", database.parsedRankings, expected))
	}

	// Compare each row manually because the floating point values might
	// not match perfectly.
	for i := range expected {
		if expected[i].TeamNumber != database.parsedRankings[i].TeamNumber ||
			math.Abs(float64(expected[i].Score-database.parsedRankings[i].Score)) > 0.001 {
			t.Fatalf(spew.Sprintf("Got %#v,\nbut expected %#v.", database.parsedRankings, expected))
		}
	}
}
