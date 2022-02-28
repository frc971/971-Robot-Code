package db

import (
	"os"
	"path/filepath"
	"reflect"
	"testing"
)

// Creates a database in TEST_TMPDIR so that we don't accidentally write it
// into the runfiles directory.
func createDatabase(t *testing.T) *Database {
	// Get the path to our temporary writable directory.
	testTmpdir := os.Getenv("TEST_TMPDIR")
	db, err := NewDatabase(filepath.Join(testTmpdir, "scouting.db"))
	if err != nil {
		t.Fatal("Failed to create a new database: ", err)
	}
	return db
}

func TestAddToMatchDB(t *testing.T) {
	db := createDatabase(t)
	defer db.Delete()

	correct := []Match{Match{MatchNumber: 7, Round: 1, CompLevel: "quals", R1: 9999, R2: 1000, R3: 777, B1: 0000, B2: 4321, B3: 1234, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6}}
	db.AddToMatch(correct[0])
	got, error_ := db.ReturnMatches()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestAddToStatsDB(t *testing.T) {
	db := createDatabase(t)
	defer db.Delete()

	correct := []Stats{
		Stats{teamNumber: 1236, MatchNumber: 7, shotsMissed: 9, upperGoalShots: 5, lowerGoalShots: 4, shotsMissedAuto: 3, upperGoalAuto: 2, lowerGoalAuto: 1, playedDefense: 2, climbing: 3},
		Stats{teamNumber: 1001, MatchNumber: 7, shotsMissed: 6, upperGoalShots: 9, lowerGoalShots: 9, shotsMissedAuto: 0, upperGoalAuto: 0, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 777, MatchNumber: 7, shotsMissed: 5, upperGoalShots: 7, lowerGoalShots: 12, shotsMissedAuto: 0, upperGoalAuto: 4, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 1000, MatchNumber: 7, shotsMissed: 12, upperGoalShots: 6, lowerGoalShots: 10, shotsMissedAuto: 0, upperGoalAuto: 7, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 4321, MatchNumber: 7, shotsMissed: 14, upperGoalShots: 12, lowerGoalShots: 3, shotsMissedAuto: 0, upperGoalAuto: 7, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 1234, MatchNumber: 7, shotsMissed: 3, upperGoalShots: 4, lowerGoalShots: 0, shotsMissedAuto: 0, upperGoalAuto: 9, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
	}
	db.AddToMatch(Match{MatchNumber: 7, Round: 1, CompLevel: "quals", R1: 1236, R2: 1001, R3: 777, B1: 1000, B2: 4321, B3: 1234, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6})
	for i := 0; i < len(correct); i++ {
		db.AddToStats(correct[i])
	}
	got, error_ := db.ReturnStats()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestQueryMatchDB(t *testing.T) {
	db := createDatabase(t)
	defer db.Delete()

	testDatabase := []Match{
		Match{MatchNumber: 2, Round: 1, CompLevel: "quals", R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149},
		Match{MatchNumber: 4, Round: 1, CompLevel: "quals", R1: 198, R2: 135, R3: 777, B1: 999, B2: 434, B3: 698},
		Match{MatchNumber: 3, Round: 1, CompLevel: "quals", R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262},
		Match{MatchNumber: 6, Round: 1, CompLevel: "quals", R1: 191, R2: 132, R3: 773, B1: 994, B2: 435, B3: 696},
	}

	for i := 0; i < len(testDatabase); i++ {
		db.AddToMatch(testDatabase[i])
	}

	correct := []Match{
		Match{MatchNumber: 2, Round: 1, CompLevel: "quals", R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6},
		Match{MatchNumber: 3, Round: 1, CompLevel: "quals", R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262, r1ID: 13, r2ID: 14, r3ID: 15, b1ID: 16, b2ID: 17, b3ID: 18},
	}

	got, error_ := db.QueryMatches(538)
	if error_ != nil {
		t.Fatal("Failed to query matches for 538: ", error_)
	}
	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestQueryStatsDB(t *testing.T) {
	db := createDatabase(t)
	defer db.Delete()

	testDatabase := []Stats{
		Stats{teamNumber: 1235, MatchNumber: 94, shotsMissed: 2, upperGoalShots: 2, lowerGoalShots: 2, shotsMissedAuto: 2, upperGoalAuto: 2, lowerGoalAuto: 2, playedDefense: 2, climbing: 2},
		Stats{teamNumber: 1234, MatchNumber: 94, shotsMissed: 4, upperGoalShots: 4, lowerGoalShots: 4, shotsMissedAuto: 4, upperGoalAuto: 4, lowerGoalAuto: 4, playedDefense: 7, climbing: 2},
		Stats{teamNumber: 1233, MatchNumber: 94, shotsMissed: 3, upperGoalShots: 3, lowerGoalShots: 3, shotsMissedAuto: 3, upperGoalAuto: 3, lowerGoalAuto: 3, playedDefense: 3, climbing: 3},
		Stats{teamNumber: 1232, MatchNumber: 94, shotsMissed: 5, upperGoalShots: 5, lowerGoalShots: 5, shotsMissedAuto: 5, upperGoalAuto: 5, lowerGoalAuto: 5, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1231, MatchNumber: 94, shotsMissed: 6, upperGoalShots: 6, lowerGoalShots: 6, shotsMissedAuto: 6, upperGoalAuto: 6, lowerGoalAuto: 6, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1239, MatchNumber: 94, shotsMissed: 7, upperGoalShots: 7, lowerGoalShots: 7, shotsMissedAuto: 7, upperGoalAuto: 7, lowerGoalAuto: 3, playedDefense: 7, climbing: 1},
	}
	db.AddToMatch(Match{MatchNumber: 94, Round: 1, CompLevel: "quals", R1: 1235, R2: 1234, R3: 1233, B1: 1232, B2: 1231, B3: 1239})
	for i := 0; i < len(testDatabase); i++ {
		db.AddToStats(testDatabase[i])
	}
	correct := []Stats{
		Stats{teamNumber: 1235, MatchNumber: 94, shotsMissed: 2, upperGoalShots: 2, lowerGoalShots: 2, shotsMissedAuto: 2, upperGoalAuto: 2, lowerGoalAuto: 2, playedDefense: 2, climbing: 2},
	}
	got, error_ := db.QueryStats(1235)
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnMatchDB(t *testing.T) {
	db := createDatabase(t)
	defer db.Delete()

	correct := []Match{
		Match{MatchNumber: 2, Round: 1, CompLevel: "quals", R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6},
		Match{MatchNumber: 3, Round: 1, CompLevel: "quals", R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262, r1ID: 7, r2ID: 8, r3ID: 9, b1ID: 10, b2ID: 11, b3ID: 12},
		Match{MatchNumber: 4, Round: 1, CompLevel: "quals", R1: 251, R2: 169, R3: 286, B1: 653, B2: 538, B3: 149, r1ID: 13, r2ID: 14, r3ID: 15, b1ID: 16, b2ID: 17, b3ID: 18},
		Match{MatchNumber: 5, Round: 1, CompLevel: "quals", R1: 198, R2: 1421, R3: 538, B1: 26, B2: 448, B3: 262, r1ID: 19, r2ID: 20, r3ID: 21, b1ID: 22, b2ID: 23, b3ID: 24},
		Match{MatchNumber: 6, Round: 1, CompLevel: "quals", R1: 251, R2: 188, R3: 286, B1: 555, B2: 538, B3: 149, r1ID: 25, r2ID: 26, r3ID: 27, b1ID: 28, b2ID: 29, b3ID: 30},
	}
	for i := 0; i < len(correct); i++ {
		db.AddToMatch(correct[i])
	}
	got, error_ := db.ReturnMatches()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnStatsDB(t *testing.T) {
	db := createDatabase(t)
	defer db.Delete()

	correct := []Stats{
		Stats{teamNumber: 1235, MatchNumber: 94, shotsMissed: 2, upperGoalShots: 2, lowerGoalShots: 2, shotsMissedAuto: 2, upperGoalAuto: 2, lowerGoalAuto: 2, playedDefense: 2, climbing: 2},
		Stats{teamNumber: 1236, MatchNumber: 94, shotsMissed: 4, upperGoalShots: 4, lowerGoalShots: 4, shotsMissedAuto: 4, upperGoalAuto: 4, lowerGoalAuto: 4, playedDefense: 7, climbing: 2},
		Stats{teamNumber: 1237, MatchNumber: 94, shotsMissed: 3, upperGoalShots: 3, lowerGoalShots: 3, shotsMissedAuto: 3, upperGoalAuto: 3, lowerGoalAuto: 3, playedDefense: 3, climbing: 3},
		Stats{teamNumber: 1238, MatchNumber: 94, shotsMissed: 5, upperGoalShots: 5, lowerGoalShots: 5, shotsMissedAuto: 5, upperGoalAuto: 5, lowerGoalAuto: 5, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1239, MatchNumber: 94, shotsMissed: 6, upperGoalShots: 6, lowerGoalShots: 6, shotsMissedAuto: 6, upperGoalAuto: 6, lowerGoalAuto: 6, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1233, MatchNumber: 94, shotsMissed: 7, upperGoalShots: 7, lowerGoalShots: 7, shotsMissedAuto: 7, upperGoalAuto: 7, lowerGoalAuto: 3, playedDefense: 7, climbing: 1},
	}
	db.AddToMatch(Match{MatchNumber: 94, Round: 1, CompLevel: "quals", R1: 1235, R2: 1236, R3: 1237, B1: 1238, B2: 1239, B3: 1233})
	for i := 0; i < len(correct); i++ {
		db.AddToStats(correct[i])
	}
	got, error_ := db.ReturnStats()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}
