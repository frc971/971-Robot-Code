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
		Stats{TeamNumber: 1236, MatchNumber: 7, ShotsMissed: 9, UpperGoalShots: 5, LowerGoalShots: 4, ShotsMissedAuto: 3, UpperGoalAuto: 2, LowerGoalAuto: 1, PlayedDefense: 2, Climbing: 3},
		Stats{TeamNumber: 1001, MatchNumber: 7, ShotsMissed: 6, UpperGoalShots: 9, LowerGoalShots: 9, ShotsMissedAuto: 0, UpperGoalAuto: 0, LowerGoalAuto: 0, PlayedDefense: 0, Climbing: 0},
		Stats{TeamNumber: 777, MatchNumber: 7, ShotsMissed: 5, UpperGoalShots: 7, LowerGoalShots: 12, ShotsMissedAuto: 0, UpperGoalAuto: 4, LowerGoalAuto: 0, PlayedDefense: 0, Climbing: 0},
		Stats{TeamNumber: 1000, MatchNumber: 7, ShotsMissed: 12, UpperGoalShots: 6, LowerGoalShots: 10, ShotsMissedAuto: 0, UpperGoalAuto: 7, LowerGoalAuto: 0, PlayedDefense: 0, Climbing: 0},
		Stats{TeamNumber: 4321, MatchNumber: 7, ShotsMissed: 14, UpperGoalShots: 12, LowerGoalShots: 3, ShotsMissedAuto: 0, UpperGoalAuto: 7, LowerGoalAuto: 0, PlayedDefense: 0, Climbing: 0},
		Stats{TeamNumber: 1234, MatchNumber: 7, ShotsMissed: 3, UpperGoalShots: 4, LowerGoalShots: 0, ShotsMissedAuto: 0, UpperGoalAuto: 9, LowerGoalAuto: 0, PlayedDefense: 0, Climbing: 0},
	}
	err := db.AddToMatch(Match{MatchNumber: 7, Round: 1, CompLevel: "quals", R1: 1236, R2: 1001, R3: 777, B1: 1000, B2: 4321, B3: 1234, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6})
	if err != nil {
		t.Fatal("Failed to add match: ", err)
	}
	for i := 0; i < len(correct); i++ {
		if err := db.AddToStats(correct[i]); err != nil {
			t.Fatal("Failed to add stats to DB: ", err)
		}
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
		Stats{TeamNumber: 1235, MatchNumber: 94, ShotsMissed: 2, UpperGoalShots: 2, LowerGoalShots: 2, ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2, PlayedDefense: 2, Climbing: 2},
		Stats{TeamNumber: 1234, MatchNumber: 94, ShotsMissed: 4, UpperGoalShots: 4, LowerGoalShots: 4, ShotsMissedAuto: 4, UpperGoalAuto: 4, LowerGoalAuto: 4, PlayedDefense: 7, Climbing: 2},
		Stats{TeamNumber: 1233, MatchNumber: 94, ShotsMissed: 3, UpperGoalShots: 3, LowerGoalShots: 3, ShotsMissedAuto: 3, UpperGoalAuto: 3, LowerGoalAuto: 3, PlayedDefense: 3, Climbing: 3},
		Stats{TeamNumber: 1232, MatchNumber: 94, ShotsMissed: 5, UpperGoalShots: 5, LowerGoalShots: 5, ShotsMissedAuto: 5, UpperGoalAuto: 5, LowerGoalAuto: 5, PlayedDefense: 7, Climbing: 1},
		Stats{TeamNumber: 1231, MatchNumber: 94, ShotsMissed: 6, UpperGoalShots: 6, LowerGoalShots: 6, ShotsMissedAuto: 6, UpperGoalAuto: 6, LowerGoalAuto: 6, PlayedDefense: 7, Climbing: 1},
		Stats{TeamNumber: 1239, MatchNumber: 94, ShotsMissed: 7, UpperGoalShots: 7, LowerGoalShots: 7, ShotsMissedAuto: 7, UpperGoalAuto: 7, LowerGoalAuto: 3, PlayedDefense: 7, Climbing: 1},
	}
	db.AddToMatch(Match{MatchNumber: 94, Round: 1, CompLevel: "quals", R1: 1235, R2: 1234, R3: 1233, B1: 1232, B2: 1231, B3: 1239})
	for i := 0; i < len(testDatabase); i++ {
		db.AddToStats(testDatabase[i])
	}
	correct := []Stats{
		Stats{TeamNumber: 1235, MatchNumber: 94, ShotsMissed: 2, UpperGoalShots: 2, LowerGoalShots: 2, ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2, PlayedDefense: 2, Climbing: 2},
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
		Stats{TeamNumber: 1235, MatchNumber: 94, ShotsMissed: 2, UpperGoalShots: 2, LowerGoalShots: 2, ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2, PlayedDefense: 2, Climbing: 2},
		Stats{TeamNumber: 1236, MatchNumber: 94, ShotsMissed: 4, UpperGoalShots: 4, LowerGoalShots: 4, ShotsMissedAuto: 4, UpperGoalAuto: 4, LowerGoalAuto: 4, PlayedDefense: 7, Climbing: 2},
		Stats{TeamNumber: 1237, MatchNumber: 94, ShotsMissed: 3, UpperGoalShots: 3, LowerGoalShots: 3, ShotsMissedAuto: 3, UpperGoalAuto: 3, LowerGoalAuto: 3, PlayedDefense: 3, Climbing: 3},
		Stats{TeamNumber: 1238, MatchNumber: 94, ShotsMissed: 5, UpperGoalShots: 5, LowerGoalShots: 5, ShotsMissedAuto: 5, UpperGoalAuto: 5, LowerGoalAuto: 5, PlayedDefense: 7, Climbing: 1},
		Stats{TeamNumber: 1239, MatchNumber: 94, ShotsMissed: 6, UpperGoalShots: 6, LowerGoalShots: 6, ShotsMissedAuto: 6, UpperGoalAuto: 6, LowerGoalAuto: 6, PlayedDefense: 7, Climbing: 1},
		Stats{TeamNumber: 1233, MatchNumber: 94, ShotsMissed: 7, UpperGoalShots: 7, LowerGoalShots: 7, ShotsMissedAuto: 7, UpperGoalAuto: 7, LowerGoalAuto: 3, PlayedDefense: 7, Climbing: 1},
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
