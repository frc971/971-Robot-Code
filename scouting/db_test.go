package db

import (
	"fmt"
	"reflect"
	"testing"
)

func TestAddToMatchDB(t *testing.T) {
	db, error_ := NewDatabase()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	correct := []Match{Match{matchNumber: 7, round: 1, compLevel: "quals", r1: 9999, r2: 1000, r3: 777, b1: 0000, b2: 4321, b3: 1234, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6}}
	db.AddToMatch(correct[0])
	got, error_ := db.ReturnMatches()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
	db.Delete()
}

func TestAddToStatsDB(t *testing.T) {
	db, error_ := NewDatabase()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	correct := []Stats{
		Stats{teamNumber: 1236, matchNumber: 7, shotsMissed: 9, upperGoalShots: 5, lowerGoalShots: 4, shotsMissedAuto: 3, upperGoalAuto: 2, lowerGoalAuto: 1, playedDefense: 2, climbing: 3},
		Stats{teamNumber: 1001, matchNumber: 7, shotsMissed: 6, upperGoalShots: 9, lowerGoalShots: 9, shotsMissedAuto: 0, upperGoalAuto: 0, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 777, matchNumber: 7, shotsMissed: 5, upperGoalShots: 7, lowerGoalShots: 12, shotsMissedAuto: 0, upperGoalAuto: 4, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 1000, matchNumber: 7, shotsMissed: 12, upperGoalShots: 6, lowerGoalShots: 10, shotsMissedAuto: 0, upperGoalAuto: 7, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 4321, matchNumber: 7, shotsMissed: 14, upperGoalShots: 12, lowerGoalShots: 3, shotsMissedAuto: 0, upperGoalAuto: 7, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
		Stats{teamNumber: 1234, matchNumber: 7, shotsMissed: 3, upperGoalShots: 4, lowerGoalShots: 0, shotsMissedAuto: 0, upperGoalAuto: 9, lowerGoalAuto: 0, playedDefense: 0, climbing: 0},
	}
	db.AddToMatch(Match{matchNumber: 7, round: 1, compLevel: "quals", r1: 1236, r2: 1001, r3: 777, b1: 1000, b2: 4321, b3: 1234, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6})
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
	db.Delete()
}

func TestQueryMatchDB(t *testing.T) {
	db, error_ := NewDatabase()
	if error_ != nil {
		t.Fatalf(error_.Error())
		fmt.Println("Error creating new database")
	}

	testDatabase := []Match{
		Match{matchNumber: 2, round: 1, compLevel: "quals", r1: 251, r2: 169, r3: 286, b1: 253, b2: 538, b3: 149},
		Match{matchNumber: 4, round: 1, compLevel: "quals", r1: 198, r2: 135, r3: 777, b1: 999, b2: 434, b3: 698},
		Match{matchNumber: 3, round: 1, compLevel: "quals", r1: 147, r2: 421, r3: 538, b1: 126, b2: 448, b3: 262},
		Match{matchNumber: 6, round: 1, compLevel: "quals", r1: 191, r2: 132, r3: 773, b1: 994, b2: 435, b3: 696},
	}

	for i := 0; i < len(testDatabase); i++ {
		db.AddToMatch(testDatabase[i])
	}

	correct := []Match{
		Match{matchNumber: 2, round: 1, compLevel: "quals", r1: 251, r2: 169, r3: 286, b1: 253, b2: 538, b3: 149, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6},
		Match{matchNumber: 3, round: 1, compLevel: "quals", r1: 147, r2: 421, r3: 538, b1: 126, b2: 448, b3: 262, r1ID: 13, r2ID: 14, r3ID: 15, b1ID: 16, b2ID: 17, b3ID: 18},
	}

	got, error_ := db.QueryMatches(538)
	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
	db.Delete()
}

func TestQueryStatsDB(t *testing.T) {
	db, error_ := NewDatabase()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	testDatabase := []Stats{
		Stats{teamNumber: 1235, matchNumber: 94, shotsMissed: 2, upperGoalShots: 2, lowerGoalShots: 2, shotsMissedAuto: 2, upperGoalAuto: 2, lowerGoalAuto: 2, playedDefense: 2, climbing: 2},
		Stats{teamNumber: 1234, matchNumber: 94, shotsMissed: 4, upperGoalShots: 4, lowerGoalShots: 4, shotsMissedAuto: 4, upperGoalAuto: 4, lowerGoalAuto: 4, playedDefense: 7, climbing: 2},
		Stats{teamNumber: 1233, matchNumber: 94, shotsMissed: 3, upperGoalShots: 3, lowerGoalShots: 3, shotsMissedAuto: 3, upperGoalAuto: 3, lowerGoalAuto: 3, playedDefense: 3, climbing: 3},
		Stats{teamNumber: 1232, matchNumber: 94, shotsMissed: 5, upperGoalShots: 5, lowerGoalShots: 5, shotsMissedAuto: 5, upperGoalAuto: 5, lowerGoalAuto: 5, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1231, matchNumber: 94, shotsMissed: 6, upperGoalShots: 6, lowerGoalShots: 6, shotsMissedAuto: 6, upperGoalAuto: 6, lowerGoalAuto: 6, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1239, matchNumber: 94, shotsMissed: 7, upperGoalShots: 7, lowerGoalShots: 7, shotsMissedAuto: 7, upperGoalAuto: 7, lowerGoalAuto: 3, playedDefense: 7, climbing: 1},
	}
	db.AddToMatch(Match{matchNumber: 94, round: 1, compLevel: "quals", r1: 1235, r2: 1234, r3: 1233, b1: 1232, b2: 1231, b3: 1239})
	for i := 0; i < len(testDatabase); i++ {
		db.AddToStats(testDatabase[i])
	}
	correct := []Stats{
		Stats{teamNumber: 1235, matchNumber: 94, shotsMissed: 2, upperGoalShots: 2, lowerGoalShots: 2, shotsMissedAuto: 2, upperGoalAuto: 2, lowerGoalAuto: 2, playedDefense: 2, climbing: 2},
	}
	got, error_ := db.QueryStats(1235)
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
	db.Delete()
}

func TestReturnMatchDB(t *testing.T) {
	db, error_ := NewDatabase()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	correct := []Match{
		Match{matchNumber: 2, round: 1, compLevel: "quals", r1: 251, r2: 169, r3: 286, b1: 253, b2: 538, b3: 149, r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6},
		Match{matchNumber: 3, round: 1, compLevel: "quals", r1: 147, r2: 421, r3: 538, b1: 126, b2: 448, b3: 262, r1ID: 7, r2ID: 8, r3ID: 9, b1ID: 10, b2ID: 11, b3ID: 12},
		Match{matchNumber: 4, round: 1, compLevel: "quals", r1: 251, r2: 169, r3: 286, b1: 653, b2: 538, b3: 149, r1ID: 13, r2ID: 14, r3ID: 15, b1ID: 16, b2ID: 17, b3ID: 18},
		Match{matchNumber: 5, round: 1, compLevel: "quals", r1: 198, r2: 1421, r3: 538, b1: 26, b2: 448, b3: 262, r1ID: 19, r2ID: 20, r3ID: 21, b1ID: 22, b2ID: 23, b3ID: 24},
		Match{matchNumber: 6, round: 1, compLevel: "quals", r1: 251, r2: 188, r3: 286, b1: 555, b2: 538, b3: 149, r1ID: 25, r2ID: 26, r3ID: 27, b1ID: 28, b2ID: 29, b3ID: 30},
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
	db.Delete()
}

func TestReturnStatsDB(t *testing.T) {
	db, error_ := NewDatabase()
	if error_ != nil {
		t.Fatalf(error_.Error())
	}
	correct := []Stats{
		Stats{teamNumber: 1235, matchNumber: 94, shotsMissed: 2, upperGoalShots: 2, lowerGoalShots: 2, shotsMissedAuto: 2, upperGoalAuto: 2, lowerGoalAuto: 2, playedDefense: 2, climbing: 2},
		Stats{teamNumber: 1236, matchNumber: 94, shotsMissed: 4, upperGoalShots: 4, lowerGoalShots: 4, shotsMissedAuto: 4, upperGoalAuto: 4, lowerGoalAuto: 4, playedDefense: 7, climbing: 2},
		Stats{teamNumber: 1237, matchNumber: 94, shotsMissed: 3, upperGoalShots: 3, lowerGoalShots: 3, shotsMissedAuto: 3, upperGoalAuto: 3, lowerGoalAuto: 3, playedDefense: 3, climbing: 3},
		Stats{teamNumber: 1238, matchNumber: 94, shotsMissed: 5, upperGoalShots: 5, lowerGoalShots: 5, shotsMissedAuto: 5, upperGoalAuto: 5, lowerGoalAuto: 5, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1239, matchNumber: 94, shotsMissed: 6, upperGoalShots: 6, lowerGoalShots: 6, shotsMissedAuto: 6, upperGoalAuto: 6, lowerGoalAuto: 6, playedDefense: 7, climbing: 1},
		Stats{teamNumber: 1233, matchNumber: 94, shotsMissed: 7, upperGoalShots: 7, lowerGoalShots: 7, shotsMissedAuto: 7, upperGoalAuto: 7, lowerGoalAuto: 3, playedDefense: 7, climbing: 1},
	}
	db.AddToMatch(Match{matchNumber: 94, round: 1, compLevel: "quals", r1: 1235, r2: 1236, r3: 1237, b1: 1238, b2: 1239, b3: 1233})
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
	db.Delete()
}
