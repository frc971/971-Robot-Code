package db

import (
	"fmt"
	"log"
	"os"
	"os/exec"
	"reflect"
	"strings"
	"testing"
	"time"

	"github.com/davecgh/go-spew/spew"
)

// Shortcut for error checking. If the specified error is non-nil, print the
// error message and exit the test.
func check(t *testing.T, err error, message string) {
	if err != nil {
		t.Fatal(message, ":", err)
	}
}

type dbFixture struct {
	db     *Database
	server *exec.Cmd
}

func (fixture dbFixture) TearDown() {
	fixture.db.Delete()
	log.Println("Shutting down testdb")
	fixture.server.Process.Signal(os.Interrupt)
	fixture.server.Process.Wait()
	log.Println("Successfully shut down testdb")
}

func createDatabase(t *testing.T) dbFixture {
	var fixture dbFixture

	log.Println("Starting up postgres.")
	fixture.server = exec.Command("testdb_server/testdb_server_/testdb_server")
	fixture.server.Stdout = os.Stdout
	fixture.server.Stderr = os.Stderr
	err := fixture.server.Start()
	check(t, err, "Failed to run postgres")

	// Wait until the server is ready. We cannot rely on the TCP socket
	// alone because postgres creates the socket before it's actually ready
	// to service requests.
	for {
		fixture.db, err = NewDatabase("test", "password", 5432)
		if err == nil {
			break
		}
		time.Sleep(50 * time.Millisecond)
	}
	log.Println("Connected to postgres.")

	fixture.db.SetDebugLogLevel()

	return fixture
}

func checkDeepEqual(t *testing.T, expected interface{}, actual interface{}) {
	if !reflect.DeepEqual(expected, actual) {
		t.Fatalf(spew.Sprintf("Got %#v,\nbut expected %#v.", actual, expected))
	}
}

func TestAddToMatchDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Match{
		Match{
			MatchNumber: 7,
			SetNumber:   1,
			CompLevel:   "quals",
			R1:          9999, R2: 1000, R3: 777, B1: 0000, B2: 4321, B3: 1234,
		},
	}

	err := fixture.db.AddToMatch(correct[0])
	check(t, err, "Failed to add match data")

	got, err := fixture.db.ReturnMatches()
	check(t, err, "Failed ReturnMatches()")

	checkDeepEqual(t, correct, got)
}

func TestAddOrUpdateRankingsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Ranking{
		Ranking{
			TeamNumber: 123,
			Losses:     1, Wins: 7, Ties: 0,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: 125,
			Losses:     2, Wins: 4, Ties: 0,
			Rank: 2, Dq: 0,
		},
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddOrUpdateRankings(correct[i])
		check(t, err, "Failed to add ranking data")
	}

	got, err := fixture.db.ReturnRankings()
	check(t, err, "Failed ReturnRankings()")

	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestAddToStatsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats{
		Stats{
			TeamNumber: 1236, MatchNumber: 7,
			StartingQuadrant: 2,
			AutoBallPickedUp: [5]bool{false, false, false, true, false},
			ShotsMissed:      9, UpperGoalShots: 5, LowerGoalShots: 4,
			ShotsMissedAuto: 3, UpperGoalAuto: 2, LowerGoalAuto: 1,
			PlayedDefense: 2, DefenseReceivedScore: 0, Climbing: 3,
			Comment: "this is a comment", CollectedBy: "josh",
		},
		Stats{
			TeamNumber: 1001, MatchNumber: 7,
			StartingQuadrant: 3,
			AutoBallPickedUp: [5]bool{true, false, true, true, false},
			ShotsMissed:      6, UpperGoalShots: 9, LowerGoalShots: 9,
			ShotsMissedAuto: 0, UpperGoalAuto: 0, LowerGoalAuto: 0,
			PlayedDefense: 0, DefenseReceivedScore: 1, Climbing: 0,
			Comment: "another comment", CollectedBy: "rupert",
		},
		Stats{
			TeamNumber: 777, MatchNumber: 7,
			StartingQuadrant: 4,
			AutoBallPickedUp: [5]bool{false, true, true, true, false},
			ShotsMissed:      5, UpperGoalShots: 7, LowerGoalShots: 12,
			ShotsMissedAuto: 0, UpperGoalAuto: 4, LowerGoalAuto: 0,
			PlayedDefense: 0, DefenseReceivedScore: 3, Climbing: 0,
			Comment: "and another", CollectedBy: "felix",
		},
		Stats{
			TeamNumber: 1000, MatchNumber: 7,
			StartingQuadrant: 1,
			AutoBallPickedUp: [5]bool{false, false, false, false, false},
			ShotsMissed:      12, UpperGoalShots: 6, LowerGoalShots: 10,
			ShotsMissedAuto: 0, UpperGoalAuto: 7, LowerGoalAuto: 0,
			PlayedDefense: 0, DefenseReceivedScore: 1, Climbing: 0,
			Comment: "and another one", CollectedBy: "thea",
		},
		Stats{
			TeamNumber: 4321, MatchNumber: 7,
			StartingQuadrant: 2,
			AutoBallPickedUp: [5]bool{true, false, false, false, false},
			ShotsMissed:      14, UpperGoalShots: 12, LowerGoalShots: 3,
			ShotsMissedAuto: 0, UpperGoalAuto: 7, LowerGoalAuto: 0,
			PlayedDefense: 0, DefenseReceivedScore: 0, Climbing: 0,
			Comment: "more comment", CollectedBy: "amy",
		},
		Stats{
			TeamNumber: 1234, MatchNumber: 7,
			StartingQuadrant: 3,
			AutoBallPickedUp: [5]bool{false, false, false, false, true},
			ShotsMissed:      3, UpperGoalShots: 4, LowerGoalShots: 0,
			ShotsMissedAuto: 0, UpperGoalAuto: 9, LowerGoalAuto: 0,
			PlayedDefense: 0, DefenseReceivedScore: 5, Climbing: 0,
			Comment: "final comment", CollectedBy: "beth",
		},
	}

	err := fixture.db.AddToMatch(Match{
		MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
		R1: 1236, R2: 1001, R3: 777, B1: 1000, B2: 4321, B3: 1234,
	})
	check(t, err, "Failed to add match")

	for i := 0; i < len(correct); i++ {
		err = fixture.db.AddToStats(correct[i])
		check(t, err, "Failed to add stats to DB")
	}

	got, err := fixture.db.ReturnStats()
	check(t, err, "Failed ReturnStats()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestAddDuplicateStats(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := Stats{
		TeamNumber: 1236, MatchNumber: 7,
		SetNumber: 1, CompLevel: "qual",
		StartingQuadrant: 2,
		AutoBallPickedUp: [5]bool{false, false, false, true, false},
		ShotsMissed:      9, UpperGoalShots: 5, LowerGoalShots: 4,
		ShotsMissedAuto: 3, UpperGoalAuto: 2, LowerGoalAuto: 1,
		PlayedDefense: 2, DefenseReceivedScore: 0, Climbing: 3,
		Comment: "this is a comment", CollectedBy: "josh",
	}

	err := fixture.db.AddToMatch(Match{
		MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
		R1: 1236, R2: 1001, R3: 777, B1: 1000, B2: 4321, B3: 1234,
	})
	check(t, err, "Failed to add match")

	// Add stats. This should succeed.
	err = fixture.db.AddToStats(stats)
	check(t, err, "Failed to add stats to DB")

	// Try again. It should fail this time.
	err = fixture.db.AddToStats(stats)
	if err == nil {
		t.Fatal("Failed to get error when adding duplicate stats.")
	}
	if !strings.Contains(err.Error(), "ERROR: duplicate key value violates unique constraint") {
		t.Fatal("Expected error message to be complain about duplicate key value, but got ", err)
	}
}

func TestQueryShiftDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Shift{
		Shift{
			MatchNumber: 1,
			R1scouter:   "Bob1", R2scouter: "Bob2", R3scouter: "Bob3", B1scouter: "Alice1", B2scouter: "Alice2", B3scouter: "Alice3",
		},
		Shift{
			MatchNumber: 2,
			R1scouter:   "Bob1", R2scouter: "Bob2", R3scouter: "Bob3", B1scouter: "Alice1", B2scouter: "Alice2", B3scouter: "Alice3",
		},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddToShift(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add shift", i))
	}

	correct := []Shift{
		Shift{
			MatchNumber: 1,
			R1scouter:   "Bob1", R2scouter: "Bob2", R3scouter: "Bob3", B1scouter: "Alice1", B2scouter: "Alice2", B3scouter: "Alice3",
		},
	}

	got, err := fixture.db.QueryAllShifts(1)
	check(t, err, "Failed to query shift for match 1")

	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestQueryStatsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Stats{
		Stats{
			TeamNumber: 1235, MatchNumber: 94, SetNumber: 2, CompLevel: "quals",
			StartingQuadrant: 1,
			AutoBallPickedUp: [5]bool{false, false, false, false, false},
			ShotsMissed:      2, UpperGoalShots: 2, LowerGoalShots: 2,
			ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2,
			PlayedDefense: 2, DefenseReceivedScore: 1, Climbing: 2},
		Stats{
			TeamNumber: 1234, MatchNumber: 94, SetNumber: 2, CompLevel: "quals",
			StartingQuadrant: 2,
			AutoBallPickedUp: [5]bool{false, false, false, false, true},
			ShotsMissed:      4, UpperGoalShots: 4, LowerGoalShots: 4,
			ShotsMissedAuto: 4, UpperGoalAuto: 4, LowerGoalAuto: 4,
			PlayedDefense: 7, DefenseReceivedScore: 1, Climbing: 2,
		},
		Stats{
			TeamNumber: 1233, MatchNumber: 94, SetNumber: 2, CompLevel: "quals",
			StartingQuadrant: 3,
			AutoBallPickedUp: [5]bool{false, false, false, false, false},
			ShotsMissed:      3, UpperGoalShots: 3, LowerGoalShots: 3,
			ShotsMissedAuto: 3, UpperGoalAuto: 3, LowerGoalAuto: 3,
			PlayedDefense: 3, DefenseReceivedScore: 0, Climbing: 3,
		},
		Stats{
			TeamNumber: 1232, MatchNumber: 94, SetNumber: 2, CompLevel: "quals",
			StartingQuadrant: 2,
			AutoBallPickedUp: [5]bool{true, false, false, false, true},
			ShotsMissed:      5, UpperGoalShots: 5, LowerGoalShots: 5,
			ShotsMissedAuto: 5, UpperGoalAuto: 5, LowerGoalAuto: 5,
			PlayedDefense: 7, DefenseReceivedScore: 2, Climbing: 1,
		},
		Stats{
			TeamNumber: 1231, MatchNumber: 94, SetNumber: 2, CompLevel: "quals",
			StartingQuadrant: 3,
			AutoBallPickedUp: [5]bool{false, false, true, false, false},
			ShotsMissed:      6, UpperGoalShots: 6, LowerGoalShots: 6,
			ShotsMissedAuto: 6, UpperGoalAuto: 6, LowerGoalAuto: 6,
			PlayedDefense: 7, DefenseReceivedScore: 3, Climbing: 1,
		},
		Stats{
			TeamNumber: 1239, MatchNumber: 94, SetNumber: 2, CompLevel: "quals",
			StartingQuadrant: 4,
			AutoBallPickedUp: [5]bool{false, true, true, false, false},
			ShotsMissed:      7, UpperGoalShots: 7, LowerGoalShots: 7,
			ShotsMissedAuto: 7, UpperGoalAuto: 7, LowerGoalAuto: 3,
			PlayedDefense: 7, DefenseReceivedScore: 5, Climbing: 1,
		},
	}

	err := fixture.db.AddToMatch(Match{
		MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
		R1: 1235, R2: 1234, R3: 1233, B1: 1232, B2: 1231, B3: 1239})
	check(t, err, "Failed to add match")

	for i := 0; i < len(testDatabase); i++ {
		err = fixture.db.AddToStats(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add stats ", i))
	}

	correct := []Stats{
		Stats{
			TeamNumber: 1235, MatchNumber: 94, SetNumber: 2, CompLevel: "quals",
			StartingQuadrant: 1,
			AutoBallPickedUp: [5]bool{false, false, false, false, false},
			ShotsMissed:      2, UpperGoalShots: 2, LowerGoalShots: 2,
			ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2,
			PlayedDefense: 2, DefenseReceivedScore: 1, Climbing: 2,
		},
	}

	got, err := fixture.db.QueryStats(1235)
	check(t, err, "Failed QueryStats()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestQueryRankingsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Ranking{
		Ranking{
			TeamNumber: 123,
			Losses:     1, Wins: 7, Ties: 2,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: 124,
			Losses:     3, Wins: 4, Ties: 0,
			Rank: 4, Dq: 2,
		},
		Ranking{
			TeamNumber: 125,
			Losses:     5, Wins: 2, Ties: 0,
			Rank: 17, Dq: 0,
		},
		Ranking{
			TeamNumber: 126,
			Losses:     0, Wins: 7, Ties: 0,
			Rank: 5, Dq: 0,
		},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddOrUpdateRankings(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add rankings ", i))
	}

	correct := []Ranking{
		Ranking{
			TeamNumber: 126,
			Losses:     0, Wins: 7, Ties: 0,
			Rank: 5, Dq: 0,
		},
	}

	got, err := fixture.db.QueryRankings(126)
	check(t, err, "Failed QueryRankings()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnMatchDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Match{
		Match{
			MatchNumber: 2, SetNumber: 1, CompLevel: "quals",
			R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149,
		},
		Match{
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262,
		},
		Match{
			MatchNumber: 4, SetNumber: 1, CompLevel: "quals",
			R1: 251, R2: 169, R3: 286, B1: 653, B2: 538, B3: 149,
		},
		Match{
			MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			R1: 198, R2: 1421, R3: 538, B1: 26, B2: 448, B3: 262,
		},
		Match{
			MatchNumber: 6, SetNumber: 1, CompLevel: "quals",
			R1: 251, R2: 188, R3: 286, B1: 555, B2: 538, B3: 149,
		},
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToMatch(correct[i])
		check(t, err, fmt.Sprint("Failed to add match", i))
	}

	got, err := fixture.db.ReturnMatches()
	check(t, err, "Failed ReturnMatches()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestOverwriteNewMatchData(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Match{
		Match{
			MatchNumber: 1, SetNumber: 1, CompLevel: "quals",
			R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149,
		},
		Match{
			MatchNumber: 2, SetNumber: 1, CompLevel: "quals",
			R1: 198, R2: 135, R3: 777, B1: 999, B2: 434, B3: 698,
		},
		Match{
			MatchNumber: 1, SetNumber: 1, CompLevel: "quals",
			R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262,
		},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddToMatch(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add match", i))
	}

	correct := []Match{
		Match{
			MatchNumber: 2, SetNumber: 1, CompLevel: "quals",
			R1: 198, R2: 135, R3: 777, B1: 999, B2: 434, B3: 698,
		},
		Match{
			MatchNumber: 1, SetNumber: 1, CompLevel: "quals",
			R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262,
		},
	}

	got, err := fixture.db.ReturnMatches()
	check(t, err, "Failed to get match list")

	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestAddReturnShiftDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Shift{
		Shift{
			MatchNumber: 1,
			R1scouter:   "Bob1", R2scouter: "Bob2", R3scouter: "Bob3", B1scouter: "Alice1", B2scouter: "Alice2", B3scouter: "Alice3",
		},
		Shift{
			MatchNumber: 2,
			R1scouter:   "Bob1", R2scouter: "Bob2", R3scouter: "Bob3", B1scouter: "Alice1", B2scouter: "Alice2", B3scouter: "Alice3",
		},
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToShift(correct[i])
		check(t, err, fmt.Sprint("Failed to add shift", i))
	}

	got, err := fixture.db.ReturnAllShifts()
	check(t, err, "Failed ReturnAllShifts()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnRankingsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Ranking{
		Ranking{
			TeamNumber: 123,
			Losses:     1, Wins: 7, Ties: 2,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: 124,
			Losses:     3, Wins: 4, Ties: 0,
			Rank: 4, Dq: 2,
		},
		Ranking{
			TeamNumber: 125,
			Losses:     5, Wins: 2, Ties: 0,
			Rank: 17, Dq: 0,
		},
		Ranking{
			TeamNumber: 126,
			Losses:     0, Wins: 7, Ties: 0,
			Rank: 5, Dq: 0,
		},
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddOrUpdateRankings(correct[i])
		check(t, err, fmt.Sprint("Failed to add rankings", i))
	}

	got, err := fixture.db.ReturnRankings()
	check(t, err, "Failed ReturnRankings()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnStatsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats{
		Stats{
			TeamNumber: 1235, MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			StartingQuadrant: 1,
			AutoBallPickedUp: [5]bool{false, false, false, false, false},
			ShotsMissed:      2, UpperGoalShots: 2, LowerGoalShots: 2,
			ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2,
			PlayedDefense: 2, DefenseReceivedScore: 3, Climbing: 2},
		Stats{
			TeamNumber: 1236, MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			StartingQuadrant: 2,
			AutoBallPickedUp: [5]bool{false, false, false, false, true},
			ShotsMissed:      4, UpperGoalShots: 4, LowerGoalShots: 4,
			ShotsMissedAuto: 4, UpperGoalAuto: 4, LowerGoalAuto: 4,
			PlayedDefense: 7, DefenseReceivedScore: 1, Climbing: 2,
		},
		Stats{
			TeamNumber: 1237, MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			StartingQuadrant: 3,
			AutoBallPickedUp: [5]bool{false, false, false, false, false},
			ShotsMissed:      3, UpperGoalShots: 3, LowerGoalShots: 3,
			ShotsMissedAuto: 3, UpperGoalAuto: 3, LowerGoalAuto: 3,
			PlayedDefense: 3, DefenseReceivedScore: 0, Climbing: 3,
		},
		Stats{
			TeamNumber: 1238, MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			StartingQuadrant: 2,
			AutoBallPickedUp: [5]bool{true, false, false, false, true},
			ShotsMissed:      5, UpperGoalShots: 5, LowerGoalShots: 5,
			ShotsMissedAuto: 5, UpperGoalAuto: 5, LowerGoalAuto: 5,
			PlayedDefense: 7, DefenseReceivedScore: 4, Climbing: 1,
		},
		Stats{
			TeamNumber: 1239, MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			StartingQuadrant: 3,
			AutoBallPickedUp: [5]bool{false, false, true, false, false},
			ShotsMissed:      6, UpperGoalShots: 6, LowerGoalShots: 6,
			ShotsMissedAuto: 6, UpperGoalAuto: 6, LowerGoalAuto: 6,
			PlayedDefense: 7, DefenseReceivedScore: 4, Climbing: 1,
		},
		Stats{
			TeamNumber: 1233, MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			StartingQuadrant: 4,
			AutoBallPickedUp: [5]bool{false, true, true, false, false},
			ShotsMissed:      7, UpperGoalShots: 7, LowerGoalShots: 7,
			ShotsMissedAuto: 7, UpperGoalAuto: 7, LowerGoalAuto: 3,
			PlayedDefense: 7, DefenseReceivedScore: 1, Climbing: 1,
		},
	}

	err := fixture.db.AddToMatch(Match{
		MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
		R1: 1235, R2: 1236, R3: 1237, B1: 1238, B2: 1239, B3: 1233})
	check(t, err, "Failed to add match")

	for i := 0; i < len(correct); i++ {
		err = fixture.db.AddToStats(correct[i])
		check(t, err, fmt.Sprint("Failed to add stats ", i))
	}

	got, err := fixture.db.ReturnStats()
	check(t, err, "Failed ReturnStats()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestRankingsDbUpdate(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Ranking{
		Ranking{
			TeamNumber: 123,
			Losses:     1, Wins: 7, Ties: 2,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: 124,
			Losses:     3, Wins: 4, Ties: 0,
			Rank: 4, Dq: 2,
		},
		Ranking{
			TeamNumber: 125,
			Losses:     5, Wins: 2, Ties: 0,
			Rank: 17, Dq: 0,
		},
		Ranking{
			TeamNumber: 126,
			Losses:     0, Wins: 7, Ties: 0,
			Rank: 5, Dq: 0,
		},
		Ranking{
			TeamNumber: 125,
			Losses:     2, Wins: 4, Ties: 1,
			Rank: 5, Dq: 0,
		},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddOrUpdateRankings(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add rankings ", i))
	}

	correct := []Ranking{
		Ranking{
			TeamNumber: 125,
			Losses:     2, Wins: 4, Ties: 1,
			Rank: 5, Dq: 0,
		},
	}

	got, err := fixture.db.QueryRankings(125)
	check(t, err, "Failed QueryRankings()")

	checkDeepEqual(t, correct, got)
}

func TestNotes(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []string{"Note 1", "Note 3"}

	err := fixture.db.AddNotes(NotesData{TeamNumber: 1234, Notes: "Note 1", GoodDriving: true, BadDriving: false, SketchyClimb: false, SolidClimb: true, GoodDefense: false, BadDefense: true})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{TeamNumber: 1235, Notes: "Note 2", GoodDriving: false, BadDriving: true, SketchyClimb: false, SolidClimb: true, GoodDefense: false, BadDefense: false})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{TeamNumber: 1234, Notes: "Note 3", GoodDriving: true, BadDriving: false, SketchyClimb: false, SolidClimb: true, GoodDefense: true, BadDefense: false})
	check(t, err, "Failed to add Note")

	actual, err := fixture.db.QueryNotes(1234)
	check(t, err, "Failed to get Notes")

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}
}

func TestDriverRanking(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []DriverRankingData{
		{ID: 1, MatchNumber: 12, Rank1: 1234, Rank2: 1235, Rank3: 1236},
		{ID: 2, MatchNumber: 12, Rank1: 1236, Rank2: 1235, Rank3: 1234},
	}

	err := fixture.db.AddDriverRanking(
		DriverRankingData{MatchNumber: 12, Rank1: 1234, Rank2: 1235, Rank3: 1236},
	)
	check(t, err, "Failed to add Driver Ranking")
	err = fixture.db.AddDriverRanking(
		DriverRankingData{MatchNumber: 12, Rank1: 1236, Rank2: 1235, Rank3: 1234},
	)
	check(t, err, "Failed to add Driver Ranking")
	err = fixture.db.AddDriverRanking(
		DriverRankingData{MatchNumber: 13, Rank1: 1235, Rank2: 1234, Rank3: 1236},
	)
	check(t, err, "Failed to add Driver Ranking")

	actual, err := fixture.db.QueryDriverRanking(12)
	check(t, err, "Failed to get Driver Ranking")

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}
}
