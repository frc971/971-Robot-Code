package db

import (
	"fmt"
	"log"
	"os"
	"os/exec"
	"reflect"
	"testing"
	"time"
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
	fixture.db.Close()
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

	return fixture
}

func TestAddToMatchDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Match{
		Match{
			MatchNumber: 7,
			Round:       1,
			CompLevel:   "quals",
			R1:          9999, R2: 1000, R3: 777, B1: 0000, B2: 4321, B3: 1234,
			r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6,
		},
	}

	err := fixture.db.AddToMatch(correct[0])
	check(t, err, "Failed to add match data")

	got, err := fixture.db.ReturnMatches()
	check(t, err, "Failed ReturnMatches()")

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
			ShotsMissed: 9, UpperGoalShots: 5, LowerGoalShots: 4,
			ShotsMissedAuto: 3, UpperGoalAuto: 2, LowerGoalAuto: 1,
			PlayedDefense: 2, Climbing: 3,
			CollectedBy: "josh",
		},
		Stats{
			TeamNumber: 1001, MatchNumber: 7,
			ShotsMissed: 6, UpperGoalShots: 9, LowerGoalShots: 9,
			ShotsMissedAuto: 0, UpperGoalAuto: 0, LowerGoalAuto: 0,
			PlayedDefense: 0, Climbing: 0,
			CollectedBy: "rupert",
		},
		Stats{
			TeamNumber: 777, MatchNumber: 7,
			ShotsMissed: 5, UpperGoalShots: 7, LowerGoalShots: 12,
			ShotsMissedAuto: 0, UpperGoalAuto: 4, LowerGoalAuto: 0,
			PlayedDefense: 0, Climbing: 0,
			CollectedBy: "felix",
		},
		Stats{
			TeamNumber: 1000, MatchNumber: 7,
			ShotsMissed: 12, UpperGoalShots: 6, LowerGoalShots: 10,
			ShotsMissedAuto: 0, UpperGoalAuto: 7, LowerGoalAuto: 0,
			PlayedDefense: 0, Climbing: 0,
			CollectedBy: "thea",
		},
		Stats{
			TeamNumber: 4321, MatchNumber: 7,
			ShotsMissed: 14, UpperGoalShots: 12, LowerGoalShots: 3,
			ShotsMissedAuto: 0, UpperGoalAuto: 7, LowerGoalAuto: 0,
			PlayedDefense: 0, Climbing: 0,
			CollectedBy: "amy",
		},
		Stats{
			TeamNumber: 1234, MatchNumber: 7,
			ShotsMissed: 3, UpperGoalShots: 4, LowerGoalShots: 0,
			ShotsMissedAuto: 0, UpperGoalAuto: 9, LowerGoalAuto: 0,
			PlayedDefense: 0, Climbing: 0,
			CollectedBy: "beth",
		},
	}

	err := fixture.db.AddToMatch(Match{
		MatchNumber: 7, Round: 1, CompLevel: "quals",
		R1: 1236, R2: 1001, R3: 777, B1: 1000, B2: 4321, B3: 1234,
		r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6,
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

func TestQueryMatchDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Match{
		Match{MatchNumber: 2, Round: 1, CompLevel: "quals", R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149},
		Match{MatchNumber: 4, Round: 1, CompLevel: "quals", R1: 198, R2: 135, R3: 777, B1: 999, B2: 434, B3: 698},
		Match{MatchNumber: 3, Round: 1, CompLevel: "quals", R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262},
		Match{MatchNumber: 6, Round: 1, CompLevel: "quals", R1: 191, R2: 132, R3: 773, B1: 994, B2: 435, B3: 696},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddToMatch(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add match", i))
	}

	correct := []Match{
		Match{
			MatchNumber: 2, Round: 1, CompLevel: "quals",
			R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149,
			r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6,
		},
		Match{
			MatchNumber: 3, Round: 1, CompLevel: "quals",
			R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262,
			r1ID: 13, r2ID: 14, r3ID: 15, b1ID: 16, b2ID: 17, b3ID: 18,
		},
	}

	got, err := fixture.db.QueryMatches(538)
	check(t, err, "Failed to query matches for 538")

	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestQueryStatsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Stats{
		Stats{
			TeamNumber: 1235, MatchNumber: 94,
			ShotsMissed: 2, UpperGoalShots: 2, LowerGoalShots: 2,
			ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2,
			PlayedDefense: 2, Climbing: 2},
		Stats{
			TeamNumber: 1234, MatchNumber: 94,
			ShotsMissed: 4, UpperGoalShots: 4, LowerGoalShots: 4,
			ShotsMissedAuto: 4, UpperGoalAuto: 4, LowerGoalAuto: 4,
			PlayedDefense: 7, Climbing: 2,
		},
		Stats{
			TeamNumber: 1233, MatchNumber: 94,
			ShotsMissed: 3, UpperGoalShots: 3, LowerGoalShots: 3,
			ShotsMissedAuto: 3, UpperGoalAuto: 3, LowerGoalAuto: 3,
			PlayedDefense: 3, Climbing: 3,
		},
		Stats{
			TeamNumber: 1232, MatchNumber: 94,
			ShotsMissed: 5, UpperGoalShots: 5, LowerGoalShots: 5,
			ShotsMissedAuto: 5, UpperGoalAuto: 5, LowerGoalAuto: 5,
			PlayedDefense: 7, Climbing: 1,
		},
		Stats{
			TeamNumber: 1231, MatchNumber: 94,
			ShotsMissed: 6, UpperGoalShots: 6, LowerGoalShots: 6,
			ShotsMissedAuto: 6, UpperGoalAuto: 6, LowerGoalAuto: 6,
			PlayedDefense: 7, Climbing: 1,
		},
		Stats{
			TeamNumber: 1239, MatchNumber: 94,
			ShotsMissed: 7, UpperGoalShots: 7, LowerGoalShots: 7,
			ShotsMissedAuto: 7, UpperGoalAuto: 7, LowerGoalAuto: 3,
			PlayedDefense: 7, Climbing: 1,
		},
	}

	err := fixture.db.AddToMatch(Match{
		MatchNumber: 94, Round: 1, CompLevel: "quals",
		R1: 1235, R2: 1234, R3: 1233, B1: 1232, B2: 1231, B3: 1239})
	check(t, err, "Failed to add match")

	for i := 0; i < len(testDatabase); i++ {
		err = fixture.db.AddToStats(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add stats", i))
	}

	correct := []Stats{
		Stats{
			TeamNumber: 1235, MatchNumber: 94,
			ShotsMissed: 2, UpperGoalShots: 2, LowerGoalShots: 2,
			ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2,
			PlayedDefense: 2, Climbing: 2,
		},
	}

	got, err := fixture.db.QueryStats(1235)
	check(t, err, "Failed QueryStats()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnMatchDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Match{
		Match{
			MatchNumber: 2, Round: 1, CompLevel: "quals",
			R1: 251, R2: 169, R3: 286, B1: 253, B2: 538, B3: 149,
			r1ID: 1, r2ID: 2, r3ID: 3, b1ID: 4, b2ID: 5, b3ID: 6,
		},
		Match{
			MatchNumber: 3, Round: 1, CompLevel: "quals",
			R1: 147, R2: 421, R3: 538, B1: 126, B2: 448, B3: 262,
			r1ID: 7, r2ID: 8, r3ID: 9, b1ID: 10, b2ID: 11, b3ID: 12,
		},
		Match{
			MatchNumber: 4, Round: 1, CompLevel: "quals",
			R1: 251, R2: 169, R3: 286, B1: 653, B2: 538, B3: 149,
			r1ID: 13, r2ID: 14, r3ID: 15, b1ID: 16, b2ID: 17, b3ID: 18,
		},
		Match{
			MatchNumber: 5, Round: 1, CompLevel: "quals",
			R1: 198, R2: 1421, R3: 538, B1: 26, B2: 448, B3: 262,
			r1ID: 19, r2ID: 20, r3ID: 21, b1ID: 22, b2ID: 23, b3ID: 24,
		},
		Match{
			MatchNumber: 6, Round: 1, CompLevel: "quals",
			R1: 251, R2: 188, R3: 286, B1: 555, B2: 538, B3: 149,
			r1ID: 25, r2ID: 26, r3ID: 27, b1ID: 28, b2ID: 29, b3ID: 30,
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

func TestReturnStatsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats{
		Stats{
			TeamNumber: 1235, MatchNumber: 94,
			ShotsMissed: 2, UpperGoalShots: 2, LowerGoalShots: 2,
			ShotsMissedAuto: 2, UpperGoalAuto: 2, LowerGoalAuto: 2,
			PlayedDefense: 2, Climbing: 2,
		},
		Stats{
			TeamNumber: 1236, MatchNumber: 94,
			ShotsMissed: 4, UpperGoalShots: 4, LowerGoalShots: 4,
			ShotsMissedAuto: 4, UpperGoalAuto: 4, LowerGoalAuto: 4,
			PlayedDefense: 7, Climbing: 2,
		},
		Stats{
			TeamNumber: 1237, MatchNumber: 94,
			ShotsMissed: 3, UpperGoalShots: 3, LowerGoalShots: 3,
			ShotsMissedAuto: 3, UpperGoalAuto: 3, LowerGoalAuto: 3,
			PlayedDefense: 3, Climbing: 3,
		},
		Stats{
			TeamNumber: 1238, MatchNumber: 94,
			ShotsMissed: 5, UpperGoalShots: 5, LowerGoalShots: 5,
			ShotsMissedAuto: 5, UpperGoalAuto: 5, LowerGoalAuto: 5,
			PlayedDefense: 7, Climbing: 1,
		},
		Stats{
			TeamNumber: 1239, MatchNumber: 94,
			ShotsMissed: 6, UpperGoalShots: 6, LowerGoalShots: 6,
			ShotsMissedAuto: 6, UpperGoalAuto: 6, LowerGoalAuto: 6,
			PlayedDefense: 7, Climbing: 1,
		},
		Stats{
			TeamNumber: 1233, MatchNumber: 94,
			ShotsMissed: 7, UpperGoalShots: 7, LowerGoalShots: 7,
			ShotsMissedAuto: 7, UpperGoalAuto: 7, LowerGoalAuto: 3,
			PlayedDefense: 7, Climbing: 1,
		},
	}

	err := fixture.db.AddToMatch(Match{
		MatchNumber: 94, Round: 1, CompLevel: "quals",
		R1: 1235, R2: 1236, R3: 1237, B1: 1238, B2: 1239, B3: 1233})
	check(t, err, "Failed to add match")

	for i := 0; i < len(correct); i++ {
		err = fixture.db.AddToStats(correct[i])
		check(t, err, fmt.Sprint("Failed to add stats", i))
	}

	got, err := fixture.db.ReturnStats()
	check(t, err, "Failed ReturnStats()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestNotes(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := NotesData{
		TeamNumber: 1234,
		Notes:      []string{"Note 1", "Note 3"},
	}

	err := fixture.db.AddNotes(NotesData{1234, []string{"Note 1"}})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{1235, []string{"Note 2"}})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{1234, []string{"Note 3"}})
	check(t, err, "Failed to add Note")

	actual, err := fixture.db.QueryNotes(1234)
	check(t, err, "Failed to get Notes")

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}
}
