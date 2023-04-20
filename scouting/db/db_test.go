package db

import (
	"fmt"
	"log"
	"os"
	"os/exec"
	"reflect"
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

	correct := []TeamMatch{
		TeamMatch{
			MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "9999",
		},
		TeamMatch{
			MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "1000",
		},
		TeamMatch{
			MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "777",
		},
		TeamMatch{
			MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "0000",
		},
		TeamMatch{
			MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "4321",
		},
		TeamMatch{
			MatchNumber: 7, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 3, TeamNumber: "1234",
		},
	}

	for _, match := range correct {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match data")
	}

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

func TestAddToStats2023DB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats2023{
		Stats2023{
			PreScouting: false,
			TeamNumber:  "6344", MatchNumber: 3, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 1, HighCubesAuto: 0, CubesDroppedAuto: 1,
			LowConesAuto: 1, MiddleConesAuto: 0, HighConesAuto: 2,
			ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 2,
			HighCubes: 1, CubesDropped: 0, LowCones: 0,
			MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 0, Mobility: true, DockedAuto: true, EngagedAuto: false,
			BalanceAttemptAuto: false, Docked: false, Engaged: false,
			BalanceAttempt: false, CollectedBy: "emma",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "7454", MatchNumber: 3, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 2, LowCubesAuto: 1,
			MiddleCubesAuto: 2, HighCubesAuto: 2, CubesDroppedAuto: 0,
			LowConesAuto: 2, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 1, MiddleCubes: 0,
			HighCubes: 0, CubesDropped: 1, LowCones: 0,
			MiddleCones: 0, HighCones: 1, ConesDropped: 0, SuperchargedPieces: 1,
			AvgCycle: 0, Mobility: false, DockedAuto: false, EngagedAuto: false,
			BalanceAttemptAuto: true, Docked: true, Engaged: true,
			BalanceAttempt: false, CollectedBy: "tyler",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "4354", MatchNumber: 3, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 3, LowCubesAuto: 0,
			MiddleCubesAuto: 1, HighCubesAuto: 1, CubesDroppedAuto: 0,
			LowConesAuto: 0, MiddleConesAuto: 2, HighConesAuto: 1,
			ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 0,
			HighCubes: 2, CubesDropped: 1, LowCones: 1,
			MiddleCones: 1, HighCones: 0, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 0, Mobility: true, DockedAuto: false, EngagedAuto: false,
			BalanceAttemptAuto: false, Docked: false, Engaged: false,
			BalanceAttempt: true, CollectedBy: "isaac",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "6533", MatchNumber: 3, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 2, HighCubesAuto: 1, CubesDroppedAuto: 1,
			LowConesAuto: 1, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 0, LowCubes: 0, MiddleCubes: 1,
			HighCubes: 2, CubesDropped: 1, LowCones: 0,
			MiddleCones: 1, HighCones: 0, ConesDropped: 0, SuperchargedPieces: 0,
			AvgCycle: 0, Mobility: false, DockedAuto: true, EngagedAuto: true,
			BalanceAttemptAuto: true, Docked: false, Engaged: false,
			BalanceAttempt: true, CollectedBy: "will",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "8354", MatchNumber: 3, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 2, LowCubesAuto: 1,
			MiddleCubesAuto: 1, HighCubesAuto: 2, CubesDroppedAuto: 0,
			LowConesAuto: 0, MiddleConesAuto: 1, HighConesAuto: 1,
			ConesDroppedAuto: 1, LowCubes: 1, MiddleCubes: 0,
			HighCubes: 0, CubesDropped: 2, LowCones: 1,
			MiddleCones: 1, HighCones: 0, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 0, Mobility: true, DockedAuto: true, EngagedAuto: false,
			BalanceAttemptAuto: false, Docked: true, Engaged: false,
			BalanceAttempt: false, CollectedBy: "unkown",
		},
	}

	matches := []TeamMatch{
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "6344"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "7454"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "4354"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "6533"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "8354"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToStats2023(correct[i])
		check(t, err, "Failed to add 2023stats to DB")
	}

	got, err := fixture.db.ReturnStats2023()
	check(t, err, "Failed ReturnStats2023()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestInsertPreScoutedStats(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := Stats2023{
		PreScouting: false,
		TeamNumber:  "6344", MatchNumber: 3, SetNumber: 1,
		CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
		MiddleCubesAuto: 1, HighCubesAuto: 0, CubesDroppedAuto: 1,
		LowConesAuto: 1, MiddleConesAuto: 0, HighConesAuto: 2,
		ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 2,
		HighCubes: 1, CubesDropped: 0, LowCones: 0,
		MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
		AvgCycle: 0, Mobility: true, DockedAuto: true, EngagedAuto: false,
		BalanceAttemptAuto: false, Docked: false, Engaged: false,
		BalanceAttempt: false, CollectedBy: "emma",
	}

	// Attempt to insert the non-pre-scouted data and make sure it fails.
	err := fixture.db.AddToStats2023(stats)
	if err == nil {
		t.Fatal("Expected error from inserting the stats.")
	}
	if err.Error() != "Failed to find team 6344 in match 3 in the schedule." {
		t.Fatal("Got:", err.Error())
	}

	// Mark the data as pre-scouting data. It should now succeed.
	stats.PreScouting = true
	err = fixture.db.AddToStats2023(stats)
	check(t, err, "Failed to add prescouted stats to DB")
}

func TestQueryingStats2023ByTeam(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := []Stats2023{
		Stats2023{
			PreScouting: false,
			TeamNumber:  "6344", MatchNumber: 3, SetNumber: 1,
			CompLevel: "qm", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 1, HighCubesAuto: 0, CubesDroppedAuto: 1,
			LowConesAuto: 1, MiddleConesAuto: 0, HighConesAuto: 2,
			ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 2,
			HighCubes: 1, CubesDropped: 0, LowCones: 0,
			MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 0, Mobility: true, DockedAuto: true, EngagedAuto: false,
			BalanceAttemptAuto: false, Docked: false, Engaged: false,
			BalanceAttempt: false, CollectedBy: "emma",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "7454", MatchNumber: 4, SetNumber: 1,
			CompLevel: "qm", StartingQuadrant: 2, LowCubesAuto: 1,
			MiddleCubesAuto: 2, HighCubesAuto: 2, CubesDroppedAuto: 0,
			LowConesAuto: 2, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 1, MiddleCubes: 0,
			HighCubes: 0, CubesDropped: 1, LowCones: 0,
			MiddleCones: 0, HighCones: 1, ConesDropped: 0, SuperchargedPieces: 1,
			AvgCycle: 0, Mobility: false, DockedAuto: true, EngagedAuto: true,
			BalanceAttemptAuto: true, Docked: false, Engaged: false,
			BalanceAttempt: false, CollectedBy: "tyler",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "6344", MatchNumber: 5, SetNumber: 1,
			CompLevel: "qm", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 1, HighCubesAuto: 0, CubesDroppedAuto: 1,
			LowConesAuto: 1, MiddleConesAuto: 0, HighConesAuto: 2,
			ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 2,
			HighCubes: 1, CubesDropped: 0, LowCones: 0,
			MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 0, Mobility: true, DockedAuto: true, EngagedAuto: false,
			BalanceAttemptAuto: false, Docked: true, Engaged: false,
			BalanceAttempt: true, CollectedBy: "emma",
		},
	}

	matches := []TeamMatch{
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "6344"},
		TeamMatch{MatchNumber: 4, SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "7454"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "6344"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
	}

	for i := range stats {
		err := fixture.db.AddToStats2023(stats[i])
		check(t, err, "Failed to add 2023stats to DB")
	}

	// Validate that requesting status for a single team gets us the
	// expected data.
	statsFor6344, err := fixture.db.ReturnStats2023ForTeam("6344", 3, 1, "qm", false)
	check(t, err, "Failed ReturnStats2023()")

	if !reflect.DeepEqual([]Stats2023{stats[0]}, statsFor6344) {
		t.Errorf("Got %#v,\nbut expected %#v.", statsFor6344, stats[0])
	}

	// Validate that requesting team data for a non-existent match returns
	// nothing.
	statsForMissing, err := fixture.db.ReturnStats2023ForTeam("6344", 9, 1, "qm", false)
	check(t, err, "Failed ReturnStats2023()")

	if !reflect.DeepEqual([]Stats2023{}, statsForMissing) {
		t.Errorf("Got %#v,\nbut expected %#v.", statsForMissing, []Stats2023{})
	}
}

func TestDeleteFromStats(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	startingStats := []Stats2023{
		Stats2023{
			PreScouting: false,
			TeamNumber:  "1111", MatchNumber: 5, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 2, LowCubesAuto: 2,
			MiddleCubesAuto: 0, HighCubesAuto: 1, CubesDroppedAuto: 1,
			LowConesAuto: 0, MiddleConesAuto: 2, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 1,
			HighCubes: 2, CubesDropped: 1, LowCones: 1,
			MiddleCones: 0, HighCones: 1, ConesDropped: 2, SuperchargedPieces: 0,
			AvgCycle: 58, Mobility: true, DockedAuto: false, EngagedAuto: false,
			BalanceAttemptAuto: true, Docked: true, Engaged: true,
			BalanceAttempt: false, CollectedBy: "unknown",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "2314", MatchNumber: 5, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 3, LowCubesAuto: 1,
			MiddleCubesAuto: 0, HighCubesAuto: 1, CubesDroppedAuto: 1,
			LowConesAuto: 0, MiddleConesAuto: 1, HighConesAuto: 0,
			ConesDroppedAuto: 0, LowCubes: 2, MiddleCubes: 0,
			HighCubes: 1, CubesDropped: 0, LowCones: 0,
			MiddleCones: 2, HighCones: 1, ConesDropped: 0, SuperchargedPieces: 1,
			AvgCycle: 34, Mobility: true, DockedAuto: true, EngagedAuto: true,
			BalanceAttemptAuto: false, Docked: true, Engaged: false,
			BalanceAttempt: false, CollectedBy: "simon",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "3242", MatchNumber: 5, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 2, HighCubesAuto: 0, CubesDroppedAuto: 1,
			LowConesAuto: 1, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 2,
			HighCubes: 0, CubesDropped: 0, LowCones: 2,
			MiddleCones: 0, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 50, Mobility: true, DockedAuto: false, EngagedAuto: false,
			BalanceAttemptAuto: true, Docked: false, Engaged: false,
			BalanceAttempt: false, CollectedBy: "eliza",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "1742", MatchNumber: 5, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 4, LowCubesAuto: 1,
			MiddleCubesAuto: 1, HighCubesAuto: 0, CubesDroppedAuto: 0,
			LowConesAuto: 0, MiddleConesAuto: 2, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 1,
			HighCubes: 2, CubesDropped: 1, LowCones: 0,
			MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 49, Mobility: false, DockedAuto: true, EngagedAuto: false,
			Docked: false, Engaged: false, CollectedBy: "isaac",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "2454", MatchNumber: 5, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 0, HighCubesAuto: 0, CubesDroppedAuto: 0,
			LowConesAuto: 1, MiddleConesAuto: 1, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 1, MiddleCubes: 2,
			HighCubes: 0, CubesDropped: 0, LowCones: 1,
			MiddleCones: 1, HighCones: 1, ConesDropped: 0, SuperchargedPieces: 0,
			AvgCycle: 70, Mobility: false, DockedAuto: true, EngagedAuto: true,
			BalanceAttemptAuto: false, Docked: false, Engaged: false,
			BalanceAttempt: true, CollectedBy: "sam",
		},
	}

	correct := []Stats2023{
		Stats2023{
			PreScouting: false,
			TeamNumber:  "3242", MatchNumber: 5, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 2, HighCubesAuto: 0, CubesDroppedAuto: 1,
			LowConesAuto: 1, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 2,
			HighCubes: 0, CubesDropped: 0, LowCones: 2,
			MiddleCones: 0, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 50, Mobility: true, DockedAuto: false, EngagedAuto: false,
			BalanceAttemptAuto: true, Docked: false, Engaged: false,
			BalanceAttempt: false, CollectedBy: "eliza",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "2454", MatchNumber: 5, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
			MiddleCubesAuto: 0, HighCubesAuto: 0, CubesDroppedAuto: 0,
			LowConesAuto: 1, MiddleConesAuto: 1, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 1, MiddleCubes: 2,
			HighCubes: 0, CubesDropped: 0, LowCones: 1,
			MiddleCones: 1, HighCones: 1, ConesDropped: 0, SuperchargedPieces: 0,
			AvgCycle: 70, Mobility: false, DockedAuto: true, EngagedAuto: true,
			BalanceAttemptAuto: false, Docked: false, Engaged: false,
			BalanceAttempt: true, CollectedBy: "sam",
		},
	}

	originalMatches := []TeamMatch{
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "1111"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "2314"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "1742"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "2454"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 3, TeamNumber: "3242"},
	}

	// Matches for which we want to delete the stats.
	matches := []TeamMatch{
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			TeamNumber: "1111"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			TeamNumber: "2314"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			TeamNumber: "1742"},
	}

	for _, match := range originalMatches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
		fmt.Println("Match has been added : ", match.TeamNumber)
	}

	for _, stat := range startingStats {
		err := fixture.db.AddToStats2023(stat)
		check(t, err, "Failed to add stat")
	}

	for _, match := range matches {
		err := fixture.db.DeleteFromStats(match.CompLevel, match.MatchNumber, match.SetNumber, match.TeamNumber)
		check(t, err, "Failed to delete stat")
	}

	got, err := fixture.db.ReturnStats2023()
	check(t, err, "Failed ReturnStats()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
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

	correct := []TeamMatch{
		TeamMatch{
			MatchNumber: 8, SetNumber: 1, CompLevel: "quals", Alliance: "R", AlliancePosition: 1, TeamNumber: "6835"},
		TeamMatch{
			MatchNumber: 8, SetNumber: 1, CompLevel: "quals", Alliance: "R", AlliancePosition: 2, TeamNumber: "4834"},
		TeamMatch{
			MatchNumber: 9, SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 3, TeamNumber: "9824"},
		TeamMatch{
			MatchNumber: 7, SetNumber: 2, CompLevel: "quals", Alliance: "B", AlliancePosition: 1, TeamNumber: "3732"},
		TeamMatch{
			MatchNumber: 8, SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 1, TeamNumber: "3732"},
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

	testDatabase := []TeamMatch{
		TeamMatch{
			MatchNumber: 9, SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 3, TeamNumber: "4464"},
		TeamMatch{
			MatchNumber: 8, SetNumber: 1, CompLevel: "quals", Alliance: "R", AlliancePosition: 2, TeamNumber: "2352"},
		TeamMatch{
			MatchNumber: 9, SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 3, TeamNumber: "6321"},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddToMatch(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add match", i))
	}

	correct := []TeamMatch{
		TeamMatch{
			MatchNumber: 8, SetNumber: 1, CompLevel: "quals", Alliance: "R", AlliancePosition: 2, TeamNumber: "2352"},
		TeamMatch{
			MatchNumber: 9, SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 3, TeamNumber: "6321"},
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

func TestReturnStats2023DB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats2023{
		Stats2023{
			PreScouting: false,
			TeamNumber:  "2343", MatchNumber: 2, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 1,
			MiddleCubesAuto: 2, HighCubesAuto: 2, CubesDroppedAuto: 1,
			LowConesAuto: 0, MiddleConesAuto: 0, HighConesAuto: 2,
			ConesDroppedAuto: 1, LowCubes: 1, MiddleCubes: 2,
			HighCubes: 1, CubesDropped: 0, LowCones: 2,
			MiddleCones: 0, HighCones: 2, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 51, Mobility: true, DockedAuto: true, EngagedAuto: true,
			BalanceAttemptAuto: false, Docked: false, Engaged: false,
			BalanceAttempt: true, CollectedBy: "isaac",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "5443", MatchNumber: 2, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 2, LowCubesAuto: 1,
			MiddleCubesAuto: 1, HighCubesAuto: 0, CubesDroppedAuto: 1,
			LowConesAuto: 1, MiddleConesAuto: 1, HighConesAuto: 0,
			ConesDroppedAuto: 0, LowCubes: 2, MiddleCubes: 2,
			HighCubes: 1, CubesDropped: 0, LowCones: 1,
			MiddleCones: 0, HighCones: 2, ConesDropped: 1, SuperchargedPieces: 1,
			AvgCycle: 39, Mobility: false, DockedAuto: false, EngagedAuto: false,
			BalanceAttemptAuto: true, Docked: false, Engaged: false,
			BalanceAttempt: false, CollectedBy: "jack",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "5436", MatchNumber: 2, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 3, LowCubesAuto: 0,
			MiddleCubesAuto: 2, HighCubesAuto: 0, CubesDroppedAuto: 1,
			LowConesAuto: 2, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 2, MiddleCubes: 2,
			HighCubes: 0, CubesDropped: 0, LowCones: 1,
			MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 45, Mobility: false, DockedAuto: true, EngagedAuto: false,
			BalanceAttemptAuto: true, Docked: false, Engaged: false,
			BalanceAttempt: true, CollectedBy: "martin",
		},
		Stats2023{
			PreScouting: false,
			TeamNumber:  "5643", MatchNumber: 2, SetNumber: 1,
			CompLevel: "quals", StartingQuadrant: 4, LowCubesAuto: 0,
			MiddleCubesAuto: 0, HighCubesAuto: 1, CubesDroppedAuto: 1,
			LowConesAuto: 2, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 1, LowCubes: 2, MiddleCubes: 2,
			HighCubes: 0, CubesDropped: 0, LowCones: 2,
			MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
			AvgCycle: 34, Mobility: true, DockedAuto: true, EngagedAuto: false,
			BalanceAttemptAuto: false, Docked: true, Engaged: false,
			BalanceAttempt: false, CollectedBy: "unknown",
		},
	}

	matches := []TeamMatch{
		TeamMatch{MatchNumber: 2, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "2343"},
		TeamMatch{MatchNumber: 2, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "5443"},
		TeamMatch{MatchNumber: 2, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "5436"},
		TeamMatch{MatchNumber: 2, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "5643"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToStats2023(correct[i])
		check(t, err, fmt.Sprint("Failed to add stats ", i))
	}

	got, err := fixture.db.ReturnStats2023()
	check(t, err, "Failed ReturnStats()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnActionsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()
	correct := []Action{
		Action{
			TeamNumber: "1235", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), TimeStamp: 0000, CollectedBy: "",
		},
		Action{
			TeamNumber: "1236", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), TimeStamp: 0321, CollectedBy: "",
		},
		Action{
			TeamNumber: "1237", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), TimeStamp: 0222, CollectedBy: "",
		},
		Action{
			TeamNumber: "1238", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), TimeStamp: 0110, CollectedBy: "",
		},
		Action{
			TeamNumber: "1239", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), TimeStamp: 0004, CollectedBy: "",
		},
		Action{
			TeamNumber: "1233", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), TimeStamp: 0005, CollectedBy: "",
		},
	}

	matches := []TeamMatch{
		TeamMatch{MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "1235"},
		TeamMatch{MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "1236"},
		TeamMatch{MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "1237"},
		TeamMatch{MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "1238"},
		TeamMatch{MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "1239"},
		TeamMatch{MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 3, TeamNumber: "1233"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddAction(correct[i])
		check(t, err, fmt.Sprint("Failed to add to actions ", i))
	}

	got, err := fixture.db.ReturnActions()
	check(t, err, "Failed ReturnActions()")

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

	err := fixture.db.AddNotes(NotesData{TeamNumber: 1234, Notes: "Note 1", GoodDriving: true, BadDriving: false, SolidPlacing: false, SketchyPlacing: true, GoodDefense: false, BadDefense: true, EasilyDefended: true})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{TeamNumber: 1235, Notes: "Note 2", GoodDriving: false, BadDriving: true, SolidPlacing: false, SketchyPlacing: true, GoodDefense: false, BadDefense: false, EasilyDefended: false})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{TeamNumber: 1234, Notes: "Note 3", GoodDriving: true, BadDriving: false, SolidPlacing: false, SketchyPlacing: true, GoodDefense: true, BadDefense: false, EasilyDefended: true})
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

func TestParsedDriverRanking(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []ParsedDriverRankingData{
		{TeamNumber: "1234", Score: 100},
		{TeamNumber: "1235", Score: 110},
		{TeamNumber: "1236", Score: 90},
	}

	for i := range expected {
		err := fixture.db.AddParsedDriverRanking(expected[i])
		check(t, err, "Failed to add Parsed Driver Ranking")
	}

	actual, err := fixture.db.ReturnAllParsedDriverRankings()
	check(t, err, "Failed to get Parsed Driver Ranking")
	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}

	// Now update one of the rankings and make sure we get the properly
	// merged result.
	err = fixture.db.AddParsedDriverRanking(ParsedDriverRankingData{
		TeamNumber: "1235", Score: 200,
	})
	check(t, err, "Failed to add Parsed Driver Ranking")

	expected = []ParsedDriverRankingData{
		{TeamNumber: "1234", Score: 100},
		{TeamNumber: "1236", Score: 90},
		{TeamNumber: "1235", Score: 200},
	}

	actual, err = fixture.db.ReturnAllParsedDriverRankings()
	check(t, err, "Failed to get Parsed Driver Ranking")
	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}
}
