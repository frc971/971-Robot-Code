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

	correct := []TeamMatch2025{
		TeamMatch2025{
			MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "9999",
		},
		TeamMatch2025{
			MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "1000",
		},
		TeamMatch2025{
			MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "777",
		},
		TeamMatch2025{
			MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "0000",
		},
		TeamMatch2025{
			MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "4321",
		},
		TeamMatch2025{
			MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 3, TeamNumber: "1234",
		},
	}

	for _, match := range correct {
		err := fixture.db.AddToMatch2025(match)
		check(t, err, "Failed to add match data")
	}

	got, err := fixture.db.ReturnMatches2025("fakeCompCode")
	check(t, err, "Failed ReturnMatches2025()")

	checkDeepEqual(t, correct, got)
}

func TestAddOrUpdateRankingsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Ranking{
		Ranking{
			TeamNumber: "123",
			Losses:     1, Wins: 7, Ties: 0,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: "125",
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

func TestAddToStats2025DB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats2025{
		Stats2025{

			CompCode: "fakeCompCode", CompType: "Regular", TeamNumber: "432",
			MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 3,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 0,
			CoralDroppedAuto: 4, AlgaeDroppedAuto: 2, CoralMissedAuto: 0, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 1, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: false, DeepCage: true, BuddieClimb: true, NoShow: false, Defense: false, CollectedBy: "jimmy",
		},
		Stats2025{

			CompCode: "fakeCompCode", CompType: "Regular", TeamNumber: "52A",
			MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 0, L2Auto: 0, L3Auto: 0, L4Auto: 0, NetAuto: 0, ProcessorAuto: 0,
			CoralDroppedAuto: 0, AlgaeDroppedAuto: 0, CoralMissedAuto: 0, AlgaeMissedAuto: 0,
			MobilityAuto: false,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 2, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: false, DeepCage: true, BuddieClimb: true, NoShow: false, Defense: false, CollectedBy: "frank",
		},
		Stats2025{

			CompCode: "fakeCompCode", CompType: "Regular", TeamNumber: "745",
			MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 2,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 0,
			CoralDroppedAuto: 4, AlgaeDroppedAuto: 2, CoralMissedAuto: 0, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 0, AvgCycle: 0, RobotDied: false,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "george",
		},
		Stats2025{
			CompCode: "fakeCompCode", CompType: "Regular", TeamNumber: "894",
			MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 4,
			L1Auto: 3, L2Auto: 1, L3Auto: 2, L4Auto: 1, NetAuto: 0, ProcessorAuto: 1,
			CoralDroppedAuto: 1, AlgaeDroppedAuto: 0, CoralMissedAuto: 1, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     4, L2Teleop: 2, L3Teleop: 3, L4Teleop: 1,
			ProcessorTeleop: 2, NetTeleop: 1,
			CoralDroppedTeleop: 3, AlgaeDroppedTeleop: 2,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 1, AvgCycle: 0, RobotDied: false,
			Park: false, ShallowCage: true, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: true, CollectedBy: "maddie",
		},
		Stats2025{

			CompCode: "fakeCompCode", CompType: "Regular", TeamNumber: "934",
			MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 3,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 0,
			CoralDroppedAuto: 4, AlgaeDroppedAuto: 2, CoralMissedAuto: 0, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 4, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: false, DeepCage: true, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "danny",
		},
		Stats2025{

			CompCode: "fakeCompCode", CompType: "Practice", TeamNumber: "942",
			MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 3,
			L1Auto: 0, L2Auto: 0, L3Auto: 0, L4Auto: 0, NetAuto: 0, ProcessorAuto: 0,
			CoralDroppedAuto: 0, AlgaeDroppedAuto: 0, CoralMissedAuto: 0, AlgaeMissedAuto: 0,
			MobilityAuto: false,
			L1Teleop:     0, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
			ProcessorTeleop: 0, NetTeleop: 0,
			CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 0,
			CoralMissedTeleop: 0, AlgaeMissedTeleop: 0,
			Penalties: 0, AvgCycle: 0, RobotDied: false,
			Park: false, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "georgia",
		},
		Stats2025{

			CompCode: "fakeCompCode", CompType: "Regular", TeamNumber: "942",
			MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 2,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 0,
			CoralDroppedAuto: 1, AlgaeDroppedAuto: 2, CoralMissedAuto: 1, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 2, AvgCycle: 0, RobotDied: true,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "fiona",
		},
	}

	matches := []TeamMatch2025{
		TeamMatch2025{MatchNumber: 3, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "894"},
		TeamMatch2025{MatchNumber: 3, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "942"},
		TeamMatch2025{MatchNumber: 3, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "432"},
		TeamMatch2025{MatchNumber: 3, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "52A"},
		TeamMatch2025{MatchNumber: 3, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "745"},
		TeamMatch2025{MatchNumber: 3, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "B", AlliancePosition: 3, TeamNumber: "934"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch2025(match)
		check(t, err, "Failed to add match")
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToStats2025(correct[i])
		check(t, err, "Failed to add 2025stats to DB")
	}

	got, err := fixture.db.QueryStats2025("fakeCompCode")
	check(t, err, "Failed QueryStats2025()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestInsertPracticeMatchStats2025(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := Stats2025{

		CompCode: "fakeCompCode",
		CompType: "Regular", TeamNumber: "6344",
		MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 4,
		L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 0,
		CoralDroppedAuto: 4, AlgaeDroppedAuto: 2, CoralMissedAuto: 0, AlgaeMissedAuto: 2,
		MobilityAuto: true,
		L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
		ProcessorTeleop: 2, NetTeleop: 3,
		CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 3,
		CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
		Penalties: 2, AvgCycle: 0, RobotDied: true,
		Park: false, ShallowCage: false, DeepCage: true, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "danny",
	}

	// Attempt to insert the non-practice match data and make sure it fails.
	err := fixture.db.AddToStats2025(stats)
	if err == nil {
		t.Fatal("Expected error from inserting the stats.")
	}
	if err.Error() != "Failed to find team 6344 in match 3 in the schedule." {
		t.Fatal("Got:", err.Error())
	}

	// Mark the data as practice match data. It should now succeed.
	stats.CompType = "Practice"
	err = fixture.db.AddToStats2025(stats)
	check(t, err, "Failed to add prescouted stats to DB")
}

func TestQueryingStats2025ByTeam(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := []Stats2025{
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "328A",
			MatchNumber: 7, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 0,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 0, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 4, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 0, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: false, DeepCage: true, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "emma",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "978",
			MatchNumber: 2, SetNumber: 2, CompLevel: "qm", StartingQuadrant: 4,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 1,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 1,
			Penalties: 1, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: true, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "danny",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "328A",
			MatchNumber: 4, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 2,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 0,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 0, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     4, L2Teleop: 0, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 0,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 3,
			Penalties: 2, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: false, DeepCage: true, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "danny",
		},
	}

	matches := []TeamMatch2025{

		TeamMatch2025{MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "328A"},
		TeamMatch2025{MatchNumber: 2, CompCode: "fakeCompCode", SetNumber: 2, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "978"},
		TeamMatch2025{MatchNumber: 4, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "328A"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch2025(match)
		check(t, err, "Failed to add match")
	}

	for i := range stats {
		err := fixture.db.AddToStats2025(stats[i])
		check(t, err, "Failed to add 2025stats to DB")
	}

	// Validate that requesting status for a single team gets us the
	// expected data.
	statsFor328A, err := fixture.db.ReturnStats2025ForTeam("fakeCompCode", "328A", 7, 1, "qm", "Regular")

	check(t, err, "Failed ReturnStats2025()")

	if !reflect.DeepEqual([]Stats2025{stats[0]}, statsFor328A) {
		t.Errorf("Got %#v,\nbut expected %#v.", statsFor328A, stats[0])
	}
	// Validate that requesting team data for a non-existent match returns
	// nothing.
	statsForMissing, err := fixture.db.ReturnStats2025ForTeam("fakeCompCode", "6344", 9, 1, "qm", "Regular")

	check(t, err, "Failed ReturnStats2025()")

	if !reflect.DeepEqual([]Stats2025{}, statsForMissing) {
		t.Errorf("Got %#v,\nbut expected %#v.", statsForMissing, []Stats2025{})
	}
}

func TestDeleteFromStats2025(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	startingStats := []Stats2025{
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "345",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 1,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 4,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 2, AlgaeMissedTeleop: 1,
			Penalties: 0, AvgCycle: 0, RobotDied: true,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "bailey",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "645",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 4,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 1,
			CoralDroppedAuto: 2, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 1, L3Teleop: 5, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 3, AlgaeMissedTeleop: 1,
			Penalties: 1, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: true, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: true, CollectedBy: "kate",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "323",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 2,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 1,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     3, L2Teleop: 4, L3Teleop: 2, L4Teleop: 0,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 1,
			Penalties: 2, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: false, DeepCage: true, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "tyler",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "542",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 1,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     1, L2Teleop: 0, L3Teleop: 0, L4Teleop: 7,
			ProcessorTeleop: 2, NetTeleop: 3,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 4,
			CoralMissedTeleop: 1, AlgaeMissedTeleop: 0,
			Penalties: 3, AvgCycle: 0, RobotDied: true,
			Park: false, ShallowCage: true, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "max",
		},
	}

	correct := []Stats2025{
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "345",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 1,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 4,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 2, AlgaeMissedTeleop: 1,
			Penalties: 0, AvgCycle: 0, RobotDied: true,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "bailey",
		},
	}

	originalMatches := []TeamMatch2025{
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "345"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "645"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "323"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "542"},
	}

	// Matches for which we want to delete the stats.
	matches := []TeamMatch2025{
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			TeamNumber: "645"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			TeamNumber: "323"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			TeamNumber: "542"},
	}

	for _, match := range originalMatches {
		err := fixture.db.AddToMatch2025(match)
		check(t, err, "Failed to add match")
		fmt.Println("Match has been added : ", match.TeamNumber)
	}

	for _, stat := range startingStats {
		err := fixture.db.AddToStats2025(stat)
		check(t, err, "Failed to add stat")
	}

	for _, match := range matches {
		err := fixture.db.DeleteFromStats2025(match.CompCode, match.CompLevel, match.MatchNumber, match.SetNumber, match.TeamNumber)
		check(t, err, "Failed to delete stat2025")
	}

	got, err := fixture.db.QueryStats2025("fakeCompCode")
	check(t, err, "Failed QueryStats2025()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestDeleteFromActions2025(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	startingActions := []Action{
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1235", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0000, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1236", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0321, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1237", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0222, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1238", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0110, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1239", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0004, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1233", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0005, CollectedBy: "",
		},
	}

	correct := []Action{
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1235", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0000, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1236", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0321, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1237", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0222, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1238", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0110, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompcode", TeamNumber: "1233", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0005, CollectedBy: "",
		},
	}

	for _, action := range startingActions {
		err := fixture.db.AddAction(action)
		check(t, err, "Failed to add stat")
	}

	err := fixture.db.DeleteFromActions2025("fakeCompcode", "quals", 94, 1, "1239")

	got, err := fixture.db.ReturnActions()
	check(t, err, "Failed ReturnActions()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestDeleteFromNotesData2025(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	startingNotes := []NotesData2025{
		NotesData2025{CompCode: "2025temp", TeamNumber: "1234", MatchNumber: 5, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: false, BadDriving: true, CoralGroundIntake: true, CoralHpIntake: false, AlgaeGroundIntake: false, SolidAlgaeShooting: false, SketchyAlgaeShooting: true, SolidCoralShooting: true, ShuffleCoral: true, SketchyCoralShooting: false, ReefIntake: false, Penalties: true, GoodDefense: false, BadDefense: true, EasilyDefended: true, NoShow: false},
		NotesData2025{CompCode: "2025temp", TeamNumber: "291", MatchNumber: 6, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: true, CoralGroundIntake: true, CoralHpIntake: false, AlgaeGroundIntake: false, SolidAlgaeShooting: false, SketchyAlgaeShooting: true, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: false, ReefIntake: true, Penalties: true, GoodDefense: true, BadDefense: true, EasilyDefended: true, NoShow: true},
		NotesData2025{CompCode: "2025temp", TeamNumber: "1234", MatchNumber: 7, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: false, CoralGroundIntake: false, CoralHpIntake: true, AlgaeGroundIntake: false, SolidAlgaeShooting: true, SketchyAlgaeShooting: true, SolidCoralShooting: true, ShuffleCoral: true, SketchyCoralShooting: false, ReefIntake: false, Penalties: false, GoodDefense: true, BadDefense: false, EasilyDefended: false, NoShow: true},
		NotesData2025{CompCode: "2025temp", TeamNumber: "62A", MatchNumber: 10, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: true, CoralGroundIntake: true, CoralHpIntake: false, AlgaeGroundIntake: false, SolidAlgaeShooting: true, SketchyAlgaeShooting: false, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: true, ReefIntake: false, Penalties: false, GoodDefense: false, BadDefense: false, EasilyDefended: true, NoShow: false},
	}

	correct := []NotesData2025{
		NotesData2025{ID: 2, CompCode: "2025temp", TeamNumber: "291", MatchNumber: 6, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: true, CoralGroundIntake: true, CoralHpIntake: false, AlgaeGroundIntake: false, SolidAlgaeShooting: false, SketchyAlgaeShooting: true, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: false, ReefIntake: true, Penalties: true, GoodDefense: true, BadDefense: true, EasilyDefended: true, NoShow: true},
		NotesData2025{ID: 3, CompCode: "2025temp", TeamNumber: "1234", MatchNumber: 7, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: false, CoralGroundIntake: false, CoralHpIntake: true, AlgaeGroundIntake: false, SolidAlgaeShooting: true, SketchyAlgaeShooting: true, SolidCoralShooting: true, ShuffleCoral: true, SketchyCoralShooting: false, ReefIntake: false, Penalties: false, GoodDefense: true, BadDefense: false, EasilyDefended: false, NoShow: true},
		NotesData2025{ID: 4, CompCode: "2025temp", TeamNumber: "62A", MatchNumber: 10, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: true, CoralGroundIntake: true, CoralHpIntake: false, AlgaeGroundIntake: false, SolidAlgaeShooting: true, SketchyAlgaeShooting: false, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: true, ReefIntake: false, Penalties: false, GoodDefense: false, BadDefense: false, EasilyDefended: true, NoShow: false},
	}

	for _, notes := range startingNotes {
		err := fixture.db.AddNotes2025(notes)
		check(t, err, "Failed to add notes")
	}

	err := fixture.db.DeleteFromNotesData2025("2025temp", "quals", 5, 1, "1234")

	got, err := fixture.db.ReturnAllNotes2025("2025temp")
	check(t, err, "Failed ReturnAllNotes2025()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestQueryRankingsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []Ranking{
		Ranking{
			TeamNumber: "123",
			Losses:     1, Wins: 7, Ties: 2,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: "124",
			Losses:     3, Wins: 4, Ties: 0,
			Rank: 4, Dq: 2,
		},
		Ranking{
			TeamNumber: "125",
			Losses:     5, Wins: 2, Ties: 0,
			Rank: 17, Dq: 0,
		},
		Ranking{
			TeamNumber: "126",
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
			TeamNumber: "126",
			Losses:     0, Wins: 7, Ties: 0,
			Rank: 5, Dq: 0,
		},
	}

	got, err := fixture.db.QueryRankings("126")
	check(t, err, "Failed QueryRankings()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnMatchDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []TeamMatch2025{

		TeamMatch2025{
			MatchNumber: 7, CompCode: "fakeCompCode", SetNumber: 2, CompLevel: "qm", Alliance: "B", AlliancePosition: 1, TeamNumber: "3732"},
		TeamMatch2025{
			MatchNumber: 8, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm", Alliance: "B", AlliancePosition: 1, TeamNumber: "3732"},
		TeamMatch2025{
			MatchNumber: 8, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm", Alliance: "R", AlliancePosition: 2, TeamNumber: "4834"},
		TeamMatch2025{
			MatchNumber: 8, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm", Alliance: "R", AlliancePosition: 1, TeamNumber: "6835"},
		TeamMatch2025{
			MatchNumber: 9, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm", Alliance: "B", AlliancePosition: 3, TeamNumber: "9824"},
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToMatch2025(correct[i])
		check(t, err, fmt.Sprint("Failed to add match", i))
	}

	got, err := fixture.db.ReturnMatches2025("fakeCompCode")
	check(t, err, "Failed ReturnMatches2025()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestOverwriteNewMatchData(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []TeamMatch2025{
		TeamMatch2025{
			MatchNumber: 9, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 3, TeamNumber: "4464"},
		TeamMatch2025{
			MatchNumber: 8, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals", Alliance: "R", AlliancePosition: 2, TeamNumber: "2352"},
		TeamMatch2025{
			MatchNumber: 9, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 3, TeamNumber: "6321"},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddToMatch2025(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add match", i))
	}

	correct := []TeamMatch2025{
		TeamMatch2025{
			MatchNumber: 8, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals", Alliance: "R", AlliancePosition: 2, TeamNumber: "2352"},
		TeamMatch2025{
			MatchNumber: 9, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "quals", Alliance: "B", AlliancePosition: 3, TeamNumber: "6321"},
	}

	got, err := fixture.db.ReturnMatches2025("fakeCompCode")
	check(t, err, "Failed to get match list")

	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnRankingsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Ranking{
		Ranking{
			TeamNumber: "123",
			Losses:     1, Wins: 7, Ties: 2,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: "124",
			Losses:     3, Wins: 4, Ties: 0,
			Rank: 4, Dq: 2,
		},
		Ranking{
			TeamNumber: "125",
			Losses:     5, Wins: 2, Ties: 0,
			Rank: 17, Dq: 0,
		},
		Ranking{
			TeamNumber: "126",
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

func TestReturnStats2025DB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats2025{
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Practice4", TeamNumber: "432",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 0, L2Auto: 0, L3Auto: 0, L4Auto: 0, NetAuto: 3, ProcessorAuto: 1,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     1, L2Teleop: 1, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 4,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 2, AlgaeMissedTeleop: 1,
			Penalties: 0, AvgCycle: 0, RobotDied: true,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "henry",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "52A",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 2, L2Auto: 2, L3Auto: 1, L4Auto: 4, NetAuto: 1, ProcessorAuto: 1,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     3, L2Teleop: 1, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 4,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 2, AlgaeMissedTeleop: 1,
			Penalties: 1, AvgCycle: 0, RobotDied: true,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "jordan",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Practice", TeamNumber: "894",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 2, L2Auto: 0, L3Auto: 0, L4Auto: 0, NetAuto: 1, ProcessorAuto: 0,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     6, L2Teleop: 0, L3Teleop: 0, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 4,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 2, AlgaeMissedTeleop: 1,
			Penalties: 1, AvgCycle: 0, RobotDied: true,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: true, CollectedBy: "emma",
		},
		Stats2025{

			CompCode: "fakeCompCode",
			CompType: "Regular", TeamNumber: "942",
			MatchNumber: 5, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			L1Auto: 2, L2Auto: 4, L3Auto: 0, L4Auto: 0, NetAuto: 0, ProcessorAuto: 0,
			CoralDroppedAuto: 5, AlgaeDroppedAuto: 2, CoralMissedAuto: 3, AlgaeMissedAuto: 2,
			MobilityAuto: true,
			L1Teleop:     3, L2Teleop: 0, L3Teleop: 3, L4Teleop: 3,
			ProcessorTeleop: 2, NetTeleop: 4,
			CoralDroppedTeleop: 2, AlgaeDroppedTeleop: 3,
			CoralMissedTeleop: 2, AlgaeMissedTeleop: 1,
			Penalties: 4, AvgCycle: 0, RobotDied: true,
			Park: true, ShallowCage: false, DeepCage: false, BuddieClimb: false, NoShow: false, Defense: false, CollectedBy: "harry",
		},
	}

	matches := []TeamMatch2025{
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "894"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "942"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "432"},
		TeamMatch2025{MatchNumber: 5, CompCode: "fakeCompCode", SetNumber: 1, CompLevel: "qm",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "52A"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch2025(match)
		check(t, err, "Failed to add match")
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToStats2025(correct[i])
		check(t, err, fmt.Sprint("Failed to add stats ", i))
	}

	got, err := fixture.db.QueryStats2025("fakeCompCode")
	check(t, err, "Failed QueryStats2025()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestReturnActionsDB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()
	correct := []Action{
		Action{
			CompCode: "fakeCompCode", TeamNumber: "1235", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0000, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompCode", TeamNumber: "1236", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0321, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompCode", TeamNumber: "1237", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0222, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompCode", TeamNumber: "1238", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0110, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompCode", TeamNumber: "1239", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0004, CollectedBy: "",
		},
		Action{
			CompCode: "fakeCompCode", TeamNumber: "1233", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0005, CollectedBy: "",
		},
	}

	matches := []TeamMatch2025{
		TeamMatch2025{CompCode: "fakeCompCode", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "1235"},
		TeamMatch2025{CompCode: "fakeCompCode", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "1236"},
		TeamMatch2025{CompCode: "fakeCompCode", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "1237"},
		TeamMatch2025{CompCode: "fakeCompCode", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "1238"},
		TeamMatch2025{CompCode: "fakeCompCode", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "1239"},
		TeamMatch2025{CompCode: "fakeCompCode", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 3, TeamNumber: "1233"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch2025(match)
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
			TeamNumber: "123",
			Losses:     1, Wins: 7, Ties: 2,
			Rank: 2, Dq: 0,
		},
		Ranking{
			TeamNumber: "124",
			Losses:     3, Wins: 4, Ties: 0,
			Rank: 4, Dq: 2,
		},
		Ranking{
			TeamNumber: "125",
			Losses:     5, Wins: 2, Ties: 0,
			Rank: 17, Dq: 0,
		},
		Ranking{
			TeamNumber: "126",
			Losses:     0, Wins: 7, Ties: 0,
			Rank: 5, Dq: 0,
		},
		Ranking{
			TeamNumber: "125",
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
			TeamNumber: "125",
			Losses:     2, Wins: 4, Ties: 1,
			Rank: 5, Dq: 0,
		},
	}

	got, err := fixture.db.QueryRankings("125")
	check(t, err, "Failed QueryRankings()")

	checkDeepEqual(t, correct, got)
}

func TestNotes(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []string{"Note 1", "Note 3"}

	err := fixture.db.AddNotes(NotesData{TeamNumber: "1234", MatchNumber: 5, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: false, SolidPlacing: false, SketchyPlacing: true, GoodDefense: false, BadDefense: true, EasilyDefended: true, NoShow: false})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{TeamNumber: "1235", MatchNumber: 54, SetNumber: 1, CompLevel: "quals", Notes: "Note 2", GoodDriving: false, BadDriving: true, SolidPlacing: false, SketchyPlacing: true, GoodDefense: false, BadDefense: false, EasilyDefended: false, NoShow: false})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes(NotesData{TeamNumber: "1234", MatchNumber: 23, SetNumber: 3, CompLevel: "quals", Notes: "Note 3", GoodDriving: true, BadDriving: false, SolidPlacing: false, SketchyPlacing: true, GoodDefense: true, BadDefense: false, EasilyDefended: true, NoShow: true})
	check(t, err, "Failed to add Note")

	actual, err := fixture.db.QueryNotes("1234")
	check(t, err, "Failed to get Notes")

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}
}

func TestNotes2025(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []string{"Note 1", "Note 3"}

	err := fixture.db.AddNotes2025(NotesData2025{CompCode: "2025temp", TeamNumber: "1234", MatchNumber: 5, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: false, CoralGroundIntake: true, CoralHpIntake: false, AlgaeGroundIntake: false, SolidAlgaeShooting: false, SketchyAlgaeShooting: true, SolidCoralShooting: true, ShuffleCoral: true, SketchyCoralShooting: false, Penalties: false, GoodDefense: false, BadDefense: true, EasilyDefended: true, NoShow: false})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes2025(NotesData2025{CompCode: "2025temp", TeamNumber: "1235", MatchNumber: 54, SetNumber: 1, CompLevel: "quals", Notes: "Note 2", GoodDriving: false, BadDriving: true, CoralGroundIntake: true, CoralHpIntake: true, AlgaeGroundIntake: false, SolidAlgaeShooting: true, SketchyAlgaeShooting: false, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: false, Penalties: false, GoodDefense: false, BadDefense: false, EasilyDefended: false, NoShow: false})
	check(t, err, "Failed to add Note")
	err = fixture.db.AddNotes2025(NotesData2025{CompCode: "2025temp", TeamNumber: "1234", MatchNumber: 23, SetNumber: 3, CompLevel: "quals", Notes: "Note 3", GoodDriving: true, BadDriving: false, CoralGroundIntake: false, CoralHpIntake: true, AlgaeGroundIntake: true, SolidAlgaeShooting: true, SketchyAlgaeShooting: false, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: false, Penalties: true, GoodDefense: true, BadDefense: false, EasilyDefended: true, NoShow: true})
	check(t, err, "Failed to add Note")

	actual, err := fixture.db.QueryNotes2025("2025temp", "1234")
	check(t, err, "Failed to query Notes 2025")

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}

	correct := []NotesData2025{
		NotesData2025{ID: 1, CompCode: "2025temp", TeamNumber: "1234", MatchNumber: 5, SetNumber: 1, CompLevel: "quals", Notes: "Note 1", GoodDriving: true, BadDriving: false, CoralGroundIntake: true, CoralHpIntake: false, AlgaeGroundIntake: false, SolidAlgaeShooting: false, SketchyAlgaeShooting: true, SolidCoralShooting: true, ShuffleCoral: true, SketchyCoralShooting: false, Penalties: false, GoodDefense: false, BadDefense: true, EasilyDefended: true, NoShow: false},
		NotesData2025{ID: 3, CompCode: "2025temp", TeamNumber: "1234", MatchNumber: 23, SetNumber: 3, CompLevel: "quals", Notes: "Note 3", GoodDriving: true, BadDriving: false, CoralGroundIntake: false, CoralHpIntake: true, AlgaeGroundIntake: true, SolidAlgaeShooting: true, SketchyAlgaeShooting: false, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: false, Penalties: true, GoodDefense: true, BadDefense: false, EasilyDefended: true, NoShow: true},
		NotesData2025{ID: 2, CompCode: "2025temp", TeamNumber: "1235", MatchNumber: 54, SetNumber: 1, CompLevel: "quals", Notes: "Note 2", GoodDriving: false, BadDriving: true, CoralGroundIntake: true, CoralHpIntake: true, AlgaeGroundIntake: false, SolidAlgaeShooting: true, SketchyAlgaeShooting: false, SolidCoralShooting: true, ShuffleCoral: false, SketchyCoralShooting: false, Penalties: false, GoodDefense: false, BadDefense: false, EasilyDefended: false, NoShow: false},
	}

	got, err2 := fixture.db.ReturnAllNotes2025("2025temp")
	check(t, err2, "Failed to get Notes")
	checkDeepEqual(t, correct, got)
}

func TestDriverRanking(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []DriverRankingData{
		{ID: 1, MatchNumber: 12, Rank1: "1234", Rank2: "1235", Rank3: "1236"},
		{ID: 2, MatchNumber: 12, Rank1: "1236", Rank2: "1235", Rank3: "1234"},
	}

	err := fixture.db.AddDriverRanking(
		DriverRankingData{MatchNumber: 12, Rank1: "1234", Rank2: "1235", Rank3: "1236"},
	)
	check(t, err, "Failed to add Driver Ranking")
	err = fixture.db.AddDriverRanking(
		DriverRankingData{MatchNumber: 12, Rank1: "1236", Rank2: "1235", Rank3: "1234"},
	)
	check(t, err, "Failed to add Driver Ranking")
	err = fixture.db.AddDriverRanking(
		DriverRankingData{MatchNumber: 13, Rank1: "1235", Rank2: "1234", Rank3: "1236"},
	)
	check(t, err, "Failed to add Driver Ranking")

	actual, err := fixture.db.QueryDriverRanking(12)
	check(t, err, "Failed to get Driver Ranking")

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}
}

func TestDriverRanking2025(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []DriverRanking2025{
		{ID: 1, CompCode: "anc2024", MatchNumber: 12, TeamNumber: "1234", Score: 1},
		{ID: 3, CompCode: "anc2024", MatchNumber: 12, TeamNumber: "832", Score: 3},
	}

	err := fixture.db.AddDriverRanking2025(
		DriverRanking2025{CompCode: "anc2024", MatchNumber: 12, TeamNumber: "1234", Score: 1},
	)
	check(t, err, "Failed to add Driver Ranking")
	err = fixture.db.AddDriverRanking2025(
		DriverRanking2025{CompCode: "anc2025", MatchNumber: 12, TeamNumber: "94", Score: 4},
	)
	check(t, err, "Failed to add Driver Ranking")
	err = fixture.db.AddDriverRanking2025(
		DriverRanking2025{CompCode: "anc2024", MatchNumber: 12, TeamNumber: "832", Score: 3},
	)
	check(t, err, "Failed to add Driver Ranking")

	actual, err := fixture.db.QueryDriverRanking2025("anc2024")
	check(t, err, "Failed to get Driver Ranking")

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("Got %#v,\nbut expected %#v.", actual, expected)
	}
}

func TestHumanRanking2025(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	expected := []HumanRanking2025{
		{ID: 2, CompCode: "fol2024", MatchNumber: 9, TeamNumber: "1234", Score: 2},
		{ID: 1, CompCode: "fol2024", MatchNumber: 9, TeamNumber: "22", Score: 1},
	}

	err := fixture.db.AddHumanRanking2025(
		HumanRanking2025{CompCode: "fol2024", MatchNumber: 9, TeamNumber: "22", Score: 1},
	)
	check(t, err, "Failed to add Human Ranking")
	err = fixture.db.AddHumanRanking2025(
		HumanRanking2025{CompCode: "fol2024", MatchNumber: 9, TeamNumber: "1234", Score: 2},
	)
	check(t, err, "Failed to add Human Ranking")
	err = fixture.db.AddHumanRanking2025(
		HumanRanking2025{CompCode: "fol2025", MatchNumber: 9, TeamNumber: "93", Score: 3},
	)
	check(t, err, "Failed to add Human Ranking")

	actual, err := fixture.db.QueryHumanRanking2025("fol2024")
	check(t, err, "Failed to get Human Ranking")

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
