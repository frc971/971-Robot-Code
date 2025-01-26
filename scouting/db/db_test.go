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

func TestAddToStats2024DB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats2024{
		Stats2024{
			CompType: "Regular", TeamNumber: "894",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 4,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 0, Amp: 5, SpeakerAmplified: 1,
			NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: false, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "emma",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "942",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
			SpeakerAuto: 2, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 0, Amp: 5, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: false, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: true, CollectedBy: "harry",
		},
		Stats2024{
			CompType: "Practice", TeamNumber: "942",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 3,
			SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 2, Amp: 1, SpeakerAmplified: 3,
			NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "kaleb",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "432",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 3,
			SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 2, Amp: 1, SpeakerAmplified: 3, Shuttled: 0, OutOfField: 2,
			NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "henry",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "52A",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 1,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 0, Amp: 1, SpeakerAmplified: 2, Shuttled: 0, OutOfField: 0,
			NotesDropped: 2, Penalties: 0, TrapNote: true, Spotlight: false, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "jordan",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "745",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
			SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 5, Amp: 0, SpeakerAmplified: 2, Shuttled: 0, OutOfField: 0,
			NotesDropped: 1, Penalties: 1, TrapNote: true, Spotlight: true, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "taylor",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "934",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 3,
			SpeakerAuto: 1, AmpAuto: 3, NotesDroppedAuto: 0, MobilityAuto: true,
			Speaker: 0, Amp: 3, SpeakerAmplified: 2, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 3, TrapNote: true, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: false, Harmony: true, RobotDied: false, NoShow: true, CollectedBy: "katie",
		},
	}

	matches := []TeamMatch{
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "894"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "942"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "432"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "52A"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "745"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 3, TeamNumber: "934"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToStats2024(correct[i])
		check(t, err, "Failed to add 2024stats to DB")
	}

	got, err := fixture.db.ReturnStats2024()
	check(t, err, "Failed ReturnStats2024()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestInsertPreScoutedStats2024(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := Stats2024{
		CompType: "Regular", TeamNumber: "6344",
		MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 4,
		SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
		Speaker: 0, Amp: 5, SpeakerAmplified: 1, Shuttled: 1,
		NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: true, AvgCycle: 0,
		Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "emma",
	}

	// Attempt to insert the non-pre-scouted data and make sure it fails.
	err := fixture.db.AddToStats2024(stats)
	if err == nil {
		t.Fatal("Expected error from inserting the stats.")
	}
	if err.Error() != "Failed to find team 6344 in match 3 in the schedule." {
		t.Fatal("Got:", err.Error())
	}

	// Mark the data as pre-scouting data. It should now succeed.
	stats.CompType = "Prescouting"
	err = fixture.db.AddToStats2024(stats)
	check(t, err, "Failed to add prescouted stats to DB")
}

func TestInsertPracticeMatchStats2024(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := Stats2024{
		CompType: "Regular", TeamNumber: "6344",
		MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 4,
		SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
		Speaker: 0, Amp: 5, SpeakerAmplified: 1,
		NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: true, AvgCycle: 0,
		Park: true, OnStage: false, Harmony: false, RobotDied: false, CollectedBy: "emma",
	}

	// Attempt to insert the non-practice match data and make sure it fails.
	err := fixture.db.AddToStats2024(stats)
	if err == nil {
		t.Fatal("Expected error from inserting the stats.")
	}
	if err.Error() != "Failed to find team 6344 in match 3 in the schedule." {
		t.Fatal("Got:", err.Error())
	}

	// Mark the data as practice match data. It should now succeed.
	stats.CompType = "Practice"
	err = fixture.db.AddToStats2024(stats)
	check(t, err, "Failed to add prescouted stats to DB")
}

func TestQueryingStats2024ByTeam(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	stats := []Stats2024{
		Stats2024{
			CompType: "Regular", TeamNumber: "328A",
			MatchNumber: 7, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 0, Amp: 5, SpeakerAmplified: 1, Shuttled: 2, OutOfField: 0,
			NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: true, AvgCycle: 0,
			Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "emma",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "978",
			MatchNumber: 2, SetNumber: 2, CompLevel: "qm", StartingQuadrant: 4,
			SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 1, Amp: 2, SpeakerAmplified: 0, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: true, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: true, CollectedBy: "emma",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "328A",
			MatchNumber: 4, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 2,
			SpeakerAuto: 1, AmpAuto: 1, NotesDroppedAuto: 1, MobilityAuto: true,
			Speaker: 0, Amp: 1, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 1,
			NotesDropped: 1, Penalties: 0, TrapNote: false, Spotlight: true, AvgCycle: 0,
			Park: false, OnStage: false, Harmony: true, RobotDied: true, NoShow: false, CollectedBy: "emma",
		},
	}

	matches := []TeamMatch{
		TeamMatch{MatchNumber: 7, SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "328A"},
		TeamMatch{MatchNumber: 2, SetNumber: 2, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "978"},
		TeamMatch{MatchNumber: 4, SetNumber: 1, CompLevel: "qm",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "328A"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
	}

	for i := range stats {
		err := fixture.db.AddToStats2024(stats[i])
		check(t, err, "Failed to add 2024stats to DB")
	}

	// Validate that requesting status for a single team gets us the
	// expected data.
	statsFor328A, err := fixture.db.ReturnStats2024ForTeam("328A", 7, 1, "qm", "Regular")
	check(t, err, "Failed ReturnStats2024()")

	if !reflect.DeepEqual([]Stats2024{stats[0]}, statsFor328A) {
		t.Errorf("Got %#v,\nbut expected %#v.", statsFor328A, stats[0])
	}
	// Validate that requesting team data for a non-existent match returns
	// nothing.
	statsForMissing, err := fixture.db.ReturnStats2024ForTeam("6344", 9, 1, "qm", "Regular")
	check(t, err, "Failed ReturnStats2024()")

	if !reflect.DeepEqual([]Stats2024{}, statsForMissing) {
		t.Errorf("Got %#v,\nbut expected %#v.", statsForMissing, []Stats2024{})
	}
}

func TestDeleteFromStats2024(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	startingStats := []Stats2024{
		Stats2024{
			CompType: "Regular", TeamNumber: "345",
			MatchNumber: 5, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 1,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 1, Amp: 3, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 3,
			NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: true, CollectedBy: "bailey",
		},
		Stats2024{
			CompType: "Practice", TeamNumber: "645",
			MatchNumber: 5, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 4,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 1, Amp: 2, SpeakerAmplified: 0, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: true, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "kate",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "323",
			MatchNumber: 5, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
			SpeakerAuto: 1, AmpAuto: 1, NotesDroppedAuto: 1, MobilityAuto: true,
			Speaker: 0, Amp: 0, SpeakerAmplified: 2, Shuttled: 0, OutOfField: 2,
			NotesDropped: 1, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "tyler",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "542",
			MatchNumber: 5, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 1,
			SpeakerAuto: 1, AmpAuto: 1, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 1, Amp: 2, SpeakerAmplified: 2, Shuttled: 2, OutOfField: 2,
			NotesDropped: 1, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: false, Harmony: true, RobotDied: false, NoShow: false, CollectedBy: "max",
		},
	}

	correct := []Stats2024{
		Stats2024{
			CompType: "Regular", TeamNumber: "345",
			MatchNumber: 5, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 1,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 1, Amp: 3, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 3,
			NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: true, CollectedBy: "bailey",
		},
	}

	originalMatches := []TeamMatch{
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "345"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "645"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "323"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 2, TeamNumber: "542"},
	}

	// Matches for which we want to delete the stats.
	matches := []TeamMatch{
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			TeamNumber: "645"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			TeamNumber: "323"},
		TeamMatch{MatchNumber: 5, SetNumber: 1, CompLevel: "quals",
			TeamNumber: "542"},
	}

	for _, match := range originalMatches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
		fmt.Println("Match has been added : ", match.TeamNumber)
	}

	for _, stat := range startingStats {
		err := fixture.db.AddToStats2024(stat)
		check(t, err, "Failed to add stat")
	}

	for _, match := range matches {
		err := fixture.db.DeleteFromStats2024(match.CompLevel, match.MatchNumber, match.SetNumber, match.TeamNumber)
		check(t, err, "Failed to delete stat2024")
	}

	got, err := fixture.db.ReturnStats2024()
	check(t, err, "Failed ReturnStats2024()")

	if !reflect.DeepEqual(correct, got) {
		t.Errorf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestDeleteFromActions(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	startingActions := []Action{
		Action{
			TeamNumber: "1235", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0000, CollectedBy: "",
		},
		Action{
			TeamNumber: "1236", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0321, CollectedBy: "",
		},
		Action{
			TeamNumber: "1237", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0222, CollectedBy: "",
		},
		Action{
			TeamNumber: "1238", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0110, CollectedBy: "",
		},
		Action{
			TeamNumber: "1239", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0004, CollectedBy: "",
		},
		Action{
			TeamNumber: "1233", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0005, CollectedBy: "",
		},
	}

	correct := []Action{
		Action{
			TeamNumber: "1235", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0000, CollectedBy: "",
		},
		Action{
			TeamNumber: "1236", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0321, CollectedBy: "",
		},
		Action{
			TeamNumber: "1237", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0222, CollectedBy: "",
		},
		Action{
			TeamNumber: "1238", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0110, CollectedBy: "",
		},
		Action{
			TeamNumber: "1233", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0005, CollectedBy: "",
		},
	}

	for _, action := range startingActions {
		err := fixture.db.AddAction(action)
		check(t, err, "Failed to add stat")
	}

	err := fixture.db.DeleteFromActions("quals", 94, 1, "1239")

	got, err := fixture.db.ReturnActions()
	check(t, err, "Failed ReturnActions()")

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

func TestQueryPitImages(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []PitImage{
		PitImage{
			TeamNumber: "723", CheckSum: "8be8h9829hf98wp",
			ImagePath: "image1.jpg", ImageData: []byte{14, 15, 32, 54},
		},
		PitImage{
			TeamNumber: "237", CheckSum: "br78232b6r7iaa",
			ImagePath: "bot.png", ImageData: []byte{32, 54, 23, 00},
		},
		PitImage{
			TeamNumber: "125A", CheckSum: "b63c728bqiq8a73",
			ImagePath: "file123.jpeg", ImageData: []byte{32, 05, 01, 28},
		},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddPitImage(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add pit image", i))
	}

	correct := []RequestedPitImage{
		RequestedPitImage{
			TeamNumber: "723", CheckSum: "8be8h9829hf98wp",
			ImagePath: "image1.jpg",
		},
	}

	got, err := fixture.db.QueryPitImages("723")
	check(t, err, "Failed to query shift for team 723")

	if !reflect.DeepEqual(correct, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correct)
	}
}

func TestQueryPitImageByChecksum(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	testDatabase := []PitImage{
		PitImage{
			TeamNumber: "723", CheckSum: "8be8h9829hf98wp",
			ImagePath: "image1.jpg", ImageData: []byte{05, 32, 00, 74, 28},
		},
		PitImage{
			TeamNumber: "237", CheckSum: "br78232b6r7iaa",
			ImagePath: "bot.png", ImageData: []byte{32, 54, 23, 00},
		},
		PitImage{
			TeamNumber: "125A", CheckSum: "b63c728bqiq8a73",
			ImagePath: "file123.jpeg", ImageData: []byte{32, 05, 01, 28},
		},
	}

	for i := 0; i < len(testDatabase); i++ {
		err := fixture.db.AddPitImage(testDatabase[i])
		check(t, err, fmt.Sprint("Failed to add pit image", i))
	}

	correctPitImage := PitImage{
		TeamNumber: "125A", CheckSum: "b63c728bqiq8a73",
		ImagePath: "file123.jpeg", ImageData: []byte{32, 05, 01, 28},
	}

	got, err := fixture.db.QueryPitImageByChecksum("b63c728bqiq8a73")
	check(t, err, "Failed to query shift for checksum 'b63c728bqiq8a73'")

	if !reflect.DeepEqual(correctPitImage, got) {
		t.Fatalf("Got %#v,\nbut expected %#v.", got, correctPitImage)
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

func TestReturnStats2024DB(t *testing.T) {
	fixture := createDatabase(t)
	defer fixture.TearDown()

	correct := []Stats2024{
		Stats2024{
			CompType: "Practice", TeamNumber: "894",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 4,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 0, Amp: 5, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: false, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "emma",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "942",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
			SpeakerAuto: 2, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 0, Amp: 5, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 2, TrapNote: true, Spotlight: false, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "harry",
		},
		Stats2024{
			CompType: "Practice", TeamNumber: "432",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 3,
			SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 2, MobilityAuto: true,
			Speaker: 2, Amp: 1, SpeakerAmplified: 3, Shuttled: 5, OutOfField: 1,
			NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "henry",
		},
		Stats2024{
			CompType: "Regular", TeamNumber: "52A",
			MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 1,
			SpeakerAuto: 1, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 0, Amp: 1, SpeakerAmplified: 2, Shuttled: 0, OutOfField: 0,
			NotesDropped: 2, Penalties: 0, TrapNote: true, Spotlight: true, AvgCycle: 0,
			Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "jordan",
		},
	}

	matches := []TeamMatch{
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 1, TeamNumber: "894"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 2, TeamNumber: "942"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "R", AlliancePosition: 3, TeamNumber: "432"},
		TeamMatch{MatchNumber: 3, SetNumber: 1, CompLevel: "quals",
			Alliance: "B", AlliancePosition: 1, TeamNumber: "52A"},
	}

	for _, match := range matches {
		err := fixture.db.AddToMatch(match)
		check(t, err, "Failed to add match")
	}

	for i := 0; i < len(correct); i++ {
		err := fixture.db.AddToStats2024(correct[i])
		check(t, err, fmt.Sprint("Failed to add stats ", i))
	}

	got, err := fixture.db.ReturnStats2024()
	check(t, err, "Failed ReturnStats2024()")

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
			CompletedAction: []byte(""), Timestamp: 0000, CollectedBy: "",
		},
		Action{
			TeamNumber: "1236", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0321, CollectedBy: "",
		},
		Action{
			TeamNumber: "1237", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0222, CollectedBy: "",
		},
		Action{
			TeamNumber: "1238", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0110, CollectedBy: "",
		},
		Action{
			TeamNumber: "1239", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0004, CollectedBy: "",
		},
		Action{
			TeamNumber: "1233", MatchNumber: 94, SetNumber: 1, CompLevel: "quals",
			CompletedAction: []byte(""), Timestamp: 0005, CollectedBy: "",
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
