package requests

import (
	"net/http"
	"reflect"
	"testing"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/debug"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2023_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2023_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

// Validates that an unhandled address results in a 404.
func Test404(t *testing.T) {
	db := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	resp, err := http.Get("http://localhost:8080/requests/foo")
	if err != nil {
		t.Fatalf("Failed to get data: %v", err)
	}
	if resp.StatusCode != http.StatusNotFound {
		t.Fatalf("Expected error code 404, but got %d instead", resp.Status)
	}
}

// Validates that we can request the full match list.
func TestRequestAllMatches(t *testing.T) {
	db := MockDatabase{
		matches: []db.TeamMatch{
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 1, TeamNumber: "5",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 2, TeamNumber: "42",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 3, TeamNumber: "600",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 1, TeamNumber: "971",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 2, TeamNumber: "400",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 3, TeamNumber: "200",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 1, TeamNumber: "6",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 2, TeamNumber: "43",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 3, TeamNumber: "601",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 1, TeamNumber: "972",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 2, TeamNumber: "401",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 3, TeamNumber: "201",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 1, TeamNumber: "7",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 2, TeamNumber: "44",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 3, TeamNumber: "602",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 1, TeamNumber: "973",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 2, TeamNumber: "402",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 3, TeamNumber: "202",
			},
		},
		// Pretend that we have some data scouting data.
		stats2023: []db.Stats2023{
			{
				TeamNumber: "5", MatchNumber: 1, SetNumber: 1,
				CompLevel: "qm", StartingQuadrant: 3, LowCubesAuto: 10,
				MiddleCubesAuto: 1, HighCubesAuto: 1, CubesDroppedAuto: 0,
				LowConesAuto: 1, MiddleConesAuto: 2, HighConesAuto: 1,
				ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 1,
				HighCubes: 2, CubesDropped: 1, LowCones: 1,
				MiddleCones: 2, HighCones: 0, ConesDropped: 1, SuperchargedPieces: 0,
				AvgCycle: 34, Mobility: false, DockedAuto: true, EngagedAuto: true,
				BalanceAttemptAuto: false, Docked: false, Engaged: false,
				BalanceAttempt: false, CollectedBy: "alex",
			},
			{
				TeamNumber: "973", MatchNumber: 3, SetNumber: 1,
				CompLevel: "qm", StartingQuadrant: 1, LowCubesAuto: 0,
				MiddleCubesAuto: 1, HighCubesAuto: 1, CubesDroppedAuto: 2,
				LowConesAuto: 0, MiddleConesAuto: 0, HighConesAuto: 0,
				ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 0,
				HighCubes: 1, CubesDropped: 0, LowCones: 0,
				MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
				AvgCycle: 53, Mobility: true, DockedAuto: true, EngagedAuto: false,
				BalanceAttemptAuto: false, Docked: false, Engaged: false,
				BalanceAttempt: true, CollectedBy: "bob",
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_matches.RequestAllMatchesT{}).Pack(builder))

	response, err := debug.RequestAllMatches("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	expected := request_all_matches_response.RequestAllMatchesResponseT{
		MatchList: []*request_all_matches_response.MatchT{
			// MatchNumber, SetNumber, CompLevel
			// R1, R2, R3, B1, B2, B3
			{
				1, 1, "qm",
				"5", "42", "600", "971", "400", "200",
				&request_all_matches_response.ScoutedLevelT{
					// The R1 team has already been data
					// scouted.
					true, false, false, false, false, false,
				},
			},
			{
				2, 1, "qm",
				"6", "43", "601", "972", "401", "201",
				&request_all_matches_response.ScoutedLevelT{
					false, false, false, false, false, false,
				},
			},
			{
				3, 1, "qm",
				"7", "44", "602", "973", "402", "202",
				&request_all_matches_response.ScoutedLevelT{
					// The B1 team has already been data
					// scouted.
					false, false, false, true, false, false,
				},
			},
		},
	}
	if len(expected.MatchList) != len(response.MatchList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.MatchList {
		if !reflect.DeepEqual(*match, *response.MatchList[i]) {
			t.Fatal("Expected for match", i, ":", *match, ", but got:", *response.MatchList[i])
		}
	}

}

// Validates that we can request the 2023 stats.
func TestRequest2023DataScouting(t *testing.T) {
	db := MockDatabase{
		stats2023: []db.Stats2023{
			{
				TeamNumber: "3634", MatchNumber: 1, SetNumber: 2,
				CompLevel: "quals", StartingQuadrant: 3, LowCubesAuto: 10,
				MiddleCubesAuto: 1, HighCubesAuto: 1, CubesDroppedAuto: 0,
				LowConesAuto: 1, MiddleConesAuto: 2, HighConesAuto: 1,
				ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 1,
				HighCubes: 2, CubesDropped: 1, LowCones: 1,
				MiddleCones: 2, HighCones: 0, ConesDropped: 1, SuperchargedPieces: 0,
				AvgCycle: 34, Mobility: false, DockedAuto: true, EngagedAuto: false,
				BalanceAttemptAuto: false, Docked: false, Engaged: false,
				BalanceAttempt: true, CollectedBy: "isaac",
			},
			{
				TeamNumber: "2343", MatchNumber: 1, SetNumber: 2,
				CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
				MiddleCubesAuto: 1, HighCubesAuto: 1, CubesDroppedAuto: 2,
				LowConesAuto: 0, MiddleConesAuto: 0, HighConesAuto: 0,
				ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 0,
				HighCubes: 1, CubesDropped: 0, LowCones: 0,
				MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
				AvgCycle: 53, Mobility: false, DockedAuto: false, EngagedAuto: false,
				BalanceAttemptAuto: true, Docked: false, Engaged: false,
				BalanceAttempt: true, CollectedBy: "unknown",
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_2023_data_scouting.Request2023DataScoutingT{}).Pack(builder))

	response, err := debug.Request2023DataScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	expected := request_2023_data_scouting_response.Request2023DataScoutingResponseT{
		StatsList: []*request_2023_data_scouting_response.Stats2023T{
			{
				TeamNumber: "3634", MatchNumber: 1, SetNumber: 2,
				CompLevel: "quals", StartingQuadrant: 3, LowCubesAuto: 10,
				MiddleCubesAuto: 1, HighCubesAuto: 1, CubesDroppedAuto: 0,
				LowConesAuto: 1, MiddleConesAuto: 2, HighConesAuto: 1,
				ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 1,
				HighCubes: 2, CubesDropped: 1, LowCones: 1,
				MiddleCones: 2, HighCones: 0, ConesDropped: 1, SuperchargedPieces: 0,
				AvgCycle: 34, Mobility: false, DockedAuto: true, EngagedAuto: false,
				BalanceAttemptAuto: false, Docked: false, Engaged: false,
				BalanceAttempt: true, CollectedBy: "isaac",
			},
			{
				TeamNumber: "2343", MatchNumber: 1, SetNumber: 2,
				CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 0,
				MiddleCubesAuto: 1, HighCubesAuto: 1, CubesDroppedAuto: 2,
				LowConesAuto: 0, MiddleConesAuto: 0, HighConesAuto: 0,
				ConesDroppedAuto: 1, LowCubes: 0, MiddleCubes: 0,
				HighCubes: 1, CubesDropped: 0, LowCones: 0,
				MiddleCones: 2, HighCones: 1, ConesDropped: 1, SuperchargedPieces: 0,
				AvgCycle: 53, Mobility: false, DockedAuto: false, EngagedAuto: false,
				BalanceAttemptAuto: true, Docked: false, Engaged: false,
				BalanceAttempt: true, CollectedBy: "unknown",
			},
		},
	}
	if len(expected.StatsList) != len(response.StatsList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.StatsList {
		if !reflect.DeepEqual(*match, *response.StatsList[i]) {
			t.Fatal("Expected for stats", i, ":", *match, ", but got:", *response.StatsList[i])
		}
	}
}

// Validates that we can request the 2023 stats.
func TestConvertActionsToStat(t *testing.T) {
	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_actions.SubmitActionsT{
		TeamNumber:  "4244",
		MatchNumber: 3,
		SetNumber:   1,
		CompLevel:   "quals",
		ActionsList: []*submit_actions.ActionT{
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypeStartMatchAction,
					Value: &submit_actions.StartMatchActionT{
						Position: 1,
					},
				},
				Timestamp: 0,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePickupObjectAction,
					Value: &submit_actions.PickupObjectActionT{
						ObjectType: submit_actions.ObjectTypekCube,
						Auto:       true,
					},
				},
				Timestamp: 400,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePickupObjectAction,
					Value: &submit_actions.PickupObjectActionT{
						ObjectType: submit_actions.ObjectTypekCube,
						Auto:       true,
					},
				},
				Timestamp: 800,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePlaceObjectAction,
					Value: &submit_actions.PlaceObjectActionT{
						ObjectType: submit_actions.ObjectTypekCube,
						ScoreLevel: submit_actions.ScoreLevelkLow,
						Auto:       true,
					},
				},
				Timestamp: 2000,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypeMobilityAction,
					Value: &submit_actions.MobilityActionT{
						Mobility: true,
					},
				},
				Timestamp: 2200,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypeAutoBalanceAction,
					Value: &submit_actions.AutoBalanceActionT{
						Docked:         true,
						Engaged:        true,
						BalanceAttempt: false,
					},
				},
				Timestamp: 2400,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePickupObjectAction,
					Value: &submit_actions.PickupObjectActionT{
						ObjectType: submit_actions.ObjectTypekCone,
						Auto:       false,
					},
				},
				Timestamp: 2800,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePlaceObjectAction,
					Value: &submit_actions.PlaceObjectActionT{
						ObjectType: submit_actions.ObjectTypekCone,
						ScoreLevel: submit_actions.ScoreLevelkHigh,
						Auto:       false,
					},
				},
				Timestamp: 3100,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePickupObjectAction,
					Value: &submit_actions.PickupObjectActionT{
						ObjectType: submit_actions.ObjectTypekCube,
						Auto:       false,
					},
				},
				Timestamp: 3500,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePlaceObjectAction,
					Value: &submit_actions.PlaceObjectActionT{
						ObjectType: submit_actions.ObjectTypekCube,
						ScoreLevel: submit_actions.ScoreLevelkSupercharged,
						Auto:       false,
					},
				},
				Timestamp: 3900,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypeEndMatchAction,
					Value: &submit_actions.EndMatchActionT{
						Docked:         true,
						Engaged:        false,
						BalanceAttempt: true,
					},
				},
				Timestamp: 4200,
			},
		},
		PreScouting: false,
	}).Pack(builder))

	submitActions := submit_actions.GetRootAsSubmitActions(builder.FinishedBytes(), 0)
	response, err := ConvertActionsToStat(submitActions)

	if err != nil {
		t.Fatal("Failed to convert actions to stats: ", err)
	}

	expected := db.Stats2023{
		PreScouting: false,
		TeamNumber:  "4244", MatchNumber: 3, SetNumber: 1,
		CompLevel: "quals", StartingQuadrant: 1, LowCubesAuto: 1,
		MiddleCubesAuto: 0, HighCubesAuto: 0, CubesDroppedAuto: 1,
		LowConesAuto: 0, MiddleConesAuto: 0, HighConesAuto: 0,
		ConesDroppedAuto: 0, LowCubes: 0, MiddleCubes: 0,
		HighCubes: 0, CubesDropped: 0, LowCones: 0,
		MiddleCones: 0, HighCones: 1, ConesDropped: 0, SuperchargedPieces: 1,
		AvgCycle: 950, Mobility: true, DockedAuto: true, EngagedAuto: true,
		BalanceAttemptAuto: false, Docked: true, Engaged: false,
		BalanceAttempt: true, CollectedBy: "",
	}

	if expected != response {
		t.Fatal("Expected ", expected, ", but got ", response)
	}
}

func TestSubmitNotes(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&database, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_notes.SubmitNotesT{
		Team:           971,
		Notes:          "Notes",
		GoodDriving:    true,
		BadDriving:     false,
		SolidPlacing:   true,
		SketchyPlacing: false,
		GoodDefense:    true,
		BadDefense:     false,
		EasilyDefended: true,
	}).Pack(builder))

	_, err := debug.SubmitNotes("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit notes: ", err)
	}

	expected := []db.NotesData{
		{
			TeamNumber:     971,
			Notes:          "Notes",
			GoodDriving:    true,
			BadDriving:     false,
			SolidPlacing:   true,
			SketchyPlacing: false,
			GoodDefense:    true,
			BadDefense:     false,
			EasilyDefended: true,
		},
	}

	if !reflect.DeepEqual(database.notes, expected) {
		t.Fatal("Submitted notes did not match", expected, database.notes)
	}
}

func TestRequestNotes(t *testing.T) {
	database := MockDatabase{
		notes: []db.NotesData{{
			TeamNumber:     971,
			Notes:          "Notes",
			GoodDriving:    true,
			BadDriving:     false,
			SolidPlacing:   true,
			SketchyPlacing: false,
			GoodDefense:    true,
			BadDefense:     false,
			EasilyDefended: true,
		}},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&database, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_notes_for_team.RequestNotesForTeamT{
		Team: 971,
	}).Pack(builder))
	response, err := debug.RequestNotes("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit notes: ", err)
	}

	if response.Notes[0].Data != "Notes" {
		t.Fatal("requested notes did not match", response)
	}
}

func TestRequestShiftSchedule(t *testing.T) {
	db := MockDatabase{
		shiftSchedule: []db.Shift{
			{
				MatchNumber: 1,
				R1scouter:   "Bob",
				R2scouter:   "James",
				R3scouter:   "Robert",
				B1scouter:   "Alice",
				B2scouter:   "Mary",
				B3scouter:   "Patricia",
			},
			{
				MatchNumber: 2,
				R1scouter:   "Liam",
				R2scouter:   "Noah",
				R3scouter:   "Oliver",
				B1scouter:   "Emma",
				B2scouter:   "Charlotte",
				B3scouter:   "Amelia",
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_shift_schedule.RequestShiftScheduleT{}).Pack(builder))

	response, err := debug.RequestShiftSchedule("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request shift schedule: ", err)
	}

	expected := request_shift_schedule_response.RequestShiftScheduleResponseT{
		ShiftSchedule: []*request_shift_schedule_response.MatchAssignmentT{
			{
				MatchNumber: 1,
				R1scouter:   "Bob",
				R2scouter:   "James",
				R3scouter:   "Robert",
				B1scouter:   "Alice",
				B2scouter:   "Mary",
				B3scouter:   "Patricia",
			},
			{
				MatchNumber: 2,
				R1scouter:   "Liam",
				R2scouter:   "Noah",
				R3scouter:   "Oliver",
				B1scouter:   "Emma",
				B2scouter:   "Charlotte",
				B3scouter:   "Amelia",
			},
		},
	}
	if len(expected.ShiftSchedule) != len(response.ShiftSchedule) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.ShiftSchedule {
		if !reflect.DeepEqual(*match, *response.ShiftSchedule[i]) {
			t.Fatal("Expected for shift schedule", i, ":", *match, ", but got:", *response.ShiftSchedule[i])
		}
	}
}

func TestSubmitShiftSchedule(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&database, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_shift_schedule.SubmitShiftScheduleT{
		ShiftSchedule: []*submit_shift_schedule.MatchAssignmentT{
			{MatchNumber: 1,
				R1scouter: "Bob",
				R2scouter: "James",
				R3scouter: "Robert",
				B1scouter: "Alice",
				B2scouter: "Mary",
				B3scouter: "Patricia"},
		},
	}).Pack(builder))

	_, err := debug.SubmitShiftSchedule("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit shift schedule: ", err)
	}

	expected := []db.Shift{
		{MatchNumber: 1,
			R1scouter: "Bob",
			R2scouter: "James",
			R3scouter: "Robert",
			B1scouter: "Alice",
			B2scouter: "Mary",
			B3scouter: "Patricia"},
	}
	if !reflect.DeepEqual(expected, database.shiftSchedule) {
		t.Fatal("Expected ", expected, ", but got:", database.shiftSchedule)
	}
}

func TestSubmitDriverRanking(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&database, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_driver_ranking.SubmitDriverRankingT{
		MatchNumber: 36,
		Rank1:       1234,
		Rank2:       1235,
		Rank3:       1236,
	}).Pack(builder))

	_, err := debug.SubmitDriverRanking("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit driver ranking: ", err)
	}

	expected := []db.DriverRankingData{
		{MatchNumber: 36, Rank1: 1234, Rank2: 1235, Rank3: 1236},
	}

	if !reflect.DeepEqual(database.driver_ranking, expected) {
		t.Fatal("Submitted notes did not match", expected, database.notes)
	}
}

// Validates that we can request the driver rankings.
func TestRequestDriverRankings(t *testing.T) {
	db := MockDatabase{
		driver_ranking: []db.DriverRankingData{
			{
				MatchNumber: 36,
				Rank1:       1234,
				Rank2:       1235,
				Rank3:       1236,
			},
			{
				MatchNumber: 36,
				Rank1:       101,
				Rank2:       202,
				Rank3:       303,
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_driver_rankings.RequestAllDriverRankingsT{}).Pack(builder))

	response, err := debug.RequestAllDriverRankings("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all driver rankings: ", err)
	}

	expected := request_all_driver_rankings_response.RequestAllDriverRankingsResponseT{
		DriverRankingList: []*request_all_driver_rankings_response.RankingT{
			{
				MatchNumber: 36,
				Rank1:       1234,
				Rank2:       1235,
				Rank3:       1236,
			},
			{
				MatchNumber: 36,
				Rank1:       101,
				Rank2:       202,
				Rank3:       303,
			},
		},
	}
	if len(expected.DriverRankingList) != len(response.DriverRankingList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.DriverRankingList {
		if !reflect.DeepEqual(*match, *response.DriverRankingList[i]) {
			t.Fatal("Expected for driver ranking", i, ":", *match, ", but got:", *response.DriverRankingList[i])
		}
	}
}

// Validates that we can request all notes.
func TestRequestAllNotes(t *testing.T) {
	db := MockDatabase{
		notes: []db.NotesData{
			{
				TeamNumber:     971,
				Notes:          "Notes",
				GoodDriving:    true,
				BadDriving:     false,
				SolidPlacing:   true,
				SketchyPlacing: false,
				GoodDefense:    true,
				BadDefense:     false,
				EasilyDefended: false,
			},
			{
				TeamNumber:     972,
				Notes:          "More Notes",
				GoodDriving:    false,
				BadDriving:     false,
				SolidPlacing:   false,
				SketchyPlacing: true,
				GoodDefense:    false,
				BadDefense:     true,
				EasilyDefended: false,
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_notes.RequestAllNotesT{}).Pack(builder))

	response, err := debug.RequestAllNotes("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all notes: ", err)
	}

	expected := request_all_notes_response.RequestAllNotesResponseT{
		NoteList: []*request_all_notes_response.NoteT{
			{
				Team:           971,
				Notes:          "Notes",
				GoodDriving:    true,
				BadDriving:     false,
				SolidPlacing:   true,
				SketchyPlacing: false,
				GoodDefense:    true,
				BadDefense:     false,
				EasilyDefended: false,
			},
			{
				Team:           972,
				Notes:          "More Notes",
				GoodDriving:    false,
				BadDriving:     false,
				SolidPlacing:   false,
				SketchyPlacing: true,
				GoodDefense:    false,
				BadDefense:     true,
				EasilyDefended: false,
			},
		},
	}
	if len(expected.NoteList) != len(response.NoteList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, note := range expected.NoteList {
		if !reflect.DeepEqual(*note, *response.NoteList[i]) {
			t.Fatal("Expected for note", i, ":", *note, ", but got:", *response.NoteList[i])
		}
	}
}

func packAction(action *submit_actions.ActionT) []byte {
	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((action).Pack(builder))
	return (builder.FinishedBytes())
}

func TestAddingActions(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&database, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_actions.SubmitActionsT{
		TeamNumber:  "1234",
		MatchNumber: 4,
		SetNumber:   1,
		CompLevel:   "qual",
		ActionsList: []*submit_actions.ActionT{
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePickupObjectAction,
					Value: &submit_actions.PickupObjectActionT{
						ObjectType: submit_actions.ObjectTypekCube,
						Auto:       true,
					},
				},
				Timestamp: 2400,
			},
			{
				ActionTaken: &submit_actions.ActionTypeT{
					Type: submit_actions.ActionTypePlaceObjectAction,
					Value: &submit_actions.PlaceObjectActionT{
						ObjectType: submit_actions.ObjectTypekCube,
						ScoreLevel: submit_actions.ScoreLevelkLow,
						Auto:       false,
					},
				},
				Timestamp: 1009,
			},
		},
		PreScouting: true,
	}).Pack(builder))

	_, err := debug.SubmitActions("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit actions: ", err)
	}

	// Make sure that the data made it into the database.
	// TODO: Add this back when we figure out how to add the serialized action into the database.

	/* expectedActionsT := []*submit_actions.ActionT{
		{
			ActionTaken: &submit_actions.ActionTypeT{
				Type:	submit_actions.ActionTypePickupObjectAction,
				Value:	&submit_actions.PickupObjectActionT{
					ObjectType: submit_actions.ObjectTypekCube,
					Auto: true,
				},
			},
			Timestamp:       2400,
		},
		{
			ActionTaken: &submit_actions.ActionTypeT{
				Type:	submit_actions.ActionTypePlaceObjectAction,
				Value:	&submit_actions.PlaceObjectActionT{
					ObjectType: submit_actions.ObjectTypekCube,
					ScoreLevel: submit_actions.ScoreLevelkLow,
					Auto: false,
				},
			},
			Timestamp:       1009,
		},
	} */

	expectedActions := []db.Action{
		{
			PreScouting:     true,
			TeamNumber:      "1234",
			MatchNumber:     4,
			SetNumber:       1,
			CompLevel:       "qual",
			CollectedBy:     "debug_cli",
			CompletedAction: []byte{},
			TimeStamp:       2400,
		},
		{
			PreScouting:     true,
			TeamNumber:      "1234",
			MatchNumber:     4,
			SetNumber:       1,
			CompLevel:       "qual",
			CollectedBy:     "debug_cli",
			CompletedAction: []byte{},
			TimeStamp:       1009,
		},
	}

	expectedStats := []db.Stats2023{
		db.Stats2023{
			PreScouting: true,
			TeamNumber:  "1234", MatchNumber: 4, SetNumber: 1,
			CompLevel: "qual", StartingQuadrant: 0, LowCubesAuto: 0,
			MiddleCubesAuto: 0, HighCubesAuto: 0, CubesDroppedAuto: 0,
			LowConesAuto: 0, MiddleConesAuto: 0, HighConesAuto: 0,
			ConesDroppedAuto: 0, LowCubes: 1, MiddleCubes: 0,
			HighCubes: 0, CubesDropped: 0, LowCones: 0,
			MiddleCones: 0, HighCones: 0, ConesDropped: 0, SuperchargedPieces: 0,
			AvgCycle: 0, Mobility: false, DockedAuto: false, EngagedAuto: false,
			BalanceAttemptAuto: false, Docked: false, Engaged: false,
			BalanceAttempt: false, CollectedBy: "debug_cli",
		},
	}

	if !reflect.DeepEqual(expectedActions, database.actions) {
		t.Fatal("Expected ", expectedActions, ", but got:", database.actions)
	}
	if !reflect.DeepEqual(expectedStats, database.stats2023) {
		t.Fatal("Expected ", expectedStats, ", but got:", database.stats2023)
	}
}

// A mocked database we can use for testing. Add functionality to this as
// needed for your tests.

type MockDatabase struct {
	matches        []db.TeamMatch
	notes          []db.NotesData
	shiftSchedule  []db.Shift
	driver_ranking []db.DriverRankingData
	stats2023      []db.Stats2023
	actions        []db.Action
}

func (database *MockDatabase) AddToMatch(match db.TeamMatch) error {
	database.matches = append(database.matches, match)
	return nil
}

func (database *MockDatabase) AddToStats2023(stats2023 db.Stats2023) error {
	database.stats2023 = append(database.stats2023, stats2023)
	return nil
}
func (database *MockDatabase) ReturnMatches() ([]db.TeamMatch, error) {
	return database.matches, nil
}

func (database *MockDatabase) ReturnStats2023() ([]db.Stats2023, error) {
	return database.stats2023, nil
}

func (database *MockDatabase) ReturnStats2023ForTeam(teamNumber string, matchNumber int32, setNumber int32, compLevel string, preScouting bool) ([]db.Stats2023, error) {
	var results []db.Stats2023
	for _, stats := range database.stats2023 {
		if stats.TeamNumber == teamNumber && stats.MatchNumber == matchNumber && stats.SetNumber == setNumber && stats.CompLevel == compLevel && stats.PreScouting == preScouting {
			results = append(results, stats)
		}
	}
	return results, nil
}

func (database *MockDatabase) QueryNotes(requestedTeam int32) ([]string, error) {
	var results []string
	for _, data := range database.notes {
		if data.TeamNumber == requestedTeam {
			results = append(results, data.Notes)
		}
	}
	return results, nil
}

func (database *MockDatabase) AddNotes(data db.NotesData) error {
	database.notes = append(database.notes, data)
	return nil
}

func (database *MockDatabase) ReturnAllNotes() ([]db.NotesData, error) {
	return database.notes, nil
}

func (database *MockDatabase) AddToShift(data db.Shift) error {
	database.shiftSchedule = append(database.shiftSchedule, data)
	return nil
}

func (database *MockDatabase) ReturnAllShifts() ([]db.Shift, error) {
	return database.shiftSchedule, nil
}

func (database *MockDatabase) QueryAllShifts(int) ([]db.Shift, error) {
	return []db.Shift{}, nil
}

func (database *MockDatabase) AddDriverRanking(data db.DriverRankingData) error {
	database.driver_ranking = append(database.driver_ranking, data)
	return nil
}

func (database *MockDatabase) ReturnAllDriverRankings() ([]db.DriverRankingData, error) {
	return database.driver_ranking, nil
}

func (database *MockDatabase) AddAction(action db.Action) error {
	database.actions = append(database.actions, action)
	return nil
}

func (database *MockDatabase) ReturnActions() ([]db.Action, error) {
	return database.actions, nil
}
