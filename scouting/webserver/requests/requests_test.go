package requests

import (
	"bytes"
	"io"
	"net/http"
	"reflect"
	"testing"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/debug"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	_ "github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
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

// Validates that we can submit new data scouting data.
func TestSubmitDataScoutingError(t *testing.T) {
	db := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	resp, err := http.Post("http://localhost:8080/requests/submit/data_scouting", "application/octet-stream", bytes.NewReader([]byte("")))
	if err != nil {
		t.Fatalf("Failed to send request: %v", err)
	}
	if resp.StatusCode != http.StatusBadRequest {
		t.Fatal("Unexpected status code. Got", resp.Status)
	}

	responseBytes, err := io.ReadAll(resp.Body)
	if err != nil {
		t.Fatal("Failed to read response bytes:", err)
	}
	errorResponse := error_response.GetRootAsErrorResponse(responseBytes, 0)

	errorMessage := string(errorResponse.ErrorMessage())
	if errorMessage != "Failed to parse SubmitDataScouting: runtime error: index out of range [3] with length 0" {
		t.Fatal("Got mismatched error message:", errorMessage)
	}
}

// Validates that we can submit new data scouting data.
func TestSubmitDataScouting(t *testing.T) {
	db := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_data_scouting.SubmitDataScoutingT{
		Team:            971,
		Match:           1,
		MissedShotsAuto: 9971,
		UpperGoalAuto:   9971,
		LowerGoalAuto:   9971,
		MissedShotsTele: 9971,
		UpperGoalTele:   9971,
		LowerGoalTele:   9971,
		DefenseRating:   9971,
		Climbing:        9971,
	}).Pack(builder))

	resp, err := http.Post("http://localhost:8080/requests/submit/data_scouting", "application/octet-stream", bytes.NewReader(builder.FinishedBytes()))
	if err != nil {
		t.Fatalf("Failed to send request: %v", err)
	}
	if resp.StatusCode != http.StatusNotImplemented {
		t.Fatal("Unexpected status code. Got", resp.Status)
	}
	// TODO(phil): We have nothing to validate yet. Fix that.
	// TODO(phil): Can we use scouting/webserver/requests/debug here?
}

// Validates that we can request the full match list.
func TestRequestAllMatches(t *testing.T) {
	db := MockDatabase{
		matches: []db.Match{
			{
				MatchNumber: 1, Round: 1, CompLevel: "qual",
				R1: 5, R2: 42, R3: 600, B1: 971, B2: 400, B3: 200,
			},
			{
				MatchNumber: 2, Round: 1, CompLevel: "qual",
				R1: 6, R2: 43, R3: 601, B1: 972, B2: 401, B3: 201,
			},
			{
				MatchNumber: 3, Round: 1, CompLevel: "qual",
				R1: 7, R2: 44, R3: 602, B1: 973, B2: 402, B3: 202,
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
			// MatchNumber, Round, CompLevel
			// R1, R2, R3, B1, B2, B3
			{
				1, 1, "qual",
				5, 42, 600, 971, 400, 200,
			},
			{
				2, 1, "qual",
				6, 43, 601, 972, 401, 201,
			},
			{
				3, 1, "qual",
				7, 44, 602, 973, 402, 202,
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

// A mocked database we can use for testing. Add functionality to this as
// needed for your tests.

type MockDatabase struct {
	matches []db.Match
}

func (database *MockDatabase) AddToMatch(db.Match) error {
	return nil
}

func (database *MockDatabase) AddToStats(db.Stats) error {
	return nil
}

func (database *MockDatabase) ReturnMatches() ([]db.Match, error) {
	return database.matches, nil
}

func (database *MockDatabase) ReturnStats() ([]db.Stats, error) {
	return []db.Stats{}, nil
}

func (database *MockDatabase) QueryMatches(int) ([]db.Match, error) {
	return []db.Match{}, nil
}

func (database *MockDatabase) QueryStats(int) ([]db.Stats, error) {
	return []db.Stats{}, nil
}
