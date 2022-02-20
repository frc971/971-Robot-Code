package requests

import (
	"bytes"
	"io"
	"net/http"
	"testing"

	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	_ "github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

// Validates that an unhandled address results in a 404.
func Test404(t *testing.T) {
	scoutingServer := server.NewScoutingServer()
	HandleRequests(scoutingServer)
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
	scoutingServer := server.NewScoutingServer()
	HandleRequests(scoutingServer)
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
	scoutingServer := server.NewScoutingServer()
	HandleRequests(scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_data_scouting.SubmitDataScoutingT{
		Team:          971,
		Match:         1,
		UpperGoalHits: 9971,
	}).Pack(builder))

	resp, err := http.Post("http://localhost:8080/requests/submit/data_scouting", "application/octet-stream", bytes.NewReader(builder.FinishedBytes()))
	if err != nil {
		t.Fatalf("Failed to send request: %v", err)
	}
	if resp.StatusCode != http.StatusNotImplemented {
		t.Fatal("Unexpected status code. Got", resp.Status)
	}
	// TODO(phil): We have nothing to validate yet. Fix that.
}
