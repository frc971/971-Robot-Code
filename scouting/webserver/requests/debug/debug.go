package debug

import (
	"bytes"
	"encoding/base64"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"

	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2023_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule_response"
	flatbuffers "github.com/google/flatbuffers/go"
)

// The username to submit the various requests as.
const DefaultUsername = "debug_cli"

// A struct that can be used as an `error`. It contains information about the
// why the server was unhappy and what the corresponding request was.
type ResponseError struct {
	Url           string
	StatusCode    int
	ErrorResponse *error_response.ErrorResponse
}

// Required to implement the `error` interface.
func (err *ResponseError) Error() string {
	return fmt.Sprintf(
		"%s returned %d %s: %s", err.Url, err.StatusCode,
		http.StatusText(err.StatusCode), err.ErrorResponse.ErrorMessage())
}

// Parse an `ErrorResponse` message that the server sent back. This happens
// whenever the status code is something other than 200. If the message is
// successfully parsed, it's turned into a `ResponseError` which implements the
// `error` interface.
func parseErrorResponse(url string, statusCode int, responseBytes []byte) error {
	getRootErrMessage := ""
	defer func() {
		if r := recover(); r != nil {
			getRootErrMessage = fmt.Sprintf("%v", r)
		}
	}()
	errorMessage := error_response.GetRootAsErrorResponse(responseBytes, 0)
	if getRootErrMessage != "" {
		return errors.New(fmt.Sprintf(
			"Failed to parse response from %s with status %d %s (bytes %v) as ErrorResponse: %s",
			url, statusCode, http.StatusText(statusCode), responseBytes, getRootErrMessage))
	}

	return &ResponseError{
		Url:           url,
		StatusCode:    statusCode,
		ErrorResponse: errorMessage,
	}
}

// Performs a POST request with the specified payload. The bytes that the
// server responds with are returned.
func performPost(url string, requestBytes []byte) ([]byte, error) {
	req, err := http.NewRequest("POST", url, bytes.NewReader(requestBytes))
	if err != nil {
		log.Printf("Failed to create a new POST request to %s: %v", url, err)
		return nil, err
	}
	req.Header.Add("Authorization", "Basic "+
		base64.StdEncoding.EncodeToString([]byte(DefaultUsername+":")))

	client := &http.Client{}
	resp, err := client.Do(req)
	if err != nil {
		log.Printf("Failed to send POST request to %s: %v", url, err)
		return nil, err
	}
	responseBytes, err := io.ReadAll(resp.Body)
	if err != nil {
		log.Printf("Failed to parse response bytes from POST to %s: %v", url, err)
		return nil, err
	}
	if resp.StatusCode != http.StatusOK {
		return nil, parseErrorResponse(url, resp.StatusCode, responseBytes)
	}
	return responseBytes, nil
}

// Sends a message to the server and returns the deserialized response.
// The first generic argument must be specified.
func sendMessage[FbT interface{}, Fb interface{ UnPack() *FbT }](url string, requestBytes []byte, parser func([]byte, flatbuffers.UOffsetT) Fb) (*FbT, error) {
	responseBytes, err := performPost(url, requestBytes)
	if err != nil {
		return nil, err
	}
	response := parser(responseBytes, 0)
	return response.UnPack(), nil
}

func RequestAllMatches(server string, requestBytes []byte) (*request_all_matches_response.RequestAllMatchesResponseT, error) {
	return sendMessage[request_all_matches_response.RequestAllMatchesResponseT](
		server+"/requests/request/all_matches", requestBytes,
		request_all_matches_response.GetRootAsRequestAllMatchesResponse)
}

func RequestAllDriverRankings(server string, requestBytes []byte) (*request_all_driver_rankings_response.RequestAllDriverRankingsResponseT, error) {
	return sendMessage[request_all_driver_rankings_response.RequestAllDriverRankingsResponseT](
		server+"/requests/request/all_driver_rankings", requestBytes,
		request_all_driver_rankings_response.GetRootAsRequestAllDriverRankingsResponse)
}

func Request2023DataScouting(server string, requestBytes []byte) (*request_2023_data_scouting_response.Request2023DataScoutingResponseT, error) {
	return sendMessage[request_2023_data_scouting_response.Request2023DataScoutingResponseT](
		server+"/requests/request/2023_data_scouting", requestBytes,
		request_2023_data_scouting_response.GetRootAsRequest2023DataScoutingResponse)
}

func SubmitNotes(server string, requestBytes []byte) (*submit_notes_response.SubmitNotesResponseT, error) {
	return sendMessage[submit_notes_response.SubmitNotesResponseT](
		server+"/requests/submit/submit_notes", requestBytes,
		submit_notes_response.GetRootAsSubmitNotesResponse)
}

func RequestNotes(server string, requestBytes []byte) (*request_notes_for_team_response.RequestNotesForTeamResponseT, error) {
	return sendMessage[request_notes_for_team_response.RequestNotesForTeamResponseT](
		server+"/requests/request/notes_for_team", requestBytes,
		request_notes_for_team_response.GetRootAsRequestNotesForTeamResponse)
}

func RequestAllNotes(server string, requestBytes []byte) (*request_all_notes_response.RequestAllNotesResponseT, error) {
	return sendMessage[request_all_notes_response.RequestAllNotesResponseT](
		server+"/requests/request/all_notes", requestBytes,
		request_all_notes_response.GetRootAsRequestAllNotesResponse)
}

func RequestShiftSchedule(server string, requestBytes []byte) (*request_shift_schedule_response.RequestShiftScheduleResponseT, error) {
	return sendMessage[request_shift_schedule_response.RequestShiftScheduleResponseT](
		server+"/requests/request/shift_schedule", requestBytes,
		request_shift_schedule_response.GetRootAsRequestShiftScheduleResponse)
}

func SubmitShiftSchedule(server string, requestBytes []byte) (*submit_shift_schedule_response.SubmitShiftScheduleResponseT, error) {
	return sendMessage[submit_shift_schedule_response.SubmitShiftScheduleResponseT](
		server+"/requests/submit/shift_schedule", requestBytes,
		submit_shift_schedule_response.GetRootAsSubmitShiftScheduleResponse)
}

func SubmitDriverRanking(server string, requestBytes []byte) (*submit_driver_ranking_response.SubmitDriverRankingResponseT, error) {
	return sendMessage[submit_driver_ranking_response.SubmitDriverRankingResponseT](
		server+"/requests/submit/submit_driver_ranking", requestBytes,
		submit_driver_ranking_response.GetRootAsSubmitDriverRankingResponse)
}

func SubmitActions(server string, requestBytes []byte) (*submit_actions_response.SubmitActionsResponseT, error) {
	return sendMessage[submit_actions_response.SubmitActionsResponseT](
		server+"/requests/submit/submit_actions", requestBytes,
		submit_actions_response.GetRootAsSubmitActionsResponse)
}
