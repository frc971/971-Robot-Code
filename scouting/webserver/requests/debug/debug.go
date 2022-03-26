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
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response"
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

// Sends a `SubmitDataScouting` message to the server and returns the
// deserialized response.
func SubmitDataScouting(server string, requestBytes []byte) (*submit_data_scouting_response.SubmitDataScoutingResponseT, error) {
	responseBytes, err := performPost(server+"/requests/submit/data_scouting", requestBytes)
	if err != nil {
		return nil, err
	}
	log.Printf("Parsing SubmitDataScoutingResponse")
	response := submit_data_scouting_response.GetRootAsSubmitDataScoutingResponse(responseBytes, 0)
	return response.UnPack(), nil
}

// Sends a `RequestAllMatches` message to the server and returns the
// deserialized response.
func RequestAllMatches(server string, requestBytes []byte) (*request_all_matches_response.RequestAllMatchesResponseT, error) {
	responseBytes, err := performPost(server+"/requests/request/all_matches", requestBytes)
	if err != nil {
		return nil, err
	}
	log.Printf("Parsing RequestAllMatchesResponse")
	response := request_all_matches_response.GetRootAsRequestAllMatchesResponse(responseBytes, 0)
	return response.UnPack(), nil
}

// Sends a `RequestMatchesForTeam` message to the server and returns the
// deserialized response.
func RequestMatchesForTeam(server string, requestBytes []byte) (*request_matches_for_team_response.RequestMatchesForTeamResponseT, error) {
	responseBytes, err := performPost(server+"/requests/request/matches_for_team", requestBytes)
	if err != nil {
		return nil, err
	}
	log.Printf("Parsing RequestMatchesForTeamResponse")
	response := request_matches_for_team_response.GetRootAsRequestMatchesForTeamResponse(responseBytes, 0)
	return response.UnPack(), nil
}

// Sends a `RequestDataScouting` message to the server and returns the
// deserialized response.
func RequestDataScouting(server string, requestBytes []byte) (*request_data_scouting_response.RequestDataScoutingResponseT, error) {
	responseBytes, err := performPost(server+"/requests/request/data_scouting", requestBytes)
	if err != nil {
		return nil, err
	}
	log.Printf("Parsing RequestDataScoutingResponse")
	response := request_data_scouting_response.GetRootAsRequestDataScoutingResponse(responseBytes, 0)
	return response.UnPack(), nil
}

// Sends a `RefreshMatchList` message to the server and returns the
// deserialized response.
func RefreshMatchList(server string, requestBytes []byte) (*refresh_match_list_response.RefreshMatchListResponseT, error) {
	responseBytes, err := performPost(server+"/requests/refresh_match_list", requestBytes)
	if err != nil {
		return nil, err
	}
	log.Printf("Parsing RefreshMatchListResponse")
	response := refresh_match_list_response.GetRootAsRefreshMatchListResponse(responseBytes, 0)
	return response.UnPack(), nil
}

func SubmitNotes(server string, requestBytes []byte) (*submit_notes_response.SubmitNotesResponseT, error) {
	responseBytes, err := performPost(server+"/requests/submit/submit_notes", requestBytes)
	if err != nil {
		return nil, err
	}

	response := submit_notes_response.GetRootAsSubmitNotesResponse(responseBytes, 0)
	return response.UnPack(), nil
}

func RequestNotes(server string, requestBytes []byte) (*request_notes_for_team_response.RequestNotesForTeamResponseT, error) {
	responseBytes, err := performPost(server+"/requests/request/notes_for_team", requestBytes)
	if err != nil {
		return nil, err
	}

	response := request_notes_for_team_response.GetRootAsRequestNotesForTeamResponse(responseBytes, 0)
	return response.UnPack(), nil
}
