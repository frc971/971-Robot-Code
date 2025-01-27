package debug

import (
	"bytes"
	"encoding/base64"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"

	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_2024_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2024_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2025_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_pit_images_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_averaged_driver_rankings_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_averaged_human_rankings_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_current_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_pit_images_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_2024_actions_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_2025_actions_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_human_ranking_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_pit_image_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule_response"
	flatbuffers "github.com/google/flatbuffers/go"
)

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
func performPost(url string, requestBytes []byte, userName string) ([]byte, error) {
	req, err := http.NewRequest("POST", url, bytes.NewReader(requestBytes))
	if err != nil {
		log.Printf("Failed to create a new POST request to %s: %v", url, err)
		return nil, err
	}
	req.Header.Add("Authorization", "Basic "+
		base64.StdEncoding.EncodeToString([]byte(userName+":")))

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
func sendMessage[FbT interface{}, Fb interface{ UnPack() *FbT }](url string, requestBytes []byte, parser func([]byte, flatbuffers.UOffsetT) Fb, userName string) (*FbT, error) {
	responseBytes, err := performPost(url, requestBytes, userName)
	if err != nil {
		return nil, err
	}
	response := parser(responseBytes, 0)
	return response.UnPack(), nil
}

func RequestAllMatches(server string, requestBytes []byte, userName string) (*request_all_matches_response.RequestAllMatchesResponseT, error) {
	return sendMessage[request_all_matches_response.RequestAllMatchesResponseT](
		server+"/requests/request/all_matches", requestBytes,
		request_all_matches_response.GetRootAsRequestAllMatchesResponse, DefaultUsername)
}

func RequestAllDriverRankings(server string, requestBytes []byte, userName string) (*request_all_driver_rankings_response.RequestAllDriverRankingsResponseT, error) {
	return sendMessage[request_all_driver_rankings_response.RequestAllDriverRankingsResponseT](
		server+"/requests/request/all_driver_rankings", requestBytes,
		request_all_driver_rankings_response.GetRootAsRequestAllDriverRankingsResponse, DefaultUsername)
}

func RequestAveragedDriverRankings2025(server string, requestBytes []byte, userName string) (*request_averaged_driver_rankings_2025_response.RequestAveragedDriverRankings2025ResponseT, error) {
	return sendMessage[request_averaged_driver_rankings_2025_response.RequestAveragedDriverRankings2025ResponseT](
		server+"/requests/request/averaged_driver_rankings_2025", requestBytes,
		request_averaged_driver_rankings_2025_response.GetRootAsRequestAveragedDriverRankings2025Response, DefaultUsername)
}

func RequestAveragedHumanRankings2025(server string, requestBytes []byte, userName string) (*request_averaged_human_rankings_2025_response.RequestAveragedHumanRankings2025ResponseT, error) {
	return sendMessage[request_averaged_human_rankings_2025_response.RequestAveragedHumanRankings2025ResponseT](
		server+"/requests/request/averaged_human_rankings_2025", requestBytes,
		request_averaged_human_rankings_2025_response.GetRootAsRequestAveragedHumanRankings2025Response, DefaultUsername)
}

func Request2024DataScouting(server string, requestBytes []byte, userName string) (*request_2024_data_scouting_response.Request2024DataScoutingResponseT, error) {
	return sendMessage[request_2024_data_scouting_response.Request2024DataScoutingResponseT](
		server+"/requests/request/2024_data_scouting", requestBytes,
		request_2024_data_scouting_response.GetRootAsRequest2024DataScoutingResponse, DefaultUsername)
}

func Request2025DataScouting(server string, requestBytes []byte, userName string) (*request_2025_data_scouting_response.Request2025DataScoutingResponseT, error) {
	return sendMessage[request_2025_data_scouting_response.Request2025DataScoutingResponseT](
		server+"/requests/request/2025_data_scouting", requestBytes,
		request_2025_data_scouting_response.GetRootAsRequest2025DataScoutingResponse, DefaultUsername)
}

func SubmitNotes(server string, requestBytes []byte, userName string) (*submit_notes_response.SubmitNotesResponseT, error) {
	return sendMessage[submit_notes_response.SubmitNotesResponseT](
		server+"/requests/submit/submit_notes", requestBytes,
		submit_notes_response.GetRootAsSubmitNotesResponse, DefaultUsername)
}

func RequestNotes(server string, requestBytes []byte, userName string) (*request_notes_for_team_response.RequestNotesForTeamResponseT, error) {
	return sendMessage[request_notes_for_team_response.RequestNotesForTeamResponseT](
		server+"/requests/request/notes_for_team", requestBytes,
		request_notes_for_team_response.GetRootAsRequestNotesForTeamResponse, DefaultUsername)
}

func RequestPitImages(server string, requestBytes []byte, userName string) (*request_pit_images_response.RequestPitImagesResponseT, error) {
	return sendMessage[request_pit_images_response.RequestPitImagesResponseT](
		server+"/requests/request/pit_images", requestBytes,
		request_pit_images_response.GetRootAsRequestPitImagesResponse, DefaultUsername)
}

func RequestAllPitImages(server string, requestBytes []byte, userName string) (*request_all_pit_images_response.RequestAllPitImagesResponseT, error) {
	return sendMessage[request_all_pit_images_response.RequestAllPitImagesResponseT](
		server+"/requests/request/all_pit_images", requestBytes,
		request_all_pit_images_response.GetRootAsRequestAllPitImagesResponse, DefaultUsername)
}

func RequestAllNotes(server string, requestBytes []byte, userName string) (*request_all_notes_response.RequestAllNotesResponseT, error) {
	return sendMessage[request_all_notes_response.RequestAllNotesResponseT](
		server+"/requests/request/all_notes", requestBytes,
		request_all_notes_response.GetRootAsRequestAllNotesResponse, DefaultUsername)
}

func RequestShiftSchedule(server string, requestBytes []byte, userName string) (*request_shift_schedule_response.RequestShiftScheduleResponseT, error) {
	return sendMessage[request_shift_schedule_response.RequestShiftScheduleResponseT](
		server+"/requests/request/shift_schedule", requestBytes,
		request_shift_schedule_response.GetRootAsRequestShiftScheduleResponse, DefaultUsername)
}

func RequestCurrentScouting(server string, requestBytes []byte, userName string) (*request_current_scouting_response.RequestCurrentScoutingResponseT, error) {
	return sendMessage[request_current_scouting_response.RequestCurrentScoutingResponseT](
		server+"/requests/request/current_scouting", requestBytes,
		request_current_scouting_response.GetRootAsRequestCurrentScoutingResponse, userName)
}

func SubmitShiftSchedule(server string, requestBytes []byte, userName string) (*submit_shift_schedule_response.SubmitShiftScheduleResponseT, error) {
	return sendMessage[submit_shift_schedule_response.SubmitShiftScheduleResponseT](
		server+"/requests/submit/shift_schedule", requestBytes,
		submit_shift_schedule_response.GetRootAsSubmitShiftScheduleResponse, DefaultUsername)
}

func SubmitDriverRanking(server string, requestBytes []byte, userName string) (*submit_driver_ranking_response.SubmitDriverRankingResponseT, error) {
	return sendMessage[submit_driver_ranking_response.SubmitDriverRankingResponseT](
		server+"/requests/submit/submit_driver_ranking", requestBytes,
		submit_driver_ranking_response.GetRootAsSubmitDriverRankingResponse, DefaultUsername)
}

func SubmitDriverRanking2025(server string, requestBytes []byte, userName string) (*submit_driver_ranking_2025_response.SubmitDriverRanking2025ResponseT, error) {
	return sendMessage[submit_driver_ranking_2025_response.SubmitDriverRanking2025ResponseT](
		server+"/requests/submit/submit_driver_ranking_2025", requestBytes,
		submit_driver_ranking_2025_response.GetRootAsSubmitDriverRanking2025Response, DefaultUsername)
}

func SubmitHumanRanking2025(server string, requestBytes []byte, userName string) (*submit_human_ranking_2025_response.SubmitHumanRanking2025ResponseT, error) {
	return sendMessage[submit_human_ranking_2025_response.SubmitHumanRanking2025ResponseT](
		server+"/requests/submit/submit_human_ranking_2025", requestBytes,
		submit_human_ranking_2025_response.GetRootAsSubmitHumanRanking2025Response, DefaultUsername)
}

func Submit2024Actions(server string, requestBytes []byte, userName string) (*submit_2024_actions_response.Submit2024ActionsResponseT, error) {
	return sendMessage[submit_2024_actions_response.Submit2024ActionsResponseT](
		server+"/requests/submit/submit_2024_actions", requestBytes,
		submit_2024_actions_response.GetRootAsSubmit2024ActionsResponse, DefaultUsername)
}

func Submit2025Actions(server string, requestBytes []byte, userName string) (*submit_2025_actions_response.Submit2025ActionsResponseT, error) {
	return sendMessage[submit_2025_actions_response.Submit2025ActionsResponseT](
		server+"/requests/submit/submit_2025_actions", requestBytes,
		submit_2025_actions_response.GetRootAsSubmit2025ActionsResponse, DefaultUsername)
}

func SubmitPitImage(server string, requestBytes []byte, userName string) (*submit_pit_image_response.SubmitPitImageResponseT, error) {
	return sendMessage[submit_pit_image_response.SubmitPitImageResponseT](
		server+"/requests/submit/submit_pit_image", requestBytes,
		submit_pit_image_response.GetRootAsSubmitPitImageResponse, DefaultUsername)
}

func Delete2024DataScouting(server string, requestBytes []byte, userName string) (*delete_2024_data_scouting_response.Delete2024DataScoutingResponseT, error) {
	return sendMessage[delete_2024_data_scouting_response.Delete2024DataScoutingResponseT](
		server+"/requests/delete/delete_2024_data_scouting", requestBytes,
		delete_2024_data_scouting_response.GetRootAsDelete2024DataScoutingResponse, DefaultUsername)
}
