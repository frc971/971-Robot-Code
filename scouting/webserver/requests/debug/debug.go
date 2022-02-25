package debug

import (
	"bytes"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"

	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
)

// Use aliases to make the rest of the code more readable.
type SubmitDataScoutingResponseT = submit_data_scouting_response.SubmitDataScoutingResponseT

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
	resp, err := http.Post(url, "application/octet-stream", bytes.NewReader(requestBytes))
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
func SubmitDataScouting(server string, requestBytes []byte) (*SubmitDataScoutingResponseT, error) {
	responseBytes, err := performPost(server+"/requests/submit/data_scouting", requestBytes)
	if err != nil {
		return nil, err
	}
	log.Printf("Parsing SubmitDataScoutingResponse")
	response := submit_data_scouting_response.GetRootAsSubmitDataScoutingResponse(responseBytes, 0)
	return response.UnPack(), nil
}
