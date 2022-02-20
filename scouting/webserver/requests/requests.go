package requests

import (
	"fmt"
	"io"
	"net/http"

	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	_ "github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

// Handles unknown requests. Just returns a 404.
func unknown(w http.ResponseWriter, req *http.Request) {
	w.WriteHeader(http.StatusNotFound)
}

func respondWithError(w http.ResponseWriter, statusCode int, errorMessage string) {
	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&error_response.ErrorResponseT{
		ErrorMessage: errorMessage,
	}).Pack(builder))
	w.WriteHeader(statusCode)
	w.Write(builder.FinishedBytes())
}

func respondNotImplemented(w http.ResponseWriter) {
	respondWithError(w, http.StatusNotImplemented, "")
}

// TODO(phil): Can we turn this into a generic?
func parseSubmitDataScouting(w http.ResponseWriter, buf []byte) (*submit_data_scouting.SubmitDataScouting, bool) {
	success := true
	defer func() {
		if r := recover(); r != nil {
			respondWithError(w, http.StatusBadRequest, fmt.Sprintf("Failed to parse SubmitDataScouting: %v", r))
			success = false
		}
	}()
	result := submit_data_scouting.GetRootAsSubmitDataScouting(buf, 0)
	return result, success
}

// Handles a SubmitDataScouting request.
func submitDataScouting(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseSubmitDataScouting(w, requestBytes)
	if !success {
		return
	}

	// TODO(phil): Actually handle the request.

	respondNotImplemented(w)
}

func HandleRequests(scoutingServer server.ScoutingServer) {
	scoutingServer.HandleFunc("/requests", unknown)
	scoutingServer.HandleFunc("/requests/submit/data_scouting", submitDataScouting)
}
