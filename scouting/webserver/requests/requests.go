package requests

import (
	"fmt"
	"io"
	"net/http"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	_ "github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

// The interface we expect the database abstraction to conform to.
// We use an interface here because it makes unit testing easier.
type Database interface {
	AddToMatch(db.Match) error
	AddToStats(db.Stats) error
	ReturnMatches() ([]db.Match, error)
	ReturnStats() ([]db.Stats, error)
	QueryMatches(int) ([]db.Match, error)
	QueryStats(int) ([]db.Stats, error)
}

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
type submitDataScoutingHandler struct {
	db Database
}

func (handler submitDataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
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
	// We have access to the database via "handler.db" here. For example:
	// stats := handler.db.ReturnStats()

	respondNotImplemented(w)
}

func HandleRequests(db Database, scoutingServer server.ScoutingServer) {
	scoutingServer.HandleFunc("/requests", unknown)
	scoutingServer.Handle("/requests/submit/data_scouting", submitDataScoutingHandler{db})
}
