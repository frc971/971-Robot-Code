package requests

import (
	"fmt"
	"io"
	"net/http"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	_ "github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

type SubmitDataScouting = submit_data_scouting.SubmitDataScouting
type RequestAllMatches = request_all_matches.RequestAllMatches
type RequestAllMatchesResponseT = request_all_matches_response.RequestAllMatchesResponseT
type RequestMatchesForTeam = request_matches_for_team.RequestMatchesForTeam
type RequestMatchesForTeamResponseT = request_matches_for_team_response.RequestMatchesForTeamResponseT
type RequestDataScouting = request_data_scouting.RequestDataScouting
type RequestDataScoutingResponseT = request_data_scouting_response.RequestDataScoutingResponseT

// The interface we expect the database abstraction to conform to.
// We use an interface here because it makes unit testing easier.
type Database interface {
	AddToMatch(db.Match) error
	AddToStats(db.Stats) error
	ReturnMatches() ([]db.Match, error)
	ReturnStats() ([]db.Stats, error)
	QueryMatches(int32) ([]db.Match, error)
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
func parseSubmitDataScouting(w http.ResponseWriter, buf []byte) (*SubmitDataScouting, bool) {
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

// TODO(phil): Can we turn this into a generic?
func parseRequestAllMatches(w http.ResponseWriter, buf []byte) (*RequestAllMatches, bool) {
	success := true
	defer func() {
		if r := recover(); r != nil {
			respondWithError(w, http.StatusBadRequest, fmt.Sprintf("Failed to parse SubmitDataScouting: %v", r))
			success = false
		}
	}()
	result := request_all_matches.GetRootAsRequestAllMatches(buf, 0)
	return result, success
}

// Handles a RequestAllMaches request.
type requestAllMatchesHandler struct {
	db Database
}

func (handler requestAllMatchesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequestAllMatches(w, requestBytes)
	if !success {
		return
	}

	matches, err := handler.db.ReturnMatches()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Faled to query database: ", err))
		return
	}

	var response RequestAllMatchesResponseT
	for _, match := range matches {
		response.MatchList = append(response.MatchList, &request_all_matches_response.MatchT{
			MatchNumber: match.MatchNumber,
			Round:       match.Round,
			CompLevel:   match.CompLevel,
			R1:          match.R1,
			R2:          match.R2,
			R3:          match.R3,
			B1:          match.B1,
			B2:          match.B2,
			B3:          match.B3,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

// TODO(phil): Can we turn this into a generic?
func parseRequestMatchesForTeam(w http.ResponseWriter, buf []byte) (*RequestMatchesForTeam, bool) {
	success := true
	defer func() {
		if r := recover(); r != nil {
			respondWithError(w, http.StatusBadRequest, fmt.Sprintf("Failed to parse SubmitDataScouting: %v", r))
			success = false
		}
	}()
	result := request_matches_for_team.GetRootAsRequestMatchesForTeam(buf, 0)
	return result, success
}

// Handles a RequestMatchesForTeam request.
type requestMatchesForTeamHandler struct {
	db Database
}

func (handler requestMatchesForTeamHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequestMatchesForTeam(w, requestBytes)
	if !success {
		return
	}

	matches, err := handler.db.QueryMatches(request.Team())
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Faled to query database: ", err))
		return
	}

	var response RequestAllMatchesResponseT
	for _, match := range matches {
		response.MatchList = append(response.MatchList, &request_all_matches_response.MatchT{
			MatchNumber: match.MatchNumber,
			Round:       match.Round,
			CompLevel:   match.CompLevel,
			R1:          match.R1,
			R2:          match.R2,
			R3:          match.R3,
			B1:          match.B1,
			B2:          match.B2,
			B3:          match.B3,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

// TODO(phil): Can we turn this into a generic?
func parseRequestDataScouting(w http.ResponseWriter, buf []byte) (*RequestDataScouting, bool) {
	success := true
	defer func() {
		if r := recover(); r != nil {
			respondWithError(w, http.StatusBadRequest, fmt.Sprintf("Failed to parse SubmitDataScouting: %v", r))
			success = false
		}
	}()
	result := request_data_scouting.GetRootAsRequestDataScouting(buf, 0)
	return result, success
}

// Handles a RequestDataScouting request.
type requestDataScoutingHandler struct {
	db Database
}

func (handler requestDataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequestDataScouting(w, requestBytes)
	if !success {
		return
	}

	stats, err := handler.db.ReturnStats()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Faled to query database: ", err))
		return
	}

	var response RequestDataScoutingResponseT
	for _, stat := range stats {
		response.StatsList = append(response.StatsList, &request_data_scouting_response.StatsT{
			Team:            stat.TeamNumber,
			Match:           stat.MatchNumber,
			MissedShotsAuto: stat.ShotsMissedAuto,
			UpperGoalAuto:   stat.UpperGoalAuto,
			LowerGoalAuto:   stat.LowerGoalAuto,
			MissedShotsTele: stat.ShotsMissed,
			UpperGoalTele:   stat.UpperGoalShots,
			LowerGoalTele:   stat.LowerGoalShots,
			DefenseRating:   stat.PlayedDefense,
			Climbing:        stat.Climbing,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func HandleRequests(db Database, scoutingServer server.ScoutingServer) {
	scoutingServer.HandleFunc("/requests", unknown)
	scoutingServer.Handle("/requests/submit/data_scouting", submitDataScoutingHandler{db})
	scoutingServer.Handle("/requests/request/all_matches", requestAllMatchesHandler{db})
	scoutingServer.Handle("/requests/request/matches_for_team", requestMatchesForTeamHandler{db})
	scoutingServer.Handle("/requests/request/data_scouting", requestDataScoutingHandler{db})
}
