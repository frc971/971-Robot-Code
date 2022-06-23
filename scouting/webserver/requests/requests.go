package requests

import (
	"encoding/base64"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"
	"strconv"
	"strings"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

type SubmitDataScouting = submit_data_scouting.SubmitDataScouting
type SubmitDataScoutingResponseT = submit_data_scouting_response.SubmitDataScoutingResponseT
type RequestAllMatches = request_all_matches.RequestAllMatches
type RequestAllMatchesResponseT = request_all_matches_response.RequestAllMatchesResponseT
type RequestMatchesForTeam = request_matches_for_team.RequestMatchesForTeam
type RequestMatchesForTeamResponseT = request_matches_for_team_response.RequestMatchesForTeamResponseT
type RequestDataScouting = request_data_scouting.RequestDataScouting
type RequestDataScoutingResponseT = request_data_scouting_response.RequestDataScoutingResponseT
type RefreshMatchList = refresh_match_list.RefreshMatchList
type RefreshMatchListResponseT = refresh_match_list_response.RefreshMatchListResponseT
type SubmitNotes = submit_notes.SubmitNotes
type SubmitNotesResponseT = submit_notes_response.SubmitNotesResponseT
type RequestNotesForTeam = request_notes_for_team.RequestNotesForTeam
type RequestNotesForTeamResponseT = request_notes_for_team_response.RequestNotesForTeamResponseT
type RequestShiftSchedule = request_shift_schedule.RequestShiftSchedule
type RequestShiftScheduleResponseT = request_shift_schedule_response.RequestShiftScheduleResponseT
type SubmitShiftSchedule = submit_shift_schedule.SubmitShiftSchedule
type SubmitShiftScheduleResponseT = submit_shift_schedule_response.SubmitShiftScheduleResponseT

// The interface we expect the database abstraction to conform to.
// We use an interface here because it makes unit testing easier.
type Database interface {
	AddToMatch(db.Match) error
	AddToShift(db.Shift) error
	AddToStats(db.Stats) error
	ReturnMatches() ([]db.Match, error)
	ReturnAllShifts() ([]db.Shift, error)
	ReturnStats() ([]db.Stats, error)
	QueryMatches(int32) ([]db.Match, error)
	QueryAllShifts(int) ([]db.Shift, error)
	QueryStats(int) ([]db.Stats, error)
	QueryNotes(int32) (db.NotesData, error)
	AddNotes(db.NotesData) error
}

type ScrapeMatchList func(int32, string) ([]scraping.Match, error)

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

func parseRequest[T interface{}](w http.ResponseWriter, buf []byte, requestName string, parser func([]byte, flatbuffers.UOffsetT) *T) (*T, bool) {
	success := true
	defer func() {
		if r := recover(); r != nil {
			respondWithError(w, http.StatusBadRequest, fmt.Sprintf("Failed to parse %s: %v", requestName, r))
			success = false
		}
	}()
	result := parser(buf, 0)
	return result, success
}

// Parses the authorization information that the browser inserts into the
// headers.  The authorization follows this format:
//
//  req.Headers["Authorization"] = []string{"Basic <base64 encoded username:password>"}
func parseUsername(req *http.Request) string {
	auth, ok := req.Header["Authorization"]
	if !ok {
		return "unknown"
	}

	parts := strings.Split(auth[0], " ")
	if !(len(parts) == 2 && parts[0] == "Basic") {
		return "unknown"
	}

	info, err := base64.StdEncoding.DecodeString(parts[1])
	if err != nil {
		log.Println("ERROR: Failed to parse Basic authentication.")
		return "unknown"
	}

	loginParts := strings.Split(string(info), ":")
	if len(loginParts) != 2 {
		return "unknown"
	}
	return loginParts[0]
}

// Handles a SubmitDataScouting request.
type submitDataScoutingHandler struct {
	db Database
}

func (handler submitDataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	// Get the username of the person submitting the data.
	username := parseUsername(req)

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest[SubmitDataScouting](w, requestBytes, "SubmitDataScouting", submit_data_scouting.GetRootAsSubmitDataScouting)
	if !success {
		return
	}

	log.Println("Got data scouting data for match", request.Match(), "team", request.Team(), "from", username)

	stats := db.Stats{
		TeamNumber:       request.Team(),
		MatchNumber:      request.Match(),
		SetNumber:        request.SetNumber(),
		CompLevel:        string(request.CompLevel()),
		StartingQuadrant: request.StartingQuadrant(),
		AutoBallPickedUp: [5]bool{
			request.AutoBall1(), request.AutoBall2(), request.AutoBall3(),
			request.AutoBall4(), request.AutoBall5(),
		},
		ShotsMissedAuto:      request.MissedShotsAuto(),
		UpperGoalAuto:        request.UpperGoalAuto(),
		LowerGoalAuto:        request.LowerGoalAuto(),
		ShotsMissed:          request.MissedShotsTele(),
		UpperGoalShots:       request.UpperGoalTele(),
		LowerGoalShots:       request.LowerGoalTele(),
		PlayedDefense:        request.DefenseRating(),
		DefenseReceivedScore: request.DefenseReceivedRating(),
		Climbing:             int32(request.ClimbLevel()),
		CollectedBy:          username,
		Comment:              string(request.Comment()),
	}

	// Do some error checking.
	if stats.StartingQuadrant < 1 || stats.StartingQuadrant > 4 {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint(
			"Invalid starting_quadrant field value of ", stats.StartingQuadrant))
		return
	}

	err = handler.db.AddToStats(stats)
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to submit datascouting data: ", err))
		return
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&SubmitDataScoutingResponseT{}).Pack(builder))
	w.Write(builder.FinishedBytes())
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

	_, success := parseRequest(w, requestBytes, "RequestAllMatches", request_all_matches.GetRootAsRequestAllMatches)
	if !success {
		return
	}

	matches, err := handler.db.ReturnMatches()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response RequestAllMatchesResponseT
	for _, match := range matches {
		response.MatchList = append(response.MatchList, &request_all_matches_response.MatchT{
			MatchNumber: match.MatchNumber,
			SetNumber:   match.SetNumber,
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

	request, success := parseRequest(w, requestBytes, "RequestMatchesForTeam", request_matches_for_team.GetRootAsRequestMatchesForTeam)
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
			SetNumber:   match.SetNumber,
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

	_, success := parseRequest(w, requestBytes, "RequestDataScouting", request_data_scouting.GetRootAsRequestDataScouting)
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
			Team:                  stat.TeamNumber,
			Match:                 stat.MatchNumber,
			SetNumber:             stat.SetNumber,
			CompLevel:             stat.CompLevel,
			StartingQuadrant:      stat.StartingQuadrant,
			AutoBall1:             stat.AutoBallPickedUp[0],
			AutoBall2:             stat.AutoBallPickedUp[1],
			AutoBall3:             stat.AutoBallPickedUp[2],
			AutoBall4:             stat.AutoBallPickedUp[3],
			AutoBall5:             stat.AutoBallPickedUp[4],
			MissedShotsAuto:       stat.ShotsMissedAuto,
			UpperGoalAuto:         stat.UpperGoalAuto,
			LowerGoalAuto:         stat.LowerGoalAuto,
			MissedShotsTele:       stat.ShotsMissed,
			UpperGoalTele:         stat.UpperGoalShots,
			LowerGoalTele:         stat.LowerGoalShots,
			DefenseRating:         stat.PlayedDefense,
			DefenseReceivedRating: stat.DefenseReceivedScore,
			ClimbLevel:            request_data_scouting_response.ClimbLevel(stat.Climbing),
			CollectedBy:           stat.CollectedBy,
			Comment:               stat.Comment,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func parseTeamKey(teamKey string) (int, error) {
	// TBA prefixes teams with "frc". Not sure why. Get rid of that.
	teamKey = strings.TrimPrefix(teamKey, "frc")
	return strconv.Atoi(teamKey)
}

// Parses the alliance data from the specified match and returns the three red
// teams and the three blue teams.
func parseTeamKeys(match *scraping.Match) ([3]int32, [3]int32, error) {
	redKeys := match.Alliances.Red.TeamKeys
	blueKeys := match.Alliances.Blue.TeamKeys

	if len(redKeys) != 3 || len(blueKeys) != 3 {
		return [3]int32{}, [3]int32{}, errors.New(fmt.Sprintf(
			"Found %d red teams and %d blue teams.", len(redKeys), len(blueKeys)))
	}

	var red [3]int32
	for i, key := range redKeys {
		team, err := parseTeamKey(key)
		if err != nil {
			return [3]int32{}, [3]int32{}, errors.New(fmt.Sprintf(
				"Failed to parse red %d team '%s' as integer: %v", i+1, key, err))
		}
		red[i] = int32(team)
	}
	var blue [3]int32
	for i, key := range blueKeys {
		team, err := parseTeamKey(key)
		if err != nil {
			return [3]int32{}, [3]int32{}, errors.New(fmt.Sprintf(
				"Failed to parse blue %d team '%s' as integer: %v", i+1, key, err))
		}
		blue[i] = int32(team)
	}
	return red, blue, nil
}

type refreshMatchListHandler struct {
	db     Database
	scrape ScrapeMatchList
}

func (handler refreshMatchListHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RefreshMatchList", refresh_match_list.GetRootAsRefreshMatchList)
	if !success {
		return
	}

	matches, err := handler.scrape(request.Year(), string(request.EventCode()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Faled to scrape match list: ", err))
		return
	}

	for _, match := range matches {
		// Make sure the data is valid.
		red, blue, err := parseTeamKeys(&match)
		if err != nil {
			respondWithError(w, http.StatusInternalServerError, fmt.Sprintf(
				"TheBlueAlliance data for match %d is malformed: %v", match.MatchNumber, err))
			return
		}
		// Add the match to the database.
		err = handler.db.AddToMatch(db.Match{
			MatchNumber: int32(match.MatchNumber),
			SetNumber:   int32(match.SetNumber),
			CompLevel:   match.CompLevel,
			R1:          red[0],
			R2:          red[1],
			R3:          red[2],
			B1:          blue[0],
			B2:          blue[1],
			B3:          blue[2],
		})
		if err != nil {
			respondWithError(w, http.StatusInternalServerError, fmt.Sprintf(
				"Failed to add match %d to the database: %v", match.MatchNumber, err))
			return
		}
	}

	var response RefreshMatchListResponseT
	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submitNoteScoutingHandler struct {
	db Database
}

func (handler submitNoteScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitNotes", submit_notes.GetRootAsSubmitNotes)
	if !success {
		return
	}

	err = handler.db.AddNotes(db.NotesData{
		TeamNumber: request.Team(),
		Notes:      []string{string(request.Notes())},
	})
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert notes: %v", err))
		return
	}

	var response SubmitNotesResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestNotesForTeamHandler struct {
	db Database
}

func (handler requestNotesForTeamHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestNotesForTeam", request_notes_for_team.GetRootAsRequestNotesForTeam)
	if !success {
		return
	}

	notesData, err := handler.db.QueryNotes(request.Team())
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query notes: %v", err))
		return
	}

	var response RequestNotesForTeamResponseT
	for _, data := range notesData.Notes {
		response.Notes = append(response.Notes, &request_notes_for_team_response.NoteT{data})
	}

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestShiftScheduleHandler struct {
	db Database
}

func (handler requestShiftScheduleHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestShiftSchedule", request_shift_schedule.GetRootAsRequestShiftSchedule)
	if !success {
		return
	}

	shiftData, err := handler.db.ReturnAllShifts()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query shift schedule: %v", err))
		return
	}

	var response RequestShiftScheduleResponseT
	for _, shifts := range shiftData {
		response.ShiftSchedule = append(response.ShiftSchedule, &request_shift_schedule_response.MatchAssignmentT{
			MatchNumber: shifts.MatchNumber,
			R1scouter:   shifts.R1scouter,
			R2scouter:   shifts.R2scouter,
			R3scouter:   shifts.R3scouter,
			B1scouter:   shifts.B1scouter,
			B2scouter:   shifts.B2scouter,
			B3scouter:   shifts.B3scouter,
		})
	}

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submitShiftScheduleHandler struct {
	db Database
}

func (handler submitShiftScheduleHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	// Get the username of the person submitting the data.
	username := parseUsername(req)

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest[SubmitShiftSchedule](w, requestBytes, "SubmitShiftSchedule", submit_shift_schedule.GetRootAsSubmitShiftSchedule)
	if !success {
		return
	}

	log.Println("Got shift schedule from", username)
	shift_schedule_length := request.ShiftScheduleLength()
	for i := 0; i < shift_schedule_length; i++ {
		var match_assignment submit_shift_schedule.MatchAssignment
		request.ShiftSchedule(&match_assignment, i)
		current_shift := db.Shift{
			MatchNumber: match_assignment.MatchNumber(),
			R1scouter:   string(match_assignment.R1scouter()),
			R2scouter:   string(match_assignment.R2scouter()),
			R3scouter:   string(match_assignment.R3scouter()),
			B1scouter:   string(match_assignment.B1scouter()),
			B2scouter:   string(match_assignment.B2scouter()),
			B3scouter:   string(match_assignment.B3scouter()),
		}
		err = handler.db.AddToShift(current_shift)
		if err != nil {
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to submit shift schedule: ", err))
			return
		}
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&SubmitShiftScheduleResponseT{}).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func HandleRequests(db Database, scrape ScrapeMatchList, scoutingServer server.ScoutingServer) {
	scoutingServer.HandleFunc("/requests", unknown)
	scoutingServer.Handle("/requests/submit/data_scouting", submitDataScoutingHandler{db})
	scoutingServer.Handle("/requests/request/all_matches", requestAllMatchesHandler{db})
	scoutingServer.Handle("/requests/request/matches_for_team", requestMatchesForTeamHandler{db})
	scoutingServer.Handle("/requests/request/data_scouting", requestDataScoutingHandler{db})
	scoutingServer.Handle("/requests/refresh_match_list", refreshMatchListHandler{db, scrape})
	scoutingServer.Handle("/requests/submit/submit_notes", submitNoteScoutingHandler{db})
	scoutingServer.Handle("/requests/request/notes_for_team", requestNotesForTeamHandler{db})
	scoutingServer.Handle("/requests/submit/shift_schedule", submitShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/request/shift_schedule", requestShiftScheduleHandler{db})
}
