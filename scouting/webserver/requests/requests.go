package requests

import (
	"encoding/base64"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"
	"sort"
	"strconv"
	"strings"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2023_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2023_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_response"
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
type RequestAllDriverRankings = request_all_driver_rankings.RequestAllDriverRankings
type RequestAllDriverRankingsResponseT = request_all_driver_rankings_response.RequestAllDriverRankingsResponseT
type RequestAllNotes = request_all_notes.RequestAllNotes
type RequestAllNotesResponseT = request_all_notes_response.RequestAllNotesResponseT
type RequestDataScouting = request_data_scouting.RequestDataScouting
type RequestDataScoutingResponseT = request_data_scouting_response.RequestDataScoutingResponseT
type Request2023DataScouting = request_2023_data_scouting.Request2023DataScouting
type Request2023DataScoutingResponseT = request_2023_data_scouting_response.Request2023DataScoutingResponseT
type SubmitNotes = submit_notes.SubmitNotes
type SubmitNotesResponseT = submit_notes_response.SubmitNotesResponseT
type RequestNotesForTeam = request_notes_for_team.RequestNotesForTeam
type RequestNotesForTeamResponseT = request_notes_for_team_response.RequestNotesForTeamResponseT
type RequestShiftSchedule = request_shift_schedule.RequestShiftSchedule
type RequestShiftScheduleResponseT = request_shift_schedule_response.RequestShiftScheduleResponseT
type SubmitShiftSchedule = submit_shift_schedule.SubmitShiftSchedule
type SubmitShiftScheduleResponseT = submit_shift_schedule_response.SubmitShiftScheduleResponseT
type SubmitDriverRanking = submit_driver_ranking.SubmitDriverRanking
type SubmitDriverRankingResponseT = submit_driver_ranking_response.SubmitDriverRankingResponseT
type SubmitActions = submit_actions.SubmitActions
type SubmitActionsResponseT = submit_actions_response.SubmitActionsResponseT

// The interface we expect the database abstraction to conform to.
// We use an interface here because it makes unit testing easier.
type Database interface {
	AddToMatch(db.TeamMatch) error
	AddToShift(db.Shift) error
	AddToStats(db.Stats) error
	AddToStats2023(db.Stats2023) error
	ReturnMatches() ([]db.TeamMatch, error)
	ReturnAllNotes() ([]db.NotesData, error)
	ReturnAllDriverRankings() ([]db.DriverRankingData, error)
	ReturnAllShifts() ([]db.Shift, error)
	ReturnStats() ([]db.Stats, error)
	ReturnStats2023() ([]db.Stats2023, error)
	QueryAllShifts(int) ([]db.Shift, error)
	QueryStats(int) ([]db.Stats, error)
	QueryNotes(int32) ([]string, error)
	AddNotes(db.NotesData) error
	AddDriverRanking(db.DriverRankingData) error
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
//	req.Headers["Authorization"] = []string{"Basic <base64 encoded username:password>"}
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

func findIndexInList(list []string, comp_level string) (int, error) {
	for index, value := range list {
		if value == comp_level {
			return index, nil
		}
	}
	return -1, errors.New(fmt.Sprint("Failed to find comp level ", comp_level, " in list ", list))
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

	// Change structure of match objects in the database(1 per team) to
	// the old match structure(1 per match) that the webserver uses.
	type Key struct {
		MatchNumber int32
		SetNumber   int32
		CompLevel   string
	}

	assembledMatches := map[Key]request_all_matches_response.MatchT{}

	for _, match := range matches {
		key := Key{match.MatchNumber, match.SetNumber, match.CompLevel}
		entry, ok := assembledMatches[key]
		if !ok {
			entry = request_all_matches_response.MatchT{
				MatchNumber: match.MatchNumber,
				SetNumber:   match.SetNumber,
				CompLevel:   match.CompLevel,
			}
		}
		switch match.Alliance {
		case "R":
			switch match.AlliancePosition {
			case 1:
				entry.R1 = match.TeamNumber
			case 2:
				entry.R2 = match.TeamNumber
			case 3:
				entry.R3 = match.TeamNumber
			default:
				respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown red position ", strconv.Itoa(int(match.AlliancePosition)), " in match ", strconv.Itoa(int(match.MatchNumber))))
				return
			}
		case "B":
			switch match.AlliancePosition {
			case 1:
				entry.B1 = match.TeamNumber
			case 2:
				entry.B2 = match.TeamNumber
			case 3:
				entry.B3 = match.TeamNumber
			default:
				respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown blue position ", strconv.Itoa(int(match.AlliancePosition)), " in match ", strconv.Itoa(int(match.MatchNumber))))
				return
			}
		default:
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown alliance ", match.Alliance, " in match ", strconv.Itoa(int(match.AlliancePosition))))
			return
		}
		assembledMatches[key] = entry
	}

	var response RequestAllMatchesResponseT
	for _, match := range assembledMatches {
		copied_match := match
		response.MatchList = append(response.MatchList, &copied_match)
	}

	var MATCH_TYPE_ORDERING = []string{"qm", "ef", "qf", "sf", "f"}

	err = nil
	sort.Slice(response.MatchList, func(i, j int) bool {
		if err != nil {
			return false
		}
		a := response.MatchList[i]
		b := response.MatchList[j]

		aMatchTypeIndex, err2 := findIndexInList(MATCH_TYPE_ORDERING, a.CompLevel)
		if err2 != nil {
			err = errors.New(fmt.Sprint("Comp level ", a.CompLevel, " not found in sorting list ", MATCH_TYPE_ORDERING, " : ", err2))
			return false
		}
		bMatchTypeIndex, err2 := findIndexInList(MATCH_TYPE_ORDERING, b.CompLevel)
		if err2 != nil {
			err = errors.New(fmt.Sprint("Comp level ", b.CompLevel, " not found in sorting list ", MATCH_TYPE_ORDERING, " : ", err2))
			return false
		}

		if aMatchTypeIndex < bMatchTypeIndex {
			return true
		}
		if aMatchTypeIndex > bMatchTypeIndex {
			return false
		}

		// Then sort by match number. E.g. in semi finals, all match 1 rounds
		// are done first. Then come match 2 rounds. And then, if necessary,
		// the match 3 rounds.
		aMatchNumber := a.MatchNumber
		bMatchNumber := b.MatchNumber
		if aMatchNumber < bMatchNumber {
			return true
		}
		if aMatchNumber > bMatchNumber {
			return false
		}
		// Lastly, sort by set number. I.e. Semi Final 1 Match 1 happens first.
		// Then comes Semi Final 2 Match 1. Then comes Semi Final 1 Match 2. Then
		// Semi Final 2 Match 2.
		aSetNumber := a.SetNumber
		bSetNumber := b.SetNumber
		if aSetNumber < bSetNumber {
			return true
		}
		if aSetNumber > bSetNumber {
			return false
		}
		return true
	})

	if err != nil {
		// check if error happened during sorting and notify webpage if that
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint(err))
		return
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
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
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
		TeamNumber:   request.Team(),
		Notes:        string(request.Notes()),
		GoodDriving:  bool(request.GoodDriving()),
		BadDriving:   bool(request.BadDriving()),
		SketchyClimb: bool(request.SketchyClimb()),
		SolidClimb:   bool(request.SolidClimb()),
		GoodDefense:  bool(request.GoodDefense()),
		BadDefense:   bool(request.BadDefense()),
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

// Handles a Request2023DataScouting request.
type request2023DataScoutingHandler struct {
	db Database
}

func (handler request2023DataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "Request2023DataScouting", request_2023_data_scouting.GetRootAsRequest2023DataScouting)
	if !success {
		return
	}

	stats, err := handler.db.ReturnStats2023()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response Request2023DataScoutingResponseT
	for _, stat := range stats {
		response.StatsList = append(response.StatsList, &request_2023_data_scouting_response.Stats2023T{
			TeamNumber:       stat.TeamNumber,
			MatchNumber:      stat.MatchNumber,
			SetNumber:        stat.SetNumber,
			CompLevel:        stat.CompLevel,
			StartingQuadrant: stat.StartingQuadrant,
			LowCubesAuto:     stat.LowCubesAuto,
			MiddleCubesAuto:  stat.MiddleCubesAuto,
			HighCubesAuto:    stat.HighCubesAuto,
			CubesDroppedAuto: stat.CubesDroppedAuto,
			LowConesAuto:     stat.LowConesAuto,
			MiddleConesAuto:  stat.MiddleConesAuto,
			HighConesAuto:    stat.HighConesAuto,
			ConesDroppedAuto: stat.ConesDroppedAuto,
			LowCubes:         stat.LowCubes,
			MiddleCubes:      stat.MiddleCubes,
			HighCubes:        stat.HighCubes,
			CubesDropped:     stat.CubesDropped,
			LowCones:         stat.LowCones,
			MiddleCones:      stat.MiddleCones,
			HighCones:        stat.HighCones,
			ConesDropped:     stat.ConesDropped,
			AvgCycle:         stat.AvgCycle,
			CollectedBy:      stat.CollectedBy,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
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

	notes, err := handler.db.QueryNotes(request.Team())
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query notes: %v", err))
		return
	}

	var response RequestNotesForTeamResponseT
	for _, data := range notes {
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

type SubmitDriverRankingHandler struct {
	db Database
}

func (handler SubmitDriverRankingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitDriverRanking", submit_driver_ranking.GetRootAsSubmitDriverRanking)
	if !success {
		return
	}

	err = handler.db.AddDriverRanking(db.DriverRankingData{
		MatchNumber: request.MatchNumber(),
		Rank1:       request.Rank1(),
		Rank2:       request.Rank2(),
		Rank3:       request.Rank3(),
	})

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert driver ranking: %v", err))
		return
	}

	var response SubmitDriverRankingResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestAllNotesHandler struct {
	db Database
}

func (handler requestAllNotesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestAllNotes", request_all_notes.GetRootAsRequestAllNotes)
	if !success {
		return
	}

	notes, err := handler.db.ReturnAllNotes()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response RequestAllNotesResponseT
	for _, note := range notes {
		response.NoteList = append(response.NoteList, &request_all_notes_response.NoteT{
			Team:         note.TeamNumber,
			Notes:        note.Notes,
			GoodDriving:  note.GoodDriving,
			BadDriving:   note.BadDriving,
			SketchyClimb: note.SketchyClimb,
			SolidClimb:   note.SolidClimb,
			GoodDefense:  note.GoodDefense,
			BadDefense:   note.BadDefense,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestAllDriverRankingsHandler struct {
	db Database
}

func (handler requestAllDriverRankingsHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestAllDriverRankings", request_all_driver_rankings.GetRootAsRequestAllDriverRankings)
	if !success {
		return
	}

	rankings, err := handler.db.ReturnAllDriverRankings()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response RequestAllDriverRankingsResponseT
	for _, ranking := range rankings {
		response.DriverRankingList = append(response.DriverRankingList, &request_all_driver_rankings_response.RankingT{
			MatchNumber: ranking.MatchNumber,
			Rank1:       ranking.Rank1,
			Rank2:       ranking.Rank2,
			Rank3:       ranking.Rank3,
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
	scoutingServer.Handle("/requests/request/all_notes", requestAllNotesHandler{db})
	scoutingServer.Handle("/requests/request/all_driver_rankings", requestAllDriverRankingsHandler{db})
	scoutingServer.Handle("/requests/request/data_scouting", requestDataScoutingHandler{db})
	scoutingServer.Handle("/requests/request/2023_data_scouting", request2023DataScoutingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_notes", submitNoteScoutingHandler{db})
	scoutingServer.Handle("/requests/request/notes_for_team", requestNotesForTeamHandler{db})
	scoutingServer.Handle("/requests/submit/shift_schedule", submitShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/request/shift_schedule", requestShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/submit/submit_driver_ranking", SubmitDriverRankingHandler{db})
}
