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
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

type RequestAllMatches = request_all_matches.RequestAllMatches
type RequestAllMatchesResponseT = request_all_matches_response.RequestAllMatchesResponseT
type RequestAllDriverRankings = request_all_driver_rankings.RequestAllDriverRankings
type RequestAllDriverRankingsResponseT = request_all_driver_rankings_response.RequestAllDriverRankingsResponseT
type RequestAllNotes = request_all_notes.RequestAllNotes
type RequestAllNotesResponseT = request_all_notes_response.RequestAllNotesResponseT
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
type Action = submit_actions.Action
type SubmitActionsResponseT = submit_actions_response.SubmitActionsResponseT

// The interface we expect the database abstraction to conform to.
// We use an interface here because it makes unit testing easier.
type Database interface {
	AddToMatch(db.TeamMatch) error
	AddToShift(db.Shift) error
	AddToStats2023(db.Stats2023) error
	ReturnMatches() ([]db.TeamMatch, error)
	ReturnAllNotes() ([]db.NotesData, error)
	ReturnAllDriverRankings() ([]db.DriverRankingData, error)
	ReturnAllShifts() ([]db.Shift, error)
	ReturnStats2023() ([]db.Stats2023, error)
	ReturnStats2023ForTeam(teamNumber string, matchNumber int32, setNumber int32, compLevel string, preScouting bool) ([]db.Stats2023, error)
	QueryAllShifts(int) ([]db.Shift, error)
	QueryNotes(int32) ([]string, error)
	AddNotes(db.NotesData) error
	AddDriverRanking(db.DriverRankingData) error
	AddAction(db.Action) error
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

// Handles a RequestAllMaches request.
type requestAllMatchesHandler struct {
	db Database
}

// Change structure of match objects in the database(1 per team) to
// the old match structure(1 per match) that the webserver uses.
// We use the information in this struct to identify which match object
// corresponds to which old match structure object.
type MatchAssemblyKey struct {
	MatchNumber int32
	SetNumber   int32
	CompLevel   string
}

func findIndexInList(list []string, comp_level string) (int, error) {
	for index, value := range list {
		if value == comp_level {
			return index, nil
		}
	}
	return -1, errors.New(fmt.Sprint("Failed to find comp level ", comp_level, " in list ", list))
}

func (handler requestAllMatchesHandler) teamHasBeenDataScouted(key MatchAssemblyKey, teamNumber string) (bool, error) {
	stats, err := handler.db.ReturnStats2023ForTeam(
		teamNumber, key.MatchNumber, key.SetNumber, key.CompLevel, false)
	if err != nil {
		return false, err
	}
	return (len(stats) > 0), nil
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

	assembledMatches := map[MatchAssemblyKey]request_all_matches_response.MatchT{}

	for _, match := range matches {
		key := MatchAssemblyKey{match.MatchNumber, match.SetNumber, match.CompLevel}

		// Retrieve the converted match structure we have assembled so
		// far. If we haven't started assembling one yet, then start a
		// new one.
		entry, ok := assembledMatches[key]
		if !ok {
			entry = request_all_matches_response.MatchT{
				MatchNumber: match.MatchNumber,
				SetNumber:   match.SetNumber,
				CompLevel:   match.CompLevel,
				DataScouted: &request_all_matches_response.ScoutedLevelT{},
			}
		}

		var team *string
		var dataScoutedTeam *bool

		// Fill in the field for the match that we have in in the
		// database. In the database, each match row only has 1 team
		// number.
		switch match.Alliance {
		case "R":
			switch match.AlliancePosition {
			case 1:
				team = &entry.R1
				dataScoutedTeam = &entry.DataScouted.R1
			case 2:
				team = &entry.R2
				dataScoutedTeam = &entry.DataScouted.R2
			case 3:
				team = &entry.R3
				dataScoutedTeam = &entry.DataScouted.R3
			default:
				respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown red position ", strconv.Itoa(int(match.AlliancePosition)), " in match ", strconv.Itoa(int(match.MatchNumber))))
				return
			}
		case "B":
			switch match.AlliancePosition {
			case 1:
				team = &entry.B1
				dataScoutedTeam = &entry.DataScouted.B1
			case 2:
				team = &entry.B2
				dataScoutedTeam = &entry.DataScouted.B2
			case 3:
				team = &entry.B3
				dataScoutedTeam = &entry.DataScouted.B3
			default:
				respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown blue position ", strconv.Itoa(int(match.AlliancePosition)), " in match ", strconv.Itoa(int(match.MatchNumber))))
				return
			}
		default:
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown alliance ", match.Alliance, " in match ", strconv.Itoa(int(match.AlliancePosition))))
			return
		}

		*team = match.TeamNumber

		// Figure out if this team has been data scouted already.
		*dataScoutedTeam, err = handler.teamHasBeenDataScouted(key, match.TeamNumber)
		if err != nil {
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint(
				"Failed to determine data scouting status for team ",
				strconv.Itoa(int(match.AlliancePosition)),
				" in match ",
				strconv.Itoa(int(match.MatchNumber)),
				err))
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
		TeamNumber:     request.Team(),
		Notes:          string(request.Notes()),
		GoodDriving:    bool(request.GoodDriving()),
		BadDriving:     bool(request.BadDriving()),
		SolidPlacing:   bool(request.SolidPlacing()),
		SketchyPlacing: bool(request.SketchyPlacing()),
		GoodDefense:    bool(request.GoodDefense()),
		BadDefense:     bool(request.BadDefense()),
		EasilyDefended: bool(request.EasilyDefended()),
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

func ConvertActionsToStat(submitActions *submit_actions.SubmitActions) (db.Stats2023, error) {
	overall_time := int64(0)
	cycles := int64(0)
	picked_up := false
	lastPlacedTime := int64(0)
	stat := db.Stats2023{
		PreScouting: submitActions.PreScouting(),
		TeamNumber:  string(submitActions.TeamNumber()), MatchNumber: submitActions.MatchNumber(), SetNumber: submitActions.SetNumber(), CompLevel: string(submitActions.CompLevel()),
		StartingQuadrant: 0, LowCubesAuto: 0, MiddleCubesAuto: 0, HighCubesAuto: 0, CubesDroppedAuto: 0,
		LowConesAuto: 0, MiddleConesAuto: 0, HighConesAuto: 0, ConesDroppedAuto: 0, LowCubes: 0, MiddleCubes: 0, HighCubes: 0,
		CubesDropped: 0, LowCones: 0, MiddleCones: 0, HighCones: 0, ConesDropped: 0, SuperchargedPieces: 0, AvgCycle: 0, CollectedBy: "",
	}
	// Loop over all actions.
	for i := 0; i < submitActions.ActionsListLength(); i++ {
		var action submit_actions.Action
		if !submitActions.ActionsList(&action, i) {
			return db.Stats2023{}, errors.New(fmt.Sprintf("Failed to parse submit_actions.Action"))
		}
		actionTable := new(flatbuffers.Table)
		action_type := action.ActionTakenType()
		if !action.ActionTaken(actionTable) {
			return db.Stats2023{}, errors.New(fmt.Sprint("Failed to parse sub-action or sub-action was missing"))
		}
		if action_type == submit_actions.ActionTypeStartMatchAction {
			var startMatchAction submit_actions.StartMatchAction
			startMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.StartingQuadrant = startMatchAction.Position()
		} else if action_type == submit_actions.ActionTypeMobilityAction {
			var mobilityAction submit_actions.MobilityAction
			mobilityAction.Init(actionTable.Bytes, actionTable.Pos)
			if mobilityAction.Mobility() {
				stat.Mobility = true
			}

		} else if action_type == submit_actions.ActionTypeAutoBalanceAction {
			var autoBalanceAction submit_actions.AutoBalanceAction
			autoBalanceAction.Init(actionTable.Bytes, actionTable.Pos)
			if autoBalanceAction.Docked() {
				stat.DockedAuto = true
			}
			if autoBalanceAction.Engaged() {
				stat.EngagedAuto = true
			}
			if autoBalanceAction.BalanceAttempt() {
				stat.BalanceAttemptAuto = true
			}
		} else if action_type == submit_actions.ActionTypePickupObjectAction {
			var pick_up_action submit_actions.PickupObjectAction
			pick_up_action.Init(actionTable.Bytes, actionTable.Pos)
			if picked_up == true {
				object := pick_up_action.ObjectType().String()
				auto := pick_up_action.Auto()
				if object == "kCube" && auto == false {
					stat.CubesDropped += 1
				} else if object == "kCube" && auto == true {
					stat.CubesDroppedAuto += 1
				} else if object == "kCone" && auto == false {
					stat.ConesDropped += 1
				} else if object == "kCube" && auto == true {
					stat.ConesDroppedAuto += 1
				}
			} else {
				picked_up = true
			}
		} else if action_type == submit_actions.ActionTypePlaceObjectAction {
			var place_action submit_actions.PlaceObjectAction
			place_action.Init(actionTable.Bytes, actionTable.Pos)
			if !picked_up {
				return db.Stats2023{}, errors.New(fmt.Sprintf("Got PlaceObjectAction without corresponding PickupObjectAction"))
			}
			object := place_action.ObjectType()
			level := place_action.ScoreLevel()
			auto := place_action.Auto()
			if object == 0 && level == 0 && auto == true {
				stat.LowCubesAuto += 1
			} else if object == 0 && level == 0 && auto == false {
				stat.LowCubes += 1
			} else if object == 0 && level == 1 && auto == true {
				stat.MiddleCubesAuto += 1
			} else if object == 0 && level == 1 && auto == false {
				stat.MiddleCubes += 1
			} else if object == 0 && level == 2 && auto == true {
				stat.HighCubesAuto += 1
			} else if object == 0 && level == 2 && auto == false {
				stat.HighCubes += 1
			} else if object == 1 && level == 0 && auto == true {
				stat.LowConesAuto += 1
			} else if object == 1 && level == 0 && auto == false {
				stat.LowCones += 1
			} else if object == 1 && level == 1 && auto == true {
				stat.MiddleConesAuto += 1
			} else if object == 1 && level == 1 && auto == false {
				stat.MiddleCones += 1
			} else if object == 1 && level == 2 && auto == true {
				stat.HighConesAuto += 1
			} else if object == 1 && level == 2 && auto == false {
				stat.HighCones += 1
			} else if level == 3 {
				stat.SuperchargedPieces += 1
			} else {
				return db.Stats2023{}, errors.New(fmt.Sprintf("Got unknown ObjectType/ScoreLevel/Auto combination"))
			}
			picked_up = false
			if lastPlacedTime != int64(0) {
				// If this is not the first time we place,
				// start counting cycle time. We define cycle
				// time as the time between placements.
				overall_time += int64(action.Timestamp()) - lastPlacedTime
				cycles += 1
			}
			lastPlacedTime = int64(action.Timestamp())
		} else if action_type == submit_actions.ActionTypeEndMatchAction {
			var endMatchAction submit_actions.EndMatchAction
			endMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			if endMatchAction.Docked() {
				stat.Docked = true
			}
			if endMatchAction.Engaged() {
				stat.Engaged = true
			}
			if endMatchAction.BalanceAttempt() {
				stat.BalanceAttempt = true
			}
		}
	}
	if cycles != 0 {
		stat.AvgCycle = overall_time / cycles
	} else {
		stat.AvgCycle = 0
	}
	return stat, nil
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
			TeamNumber:         stat.TeamNumber,
			MatchNumber:        stat.MatchNumber,
			SetNumber:          stat.SetNumber,
			CompLevel:          stat.CompLevel,
			StartingQuadrant:   stat.StartingQuadrant,
			LowCubesAuto:       stat.LowCubesAuto,
			MiddleCubesAuto:    stat.MiddleCubesAuto,
			HighCubesAuto:      stat.HighCubesAuto,
			CubesDroppedAuto:   stat.CubesDroppedAuto,
			LowConesAuto:       stat.LowConesAuto,
			MiddleConesAuto:    stat.MiddleConesAuto,
			HighConesAuto:      stat.HighConesAuto,
			ConesDroppedAuto:   stat.ConesDroppedAuto,
			LowCubes:           stat.LowCubes,
			MiddleCubes:        stat.MiddleCubes,
			HighCubes:          stat.HighCubes,
			CubesDropped:       stat.CubesDropped,
			LowCones:           stat.LowCones,
			MiddleCones:        stat.MiddleCones,
			HighCones:          stat.HighCones,
			ConesDropped:       stat.ConesDropped,
			SuperchargedPieces: stat.SuperchargedPieces,
			AvgCycle:           stat.AvgCycle,
			Mobility:           stat.Mobility,
			DockedAuto:         stat.DockedAuto,
			EngagedAuto:        stat.EngagedAuto,
			BalanceAttemptAuto: stat.BalanceAttemptAuto,
			Docked:             stat.Docked,
			Engaged:            stat.Engaged,
			BalanceAttempt:     stat.BalanceAttempt,
			CollectedBy:        stat.CollectedBy,
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
			Team:           note.TeamNumber,
			Notes:          note.Notes,
			GoodDriving:    note.GoodDriving,
			BadDriving:     note.BadDriving,
			SolidPlacing:   note.SolidPlacing,
			SketchyPlacing: note.SketchyPlacing,
			GoodDefense:    note.GoodDefense,
			BadDefense:     note.BadDefense,
			EasilyDefended: note.EasilyDefended,
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

type submitActionsHandler struct {
	db Database
}

func (handler submitActionsHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	// Get the username of the person submitting the data.
	username := parseUsername(req)

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitActions", submit_actions.GetRootAsSubmitActions)
	if !success {
		return
	}

	log.Println("Got actions for match", request.MatchNumber(), "team", request.TeamNumber(), "from", username)

	for i := 0; i < request.ActionsListLength(); i++ {

		var action Action
		request.ActionsList(&action, i)

		dbAction := db.Action{
			PreScouting: request.PreScouting(),
			TeamNumber:  string(request.TeamNumber()),
			MatchNumber: request.MatchNumber(),
			SetNumber:   request.SetNumber(),
			CompLevel:   string(request.CompLevel()),
			//TODO: Serialize CompletedAction
			CompletedAction: []byte{},
			TimeStamp:       action.Timestamp(),
			CollectedBy:     username,
		}

		// Do some error checking.
		if action.Timestamp() < 0 {
			respondWithError(w, http.StatusBadRequest, fmt.Sprint(
				"Invalid timestamp field value of ", action.Timestamp()))
			return
		}

		err = handler.db.AddAction(dbAction)
		if err != nil {
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to add action to database: ", err))
			return
		}
	}

	stats, err := ConvertActionsToStat(request)
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to convert actions to stats: ", err))
		return
	}

	stats.CollectedBy = username

	err = handler.db.AddToStats2023(stats)
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to submit stats: ", stats, ": ", err))
		return
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&SubmitActionsResponseT{}).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func HandleRequests(db Database, scoutingServer server.ScoutingServer) {
	scoutingServer.HandleFunc("/requests", unknown)
	scoutingServer.Handle("/requests/request/all_matches", requestAllMatchesHandler{db})
	scoutingServer.Handle("/requests/request/all_notes", requestAllNotesHandler{db})
	scoutingServer.Handle("/requests/request/all_driver_rankings", requestAllDriverRankingsHandler{db})
	scoutingServer.Handle("/requests/request/2023_data_scouting", request2023DataScoutingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_notes", submitNoteScoutingHandler{db})
	scoutingServer.Handle("/requests/request/notes_for_team", requestNotesForTeamHandler{db})
	scoutingServer.Handle("/requests/submit/shift_schedule", submitShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/request/shift_schedule", requestShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/submit/submit_driver_ranking", SubmitDriverRankingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_actions", submitActionsHandler{db})
}
