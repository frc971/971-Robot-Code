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
	"time"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_2024_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_2024_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_2025_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_2025_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_notes_2025"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_notes_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2024_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2024_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2025_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2025_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_2025"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_pit_images"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_pit_images_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_averaged_driver_rankings_2025"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_averaged_driver_rankings_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_averaged_human_rankings_2025"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_averaged_human_rankings_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_current_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_current_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_2025_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_2025_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_pit_images"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_pit_images_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_2024_actions"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_2024_actions_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_2025_actions"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_2025_actions_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_2025"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_human_ranking_2025"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_human_ranking_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_2025"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_2025_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_pit_image"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_pit_image_response"
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
type RequestAllNotes2025 = request_all_notes_2025.RequestAllNotes2025
type RequestAllNotes2025ResponseT = request_all_notes_2025_response.RequestAllNotes2025ResponseT
type Request2024DataScouting = request_2024_data_scouting.Request2024DataScouting
type Request2024DataScoutingResponseT = request_2024_data_scouting_response.Request2024DataScoutingResponseT
type Request2025DataScouting = request_2025_data_scouting.Request2025DataScouting
type Request2025DataScoutingResponseT = request_2025_data_scouting_response.Request2025DataScoutingResponseT
type SubmitNotes = submit_notes.SubmitNotes
type SubmitNotesResponseT = submit_notes_response.SubmitNotesResponseT
type SubmitNotes2025 = submit_notes_2025.SubmitNotes2025
type SubmitNotes2025ResponseT = submit_notes_2025_response.SubmitNotes2025ResponseT
type SubmitPitImage = submit_pit_image.SubmitPitImage
type SubmitPitImageResponseT = submit_pit_image_response.SubmitPitImageResponseT
type RequestPitImages = request_pit_images.RequestPitImages
type RequestPitImagesResponseT = request_pit_images_response.RequestPitImagesResponseT
type RequestAllPitImages = request_all_pit_images.RequestAllPitImages
type RequestAllPitImagesResponseT = request_all_pit_images_response.RequestAllPitImagesResponseT
type RequestCurrentScouting = request_current_scouting.RequestCurrentScouting
type RequestCurrentScoutingResponseT = request_current_scouting_response.RequestCurrentScoutingResponseT
type RequestNotesForTeam = request_notes_for_team.RequestNotesForTeam
type RequestNotesForTeamResponseT = request_notes_for_team_response.RequestNotesForTeamResponseT
type RequestNotes2025ForTeam = request_notes_2025_for_team.RequestNotes2025ForTeam
type RequestNotes2025ForTeamResponseT = request_notes_2025_for_team_response.RequestNotes2025ForTeamResponseT
type RequestShiftSchedule = request_shift_schedule.RequestShiftSchedule
type RequestShiftScheduleResponseT = request_shift_schedule_response.RequestShiftScheduleResponseT
type RequestAveragedDriverRankings2025 = request_averaged_driver_rankings_2025.RequestAveragedDriverRankings2025
type RequestAveragedDriverRankings2025ResponseT = request_averaged_driver_rankings_2025_response.RequestAveragedDriverRankings2025ResponseT
type RequestAveragedHumanRankings2025 = request_averaged_human_rankings_2025.RequestAveragedHumanRankings2025
type RequestAveragedHumanRankings2025ResponseT = request_averaged_human_rankings_2025_response.RequestAveragedHumanRankings2025ResponseT
type SubmitShiftSchedule = submit_shift_schedule.SubmitShiftSchedule
type SubmitShiftScheduleResponseT = submit_shift_schedule_response.SubmitShiftScheduleResponseT
type SubmitDriverRanking = submit_driver_ranking.SubmitDriverRanking
type SubmitDriverRankingResponseT = submit_driver_ranking_response.SubmitDriverRankingResponseT
type Action2024 = submit_2024_actions.Action
type Action2025 = submit_2025_actions.Action
type Submit2024Actions = submit_2024_actions.Submit2024Actions
type Submit2024ActionsResponseT = submit_2024_actions_response.Submit2024ActionsResponseT
type Submit2025Actions = submit_2025_actions.Submit2025Actions
type Submit2025ActionsResponseT = submit_2025_actions_response.Submit2025ActionsResponseT
type SubmitDriverRanking2025 = submit_driver_ranking_2025.SubmitDriverRanking2025
type SubmitDriverRanking2025ResponseT = submit_driver_ranking_2025_response.SubmitDriverRanking2025ResponseT
type SubmitHumanRanking2025 = submit_human_ranking_2025.SubmitHumanRanking2025
type SubmitHumanRanking2025ResponseT = submit_human_ranking_2025_response.SubmitHumanRanking2025ResponseT
type Delete2024DataScouting = delete_2024_data_scouting.Delete2024DataScouting
type Delete2024DataScoutingResponseT = delete_2024_data_scouting_response.Delete2024DataScoutingResponseT
type Delete2025DataScouting = delete_2025_data_scouting.Delete2025DataScouting
type Delete2025DataScoutingResponseT = delete_2025_data_scouting_response.Delete2025DataScoutingResponseT
type DeleteNotes2025 = delete_notes_2025.DeleteNotes2025
type DeleteNotes2025ResponseT = delete_notes_2025_response.DeleteNotes2025ResponseT

// The interface we expect the database abstraction to conform to.
// We use an interface here because it makes unit testing easier.
type Database interface {
	AddToMatch(db.TeamMatch) error
	AddToMatch2025(db.TeamMatch2025) error
	AddToShift(db.Shift) error
	AddToStats2024(db.Stats2024) error
	AddToStats2025(db.Stats2025) error
	ReturnMatches() ([]db.TeamMatch, error)
	ReturnMatches2025() ([]db.TeamMatch2025, error)
	ReturnAllNotes() ([]db.NotesData, error)
	ReturnAllNotes2025(string) ([]db.NotesData2025, error)
	ReturnAllDriverRankings() ([]db.DriverRankingData, error)
	ReturnAllShifts() ([]db.Shift, error)
	ReturnStats2024() ([]db.Stats2024, error)
	ReturnStats2024ForTeam(teamNumber string, matchNumber int32, setNumber int32, compLevel string, compType string) ([]db.Stats2024, error)
	ReturnStats2025() ([]db.Stats2025, error)
	ReturnStats2025ForTeam(compCode string, teamNumber string, matchNumber int32, setNumber int32, compLevel string, compType string) ([]db.Stats2025, error)
	QueryDriverRanking2025(compCode string) ([]db.DriverRanking2025, error)
	QueryHumanRanking2025(compCode string) ([]db.HumanRanking2025, error)
	QueryAllShifts(int) ([]db.Shift, error)
	QueryNotes(string) ([]string, error)
	QueryNotes2025(compCode string, teamNumber string) ([]string, error)
	QueryStats2025(compCode string) ([]db.Stats2025, error)
	QueryPitImages(string) ([]db.RequestedPitImage, error)
	ReturnPitImages() ([]db.PitImage, error)
	AddNotes(db.NotesData) error
	AddNotes2025(db.NotesData2025) error
	AddPitImage(db.PitImage) error
	AddDriverRanking(db.DriverRankingData) error
	AddAction(db.Action) error
	AddDriverRanking2025(db.DriverRanking2025) error
	AddHumanRanking2025(db.HumanRanking2025) error
	DeleteFromStats2024(string, int32, int32, string) error
	DeleteFromStats2025(string, string, int32, int32, string) error
	DeleteFromNotesData2025(string, string, int32, int32, string) error
	DeleteFromActions(string, int32, int32, string) error
	DeleteFromActions2025(string, string, int32, int32, string) error
}

type Clock interface {
	Now() time.Time
}

type RealClock struct{}

func (RealClock) Now() time.Time {
	return time.Now()
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
	CompCode    string
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
	stats, err := handler.db.ReturnStats2025ForTeam(
		key.CompCode, teamNumber, key.MatchNumber, key.SetNumber, key.CompLevel, "Regular")
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

	matches, err := handler.db.ReturnMatches2025()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	assembledMatches := map[MatchAssemblyKey]request_all_matches_response.MatchT{}

	for _, match := range matches {
		key := MatchAssemblyKey{match.MatchNumber, match.CompCode, match.SetNumber, match.CompLevel}

		// Retrieve the converted match structure we have assembled so
		// far. If we haven't started assembling one yet, then start a
		// new one.
		entry, ok := assembledMatches[key]
		if !ok {
			entry = request_all_matches_response.MatchT{
				MatchNumber: match.MatchNumber,
				CompCode:    match.CompCode,
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

type requestCurrentScoutingHandler struct {
	// Map that has a key of team number with a value is a map of names to timestamps
	// so there aren't duplicate timestamps for one person.
	scoutingMap map[string]map[string]time.Time
	db          Database
	clock       Clock
}

func (handler requestCurrentScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestCurrentScouting", request_current_scouting.GetRootAsRequestCurrentScouting)
	if !success {
		return
	}
	currentTime := handler.clock.Now()
	teamNumber := string(request.TeamNumber())
	collectedBy := parseUsername(req)

	if handler.scoutingMap[teamNumber] == nil {
		handler.scoutingMap[teamNumber] = map[string]time.Time{}
	}
	handler.scoutingMap[teamNumber][collectedBy] = currentTime
	// Delete any scout information from 10+ seconds ago.
	for team, teamMap := range handler.scoutingMap {
		for name, timeStamp := range teamMap {
			if currentTime.Sub(timeStamp) >= 10*time.Second {
				delete(handler.scoutingMap[team], name)
			}
		}
	}

	var response RequestCurrentScoutingResponseT
	for name, _ := range handler.scoutingMap[teamNumber] {
		if name != collectedBy {
			response.CollectedBy = append(response.CollectedBy, &request_current_scouting_response.CollectedByT{
				Name: name,
			})
		}
	}

	builder := flatbuffers.NewBuilder(10)
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
		TeamNumber:     string(request.Team()),
		Notes:          string(request.Notes()),
		GoodDriving:    bool(request.GoodDriving()),
		BadDriving:     bool(request.BadDriving()),
		SolidPlacing:   bool(request.SolidPlacing()),
		SketchyPlacing: bool(request.SketchyPlacing()),
		GoodDefense:    bool(request.GoodDefense()),
		BadDefense:     bool(request.BadDefense()),
		EasilyDefended: bool(request.EasilyDefended()),
		NoShow:         bool(request.NoShow()),
		MatchNumber:    request.MatchNumber(),
		SetNumber:      request.SetNumber(),
		CompLevel:      string(request.CompLevel()),
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

type submitNote2025ScoutingHandler struct {
	db Database
}

func (handler submitNote2025ScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitNotes2025", submit_notes_2025.GetRootAsSubmitNotes2025)
	if !success {
		return
	}

	err = handler.db.AddNotes2025(db.NotesData2025{
		CompCode:             string(request.CompCode()),
		TeamNumber:           string(request.TeamNumber()),
		MatchNumber:          request.MatchNumber(),
		SetNumber:            request.SetNumber(),
		CompLevel:            string(request.CompLevel()),
		Notes:                string(request.Notes()),
		GoodDriving:          bool(request.GoodDriving()),
		BadDriving:           bool(request.BadDriving()),
		CoralGroundIntake:    bool(request.CoralGroundIntake()),
		CoralHpIntake:        bool(request.CoralHpIntake()),
		AlgaeGroundIntake:    bool(request.AlgaeGroundIntake()),
		SolidAlgaeShooting:   bool(request.SolidAlgaeShooting()),
		SketchyAlgaeShooting: bool(request.SketchyAlgaeShooting()),
		SolidCoralShooting:   bool(request.SolidCoralShooting()),
		SketchyCoralShooting: bool(request.SketchyCoralShooting()),
		ShuffleCoral:         bool(request.ShuffleCoral()),
		Penalties:            bool(request.Penalties()),
		GoodDefense:          bool(request.GoodDefense()),
		BadDefense:           bool(request.BadDefense()),
		EasilyDefended:       bool(request.EasilyDefended()),
		NoShow:               bool(request.NoShow()),
	})
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert notes 2025: %v", err))
		return
	}

	var response SubmitNotes2025ResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submitPitImageScoutingHandler struct {
	db Database
}

func (handler submitPitImageScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitPitImage", submit_pit_image.GetRootAsSubmitPitImage)
	if !success {
		return
	}

	err = handler.db.AddPitImage(db.PitImage{
		TeamNumber: string(request.TeamNumber()),
		CheckSum:   db.ComputeSha256FromByteArray(request.ImageDataBytes()),
		ImagePath:  string(request.ImagePath()),
		ImageData:  request.ImageDataBytes(),
	})
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert notes: %v", err))
		return
	}

	var response SubmitPitImageResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func ConvertActionsToStat2024(submit2024Actions *submit_2024_actions.Submit2024Actions) (db.Stats2024, error) {
	overall_time := int64(0)
	cycles := int64(0)
	picked_up := false
	lastPlacedTime := int64(0)
	stat := db.Stats2024{
		CompType: string(submit2024Actions.CompType()), TeamNumber: string(submit2024Actions.TeamNumber()),
		MatchNumber: submit2024Actions.MatchNumber(), SetNumber: submit2024Actions.SetNumber(), CompLevel: string(submit2024Actions.CompLevel()),
		StartingQuadrant: 0, SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
		Speaker: 0, Amp: 0, SpeakerAmplified: 0, NotesDropped: 0, Shuttled: 0, OutOfField: 0, Penalties: 0,
		TrapNote: false, Spotlight: false, AvgCycle: 0, Park: false, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "",
	}
	// Loop over all actions.
	for i := 0; i < submit2024Actions.ActionsListLength(); i++ {
		var action submit_2024_actions.Action
		if !submit2024Actions.ActionsList(&action, i) {
			return db.Stats2024{}, errors.New(fmt.Sprintf("Failed to parse submit_2024_actions.Action"))
		}
		actionTable := new(flatbuffers.Table)
		action_type := action.ActionTakenType()
		if !action.ActionTaken(actionTable) {
			return db.Stats2024{}, errors.New(fmt.Sprint("Failed to parse sub-action or sub-action was missing"))
		}
		if action_type == submit_2024_actions.ActionTypeStartMatchAction {
			var startMatchAction submit_2024_actions.StartMatchAction
			startMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.StartingQuadrant = startMatchAction.Position()
		} else if action_type == submit_2024_actions.ActionTypeMobilityAction {
			var mobilityAction submit_2024_actions.MobilityAction
			mobilityAction.Init(actionTable.Bytes, actionTable.Pos)
			if mobilityAction.Mobility() {
				stat.MobilityAuto = true
			}
		} else if action_type == submit_2024_actions.ActionTypePenaltyAction {
			var penaltyAction submit_2024_actions.PenaltyAction
			penaltyAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.Penalties += penaltyAction.Penalties()

		} else if action_type == submit_2024_actions.ActionTypeRobotDeathAction {
			var robotDeathAction submit_2024_actions.RobotDeathAction
			robotDeathAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.RobotDied = true

		} else if action_type == submit_2024_actions.ActionTypeNoShowAction {
			var NoShowAction submit_2024_actions.NoShowAction
			NoShowAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.NoShow = true

		} else if action_type == submit_2024_actions.ActionTypePickupNoteAction {
			var pick_up_action submit_2024_actions.PickupNoteAction
			pick_up_action.Init(actionTable.Bytes, actionTable.Pos)
			picked_up = true
		} else if action_type == submit_2024_actions.ActionTypePlaceNoteAction {
			var place_action submit_2024_actions.PlaceNoteAction
			place_action.Init(actionTable.Bytes, actionTable.Pos)
			if !picked_up {
				return db.Stats2024{}, errors.New(fmt.Sprintf("Got PlaceNoteAction without corresponding PickupObjectAction"))
			}
			score_type := place_action.ScoreType()
			auto := place_action.Auto()
			count_in_cycle := true
			if score_type == submit_2024_actions.ScoreTypekAMP && auto {
				stat.AmpAuto += 1
			} else if score_type == submit_2024_actions.ScoreTypekAMP && !auto {
				stat.Amp += 1
			} else if score_type == submit_2024_actions.ScoreTypekSPEAKER && !auto {
				stat.Speaker += 1
			} else if score_type == submit_2024_actions.ScoreTypekSPEAKER && auto {
				stat.SpeakerAuto += 1
			} else if score_type == submit_2024_actions.ScoreTypekSPEAKER_AMPLIFIED && !auto {
				stat.SpeakerAmplified += 1
			} else if score_type == submit_2024_actions.ScoreTypekDROPPED && auto {
				stat.NotesDroppedAuto += 1
				count_in_cycle = false
			} else if score_type == submit_2024_actions.ScoreTypekDROPPED && !auto {
				stat.NotesDropped += 1
				count_in_cycle = false
			} else if score_type == submit_2024_actions.ScoreTypekSHUTTLED {
				stat.Shuttled += 1
				count_in_cycle = false
			} else if score_type == submit_2024_actions.ScoreTypekOUT_OF_FIELD {
				stat.OutOfField += 1
				count_in_cycle = false
			} else {
				return db.Stats2024{}, errors.New(fmt.Sprintf("Got unknown ObjectType/ScoreLevel/Auto combination"))
			}
			picked_up = false
			if count_in_cycle {
				// Assuming dropped, shuttled, and out of field
				// notes are not counted in total cycle time.
				if lastPlacedTime != int64(0) {
					// If this is not the first time we place,
					// start counting cycle time. We define cycle
					// time as the time between placements.
					overall_time += int64(action.Timestamp()) - lastPlacedTime
				}
				cycles += 1
				lastPlacedTime = int64(action.Timestamp())
			}
		} else if action_type == submit_2024_actions.ActionTypeEndMatchAction {
			var endMatchAction submit_2024_actions.EndMatchAction
			endMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			if endMatchAction.StageType() == submit_2024_actions.StageTypekON_STAGE {
				stat.OnStage = true
			} else if endMatchAction.StageType() == submit_2024_actions.StageTypekPARK {
				stat.Park = true
			} else if endMatchAction.StageType() == submit_2024_actions.StageTypekHARMONY {
				stat.Harmony = true
			}
			stat.TrapNote = endMatchAction.TrapNote()
			stat.Spotlight = endMatchAction.Spotlight()
		}
	}
	if cycles != 0 {
		stat.AvgCycle = overall_time / cycles
	} else {
		stat.AvgCycle = 0
	}
	return stat, nil
}

func ConvertActionsToStat2025(submit2025Actions *submit_2025_actions.Submit2025Actions) (db.Stats2025, error) {
	overall_time := int64(0)
	cycles := int64(0)
	coral_picked_up := false
	algae_picked_up := false
	lastPlacedTime := int64(0)
	stat := db.Stats2025{
		CompCode: string(submit2025Actions.CompCode()), CompType: string(submit2025Actions.CompType()), TeamNumber: string(submit2025Actions.TeamNumber()),
		MatchNumber: submit2025Actions.MatchNumber(), SetNumber: submit2025Actions.SetNumber(), CompLevel: string(submit2025Actions.CompLevel()),
		StartingQuadrant: 0, ProcessorAuto: 0, NetAuto: 0, CoralDroppedAuto: 0, AlgaeDroppedAuto: 0, CoralMissedAuto: 0, AlgaeMissedAuto: 0, MobilityAuto: false,
		L1Auto: 0, L2Auto: 0, L3Auto: 0, L4Auto: 0,
		ProcessorTeleop: 0, NetTeleop: 0, CoralDroppedTeleop: 0, AlgaeDroppedTeleop: 0, CoralMissedTeleop: 0, AlgaeMissedTeleop: 0,
		L1Teleop: 0, L2Teleop: 0, L3Teleop: 0, L4Teleop: 0,
		Penalties: 0, ShallowCage: false, DeepCage: false, AvgCycle: 0, Park: false, BuddieClimb: false, RobotDied: false, NoShow: false, CollectedBy: "",
	}
	// Loop over all actions.
	for i := 0; i < submit2025Actions.ActionsListLength(); i++ {
		var action submit_2025_actions.Action
		if !submit2025Actions.ActionsList(&action, i) {
			return db.Stats2025{}, errors.New(fmt.Sprintf("Failed to parse submit_2025_actions.Action"))
		}
		actionTable := new(flatbuffers.Table)
		action_type := action.ActionTakenType()
		if !action.ActionTaken(actionTable) {
			return db.Stats2025{}, errors.New(fmt.Sprint("Failed to parse sub-action or sub-action was missing"))
		}
		if action_type == submit_2025_actions.ActionTypeStartMatchAction {
			var startMatchAction submit_2025_actions.StartMatchAction
			startMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.StartingQuadrant = startMatchAction.Position()
		} else if action_type == submit_2025_actions.ActionTypeMobilityAction {
			var mobilityAction submit_2025_actions.MobilityAction
			mobilityAction.Init(actionTable.Bytes, actionTable.Pos)
			if mobilityAction.Mobility() {
				stat.MobilityAuto = true
			}
		} else if action_type == submit_2025_actions.ActionTypePenaltyAction {
			var penaltyAction submit_2025_actions.PenaltyAction
			penaltyAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.Penalties += penaltyAction.Penalties()

		} else if action_type == submit_2025_actions.ActionTypeRobotDeathAction {
			var robotDeathAction submit_2025_actions.RobotDeathAction
			robotDeathAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.RobotDied = true

		} else if action_type == submit_2025_actions.ActionTypeNoShowAction {
			var NoShowAction submit_2025_actions.NoShowAction
			NoShowAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.NoShow = true

		} else if action_type == submit_2025_actions.ActionTypePickupCoralAction {
			var pick_up_action submit_2025_actions.PickupCoralAction
			pick_up_action.Init(actionTable.Bytes, actionTable.Pos)
			coral_picked_up = true
		} else if action_type == submit_2025_actions.ActionTypePlaceCoralAction {
			var place_action submit_2025_actions.PlaceCoralAction
			place_action.Init(actionTable.Bytes, actionTable.Pos)
			if !coral_picked_up {
				return db.Stats2025{}, errors.New(fmt.Sprintf("Got PlaceCoralAction without corresponding PickupObjectAction"))
			}
			score_type := place_action.ScoreType()
			auto := place_action.Auto()
			count_in_cycle := true
			if score_type == submit_2025_actions.ScoreTypekPROCESSOR && auto {
				stat.ProcessorAuto += 1
			} else if score_type == submit_2025_actions.ScoreTypekPROCESSOR && !auto {
				stat.ProcessorTeleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekNET && auto {
				stat.NetAuto += 1
			} else if score_type == submit_2025_actions.ScoreTypekNET && !auto {
				stat.NetTeleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL1 && auto {
				stat.L1Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL1 && !auto {
				stat.L1Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL2 && auto {
				stat.L2Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL2 && !auto {
				stat.L2Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL3 && auto {
				stat.L3Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL3 && !auto {
				stat.L3Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL4 && auto {
				stat.L4Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL4 && !auto {
				stat.L4Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekDROPPED && auto {
				stat.CoralDroppedAuto += 1
				count_in_cycle = false
			} else if score_type == submit_2025_actions.ScoreTypekDROPPED && !auto {
				stat.CoralDroppedTeleop += 1
				count_in_cycle = false
			} else if score_type == submit_2025_actions.ScoreTypekMISSED && auto {
				stat.CoralMissedAuto += 1
				count_in_cycle = false
			} else if score_type == submit_2025_actions.ScoreTypekMISSED && !auto {
				stat.CoralMissedTeleop += 1
				count_in_cycle = false
			} else {
				return db.Stats2025{}, errors.New(fmt.Sprintf("Got unknown ObjectType/ScoreLevel/Auto combination"))
			}
			coral_picked_up = false
			if count_in_cycle {
				if lastPlacedTime != int64(0) {
					// If this is not the first time we place,
					// start counting cycle time. We define cycle
					// time as the time between placements.
					overall_time += int64(action.Timestamp()) - lastPlacedTime
				}
				cycles += 1
				lastPlacedTime = int64(action.Timestamp())
			}
		} else if action_type == submit_2025_actions.ActionTypePickupAlgaeAction {
			var pick_up_action submit_2025_actions.PickupAlgaeAction
			pick_up_action.Init(actionTable.Bytes, actionTable.Pos)
			algae_picked_up = true
		} else if action_type == submit_2025_actions.ActionTypePlaceAlgaeAction {
			var place_action submit_2025_actions.PlaceAlgaeAction
			place_action.Init(actionTable.Bytes, actionTable.Pos)
			if !algae_picked_up {
				return db.Stats2025{}, errors.New(fmt.Sprintf("Got PlaceAlgaeAction without corresponding PickupObjectAction"))
			}
			score_type := place_action.ScoreType()
			auto := place_action.Auto()
			count_in_cycle := true
			if score_type == submit_2025_actions.ScoreTypekPROCESSOR && auto {
				stat.ProcessorAuto += 1
			} else if score_type == submit_2025_actions.ScoreTypekPROCESSOR && !auto {
				stat.ProcessorTeleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekNET && auto {
				stat.NetAuto += 1
			} else if score_type == submit_2025_actions.ScoreTypekNET && !auto {
				stat.NetTeleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL1 && auto {
				stat.L1Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL1 && !auto {
				stat.L1Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL2 && auto {
				stat.L2Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL2 && !auto {
				stat.L2Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL3 && auto {
				stat.L3Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL3 && !auto {
				stat.L3Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekL4 && auto {
				stat.L4Auto += 1
			} else if score_type == submit_2025_actions.ScoreTypekL4 && !auto {
				stat.L4Teleop += 1
			} else if score_type == submit_2025_actions.ScoreTypekDROPPED && auto {
				stat.AlgaeDroppedAuto += 1
				count_in_cycle = false
			} else if score_type == submit_2025_actions.ScoreTypekDROPPED && !auto {
				stat.AlgaeDroppedTeleop += 1
				count_in_cycle = false
			} else if score_type == submit_2025_actions.ScoreTypekMISSED && auto {
				stat.AlgaeMissedAuto += 1
				count_in_cycle = false
			} else if score_type == submit_2025_actions.ScoreTypekMISSED && !auto {
				stat.AlgaeMissedTeleop += 1
				count_in_cycle = false
			} else {
				return db.Stats2025{}, errors.New(fmt.Sprintf("Got unknown ObjectType/ScoreLevel/Auto combination"))
			}
			algae_picked_up = false
			if count_in_cycle {
				if lastPlacedTime != int64(0) {
					// If this is not the first time we place,
					// start counting cycle time. We define cycle
					// time as the time between placements.
					overall_time += int64(action.Timestamp()) - lastPlacedTime
				}
				cycles += 1
				lastPlacedTime = int64(action.Timestamp())
			}
		} else if action_type == submit_2025_actions.ActionTypeEndMatchAction {
			var endMatchAction submit_2025_actions.EndMatchAction
			endMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			if endMatchAction.CageType() == submit_2025_actions.CageTypekSHALLOW_CAGE {
				stat.ShallowCage = true
			} else if endMatchAction.CageType() == submit_2025_actions.CageTypekDEEP_CAGE {
				stat.DeepCage = true
			} else if endMatchAction.CageType() == submit_2025_actions.CageTypekPARK {
				stat.Park = true
			} else if endMatchAction.CageType() == submit_2025_actions.CageTypekBUDDIE {
				stat.BuddieClimb = true
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

// Handles a Request2024DataScouting request.
type request2024DataScoutingHandler struct {
	db Database
}

type request2025DataScoutingHandler struct {
	db Database
}

func (handler request2024DataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "Request2024DataScouting", request_2024_data_scouting.GetRootAsRequest2024DataScouting)
	if !success {
		return
	}

	stats, err := handler.db.ReturnStats2024()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response Request2024DataScoutingResponseT
	for _, stat := range stats {
		response.StatsList = append(response.StatsList, &request_2024_data_scouting_response.Stats2024T{
			TeamNumber:       stat.TeamNumber,
			MatchNumber:      stat.MatchNumber,
			SetNumber:        stat.SetNumber,
			CompLevel:        stat.CompLevel,
			StartingQuadrant: stat.StartingQuadrant,
			SpeakerAuto:      stat.SpeakerAuto,
			AmpAuto:          stat.AmpAuto,
			NotesDroppedAuto: stat.NotesDroppedAuto,
			MobilityAuto:     stat.MobilityAuto,
			Speaker:          stat.Speaker,
			Amp:              stat.Amp,
			SpeakerAmplified: stat.SpeakerAmplified,
			NotesDropped:     stat.NotesDropped,
			Shuttled:         stat.Shuttled,
			OutOfField:       stat.OutOfField,
			Penalties:        stat.Penalties,
			TrapNote:         stat.TrapNote,
			Spotlight:        stat.Spotlight,
			AvgCycle:         stat.AvgCycle,
			Park:             stat.Park,
			OnStage:          stat.OnStage,
			Harmony:          stat.Harmony,
			RobotDied:        stat.RobotDied,
			NoShow:           stat.NoShow,
			CollectedBy:      stat.CollectedBy,
			CompType:         stat.CompType,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func (handler request2025DataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "Request2025DataScouting", request_2025_data_scouting.GetRootAsRequest2025DataScouting)
	if !success {
		return
	}

	stats, err := handler.db.QueryStats2025(string(request.CompCode()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response Request2025DataScoutingResponseT
	for _, stat := range stats {
		response.StatsList = append(response.StatsList, &request_2025_data_scouting_response.Stats2025T{
			CompCode:           stat.CompCode,
			TeamNumber:         stat.TeamNumber,
			MatchNumber:        stat.MatchNumber,
			SetNumber:          stat.SetNumber,
			CompLevel:          stat.CompLevel,
			StartingQuadrant:   stat.StartingQuadrant,
			ProcessorAuto:      stat.ProcessorAuto,
			NetAuto:            stat.NetAuto,
			CoralDroppedAuto:   stat.CoralDroppedAuto,
			AlgaeDroppedAuto:   stat.AlgaeDroppedAuto,
			CoralMissedAuto:    stat.CoralMissedAuto,
			AlgaeMissedAuto:    stat.AlgaeMissedAuto,
			L1Auto:             stat.L1Auto,
			L2Auto:             stat.L2Auto,
			L3Auto:             stat.L3Auto,
			L4Auto:             stat.L4Auto,
			MobilityAuto:       stat.MobilityAuto,
			ProcessorTeleop:    stat.ProcessorTeleop,
			NetTeleop:          stat.NetTeleop,
			CoralDroppedTeleop: stat.CoralDroppedTeleop,
			AlgaeDroppedTeleop: stat.AlgaeDroppedTeleop,
			CoralMissedTeleop:  stat.CoralMissedTeleop,
			AlgaeMissedTeleop:  stat.AlgaeMissedTeleop,
			L1Teleop:           stat.L1Teleop,
			L2Teleop:           stat.L2Teleop,
			L3Teleop:           stat.L3Teleop,
			L4Teleop:           stat.L4Teleop,
			Penalties:          stat.Penalties,
			AvgCycle:           stat.AvgCycle,
			Park:               stat.Park,
			ShallowCage:        stat.ShallowCage,
			DeepCage:           stat.DeepCage,
			BuddieClimb:        stat.BuddieClimb,
			RobotDied:          stat.RobotDied,
			NoShow:             stat.NoShow,
			CollectedBy:        stat.CollectedBy,
			CompType:           stat.CompType,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestAllPitImagesHandler struct {
	db Database
}

func (handler requestAllPitImagesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestAllPitImages", request_all_pit_images.GetRootAsRequestAllPitImages)
	if !success {
		return
	}

	images, err := handler.db.ReturnPitImages()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to get pit images: %v", err))
		return
	}

	var response RequestAllPitImagesResponseT
	for _, data := range images {
		response.PitImageList = append(response.PitImageList, &request_all_pit_images_response.PitImageT{
			TeamNumber: data.TeamNumber,
			ImagePath:  data.ImagePath,
			CheckSum:   data.CheckSum,
		})
	}

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestPitImagesHandler struct {
	db Database
}

func (handler requestPitImagesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestPitImages", request_pit_images.GetRootAsRequestPitImages)
	if !success {
		return
	}

	images, err := handler.db.QueryPitImages(string(request.TeamNumber()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query pit images: %v", err))
		return
	}

	var response RequestPitImagesResponseT
	for _, data := range images {
		response.PitImageList = append(response.PitImageList, &request_pit_images_response.PitImageT{
			TeamNumber: data.TeamNumber,
			ImagePath:  data.ImagePath,
			CheckSum:   data.CheckSum,
		})
	}

	builder := flatbuffers.NewBuilder(1024)
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

	notes, err := handler.db.QueryNotes(string(request.Team()))
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

type requestNotes2025ForTeamHandler struct {
	db Database
}

func (handler requestNotes2025ForTeamHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestNotes2025ForTeam", request_notes_2025_for_team.GetRootAsRequestNotes2025ForTeam)
	if !success {
		return
	}

	notes, err := handler.db.QueryNotes2025(string(request.CompCode()), string(request.TeamNumber()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query notes 2025: %v", err))
		return
	}

	var response RequestNotes2025ForTeamResponseT
	for _, data := range notes {
		response.Notes2025 = append(response.Notes2025, &request_notes_2025_for_team_response.Note2025T{data})
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
			R1Scouter:   shifts.R1scouter,
			R2Scouter:   shifts.R2scouter,
			R3Scouter:   shifts.R3scouter,
			B1Scouter:   shifts.B1scouter,
			B2Scouter:   shifts.B2scouter,
			B3Scouter:   shifts.B3scouter,
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
			R1scouter:   string(match_assignment.R1Scouter()),
			R2scouter:   string(match_assignment.R2Scouter()),
			R3scouter:   string(match_assignment.R3Scouter()),
			B1scouter:   string(match_assignment.B1Scouter()),
			B2scouter:   string(match_assignment.B2Scouter()),
			B3scouter:   string(match_assignment.B3Scouter()),
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
		Rank1:       string(request.Rank1()),
		Rank2:       string(request.Rank2()),
		Rank3:       string(request.Rank3()),
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

type SubmitDriverRanking2025Handler struct {
	db Database
}

func (handler SubmitDriverRanking2025Handler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitDriverRanking2025", submit_driver_ranking_2025.GetRootAsSubmitDriverRanking2025)
	if !success {
		return
	}

	err = handler.db.AddDriverRanking2025(db.DriverRanking2025{
		CompCode:    string(request.CompCode()),
		MatchNumber: request.MatchNumber(),
		TeamNumber:  string(request.TeamNumber()),
		Score:       request.Score(),
	})

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert driver ranking: %v", err))
		return
	}

	var response SubmitDriverRanking2025ResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type SubmitHumanRanking2025Handler struct {
	db Database
}

func (handler SubmitHumanRanking2025Handler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitHumanRanking2025", submit_human_ranking_2025.GetRootAsSubmitHumanRanking2025)
	if !success {
		return
	}

	err = handler.db.AddHumanRanking2025(db.HumanRanking2025{
		CompCode:    string(request.CompCode()),
		MatchNumber: request.MatchNumber(),
		TeamNumber:  string(request.TeamNumber()),
		Score:       request.Score(),
	})

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert driver ranking: %v", err))
		return
	}

	var response SubmitHumanRanking2025ResponseT
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
			NoShow:         note.NoShow,
			MatchNumber:    note.MatchNumber,
			CompLevel:      note.CompLevel,
			SetNumber:      note.SetNumber,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestAllNotes2025Handler struct {
	db Database
}

func (handler requestAllNotes2025Handler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestAllNotes2025", request_all_notes_2025.GetRootAsRequestAllNotes2025)
	if !success {
		return
	}

	notes, err := handler.db.ReturnAllNotes2025(string(request.CompCode()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response RequestAllNotes2025ResponseT
	for _, note := range notes {
		response.Note2025List = append(response.Note2025List, &request_all_notes_2025_response.Note2025T{
			CompCode:             note.CompCode,
			TeamNumber:           note.TeamNumber,
			MatchNumber:          note.MatchNumber,
			SetNumber:            note.SetNumber,
			CompLevel:            note.CompLevel,
			Notes:                note.Notes,
			GoodDriving:          note.GoodDriving,
			BadDriving:           note.BadDriving,
			CoralGroundIntake:    note.CoralGroundIntake,
			CoralHpIntake:        note.CoralHpIntake,
			AlgaeGroundIntake:    note.AlgaeGroundIntake,
			SolidAlgaeShooting:   note.SolidAlgaeShooting,
			SketchyAlgaeShooting: note.SketchyAlgaeShooting,
			SolidCoralShooting:   note.SolidCoralShooting,
			SketchyCoralShooting: note.SketchyCoralShooting,
			ShuffleCoral:         note.ShuffleCoral,
			Penalties:            note.Penalties,
			GoodDefense:          note.GoodDefense,
			BadDefense:           note.BadDefense,
			EasilyDefended:       note.EasilyDefended,
			NoShow:               note.NoShow,
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

type RankingIdentifier struct {
	CompCode   string
	TeamNumber string
}

type AveragedRankingData struct {
	CompCode   string
	TeamNumber string
	Score      float32
}

type RequestAveragedHumanRankings2025Handler struct {
	db Database
}

func (handler RequestAveragedHumanRankings2025Handler) ServeHTTP(w http.ResponseWriter, req *http.Request) {

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestAveragedHumanRankings2025", request_averaged_human_rankings_2025.GetRootAsRequestAveragedHumanRankings2025)
	if !success {
		return
	}

	rankings, err := handler.db.QueryHumanRanking2025(string(request.CompCode()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	rankingSums := make(map[RankingIdentifier]db.HumanRanking2025)
	currNumOfRankings := make(map[RankingIdentifier]float32)

	var response RequestAveragedHumanRankings2025ResponseT
	for _, ranking := range rankings {
		identifier := RankingIdentifier{CompCode: string(ranking.CompCode), TeamNumber: string(ranking.TeamNumber)}
		sum, ok := rankingSums[identifier]
		if ok {
			sum.Score += ranking.Score
			rankingSums[identifier] = sum
		} else {
			sum = db.HumanRanking2025{CompCode: string(ranking.CompCode), TeamNumber: string(ranking.TeamNumber), Score: 0}
			sum.Score = ranking.Score
			rankingSums[identifier] = sum
		}

		_, ok2 := currNumOfRankings[identifier]
		if ok2 {
			currNumOfRankings[identifier] += float32(1)
		} else {
			currNumOfRankings[identifier] = float32(1)
		}
	}

	for _, ranking := range rankingSums {
		identifier := RankingIdentifier{CompCode: string(ranking.CompCode), TeamNumber: string(ranking.TeamNumber)}
		score := float32(ranking.Score) / currNumOfRankings[identifier]
		response.Rankings2025List = append(response.Rankings2025List, &request_averaged_human_rankings_2025_response.HumanRanking2025T{
			CompCode:   ranking.CompCode,
			TeamNumber: ranking.TeamNumber,
			Score:      score,
		})
	}

	sort.Slice(response.Rankings2025List, func(i, j int) bool {
		return response.Rankings2025List[i].TeamNumber < response.Rankings2025List[j].TeamNumber
	})

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type RequestAveragedDriverRankings2025Handler struct {
	db Database
}

func (handler RequestAveragedDriverRankings2025Handler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestAveragedDriverRankings2025", request_averaged_driver_rankings_2025.GetRootAsRequestAveragedDriverRankings2025)
	if !success {
		return
	}

	rankings, err := handler.db.QueryDriverRanking2025(string(request.CompCode()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	rankingSums := make(map[RankingIdentifier]db.DriverRanking2025)
	currNumOfRankings := make(map[RankingIdentifier]float32)

	var response RequestAveragedDriverRankings2025ResponseT
	for _, ranking := range rankings {
		identifier := RankingIdentifier{CompCode: string(ranking.CompCode), TeamNumber: string(ranking.TeamNumber)}
		sum, ok := rankingSums[identifier]
		if ok {
			sum.Score += ranking.Score
			rankingSums[identifier] = sum
		} else {
			sum = db.DriverRanking2025{CompCode: string(ranking.CompCode), TeamNumber: string(ranking.TeamNumber), Score: 0}
			sum.Score = ranking.Score
			rankingSums[identifier] = sum
		}

		_, ok2 := currNumOfRankings[identifier]
		if ok2 {
			currNumOfRankings[identifier] += float32(1)
		} else {
			currNumOfRankings[identifier] = float32(1)
		}
	}

	for _, ranking := range rankingSums {
		identifier := RankingIdentifier{CompCode: string(ranking.CompCode), TeamNumber: string(ranking.TeamNumber)}
		score := float32(ranking.Score) / currNumOfRankings[identifier]
		response.Rankings2025List = append(response.Rankings2025List, &request_averaged_driver_rankings_2025_response.DriverRanking2025T{
			CompCode:   ranking.CompCode,
			TeamNumber: ranking.TeamNumber,
			Score:      score,
		})
	}

	sort.Slice(response.Rankings2025List, func(i, j int) bool {
		return response.Rankings2025List[i].TeamNumber < response.Rankings2025List[j].TeamNumber
	})

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submit2024ActionsHandler struct {
	db Database
}

func (handler submit2024ActionsHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	// Get the username of the person submitting the data.
	username := parseUsername(req)

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		log.Println("Failed to receive submission request from", username)
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "Submit2024Actions", submit_2024_actions.GetRootAsSubmit2024Actions)
	if !success {
		log.Println("Failed to parse submission request from", username)
		return
	}

	log.Println("Got actions for match", request.MatchNumber(), "team", string(request.TeamNumber()), "type", string(request.CompType()), "from", username)

	for i := 0; i < request.ActionsListLength(); i++ {

		var action Action2024
		request.ActionsList(&action, i)

		dbAction := db.Action{
			CompType:    string(request.CompType()),
			TeamNumber:  string(request.TeamNumber()),
			MatchNumber: request.MatchNumber(),
			SetNumber:   request.SetNumber(),
			CompLevel:   string(request.CompLevel()),
			//TODO: Serialize CompletedAction
			CompletedAction: []byte{},
			Timestamp:       action.Timestamp(),
			CollectedBy:     username,
		}

		// Do some error checking.
		if action.Timestamp() < 0 {
			log.Println("Got action with invalid timestamp (", action.Timestamp(), ") from", username)
			respondWithError(w, http.StatusBadRequest, fmt.Sprint(
				"Invalid timestamp field value of ", action.Timestamp()))
			return
		}

		err = handler.db.AddAction(dbAction)
		if err != nil {
			log.Println("Failed to add action from", username, "to the database:", err)
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to add action to database: ", err))
			return
		}
	}

	stats, err := ConvertActionsToStat2024(request)
	if err != nil {
		log.Println("Failed to add action from", username, "to the database:", err)
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to convert actions to stats: ", err))
		return
	}

	stats.CollectedBy = username

	err = handler.db.AddToStats2024(stats)
	if err != nil {
		log.Println("Failed to submit stats from", username, "to the database:", err)
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to submit stats2024: ", stats, ": ", err))
		return
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&Submit2024ActionsResponseT{}).Pack(builder))
	w.Write(builder.FinishedBytes())

	log.Println("Successfully added stats from", username)
}

type submit2025ActionsHandler struct {
	db Database
}

func (handler submit2025ActionsHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	// Get the username of the person submitting the data.
	username := parseUsername(req)

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		log.Println("Failed to receive submission request from", username)
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "Submit2025Actions", submit_2025_actions.GetRootAsSubmit2025Actions)
	if !success {
		log.Println("Failed to parse submission request from", username)
		return
	}

	log.Println("Got actions for match", request.MatchNumber(), "team", string(request.TeamNumber()), "type", string(request.CompType()), "from", username)

	for i := 0; i < request.ActionsListLength(); i++ {

		var action Action2025
		request.ActionsList(&action, i)

		dbAction := db.Action{
			CompCode:    string(request.CompCode()),
			CompType:    string(request.CompType()),
			TeamNumber:  string(request.TeamNumber()),
			MatchNumber: request.MatchNumber(),
			SetNumber:   request.SetNumber(),
			CompLevel:   string(request.CompLevel()),
			//TODO: Serialize CompletedAction
			CompletedAction: []byte{},
			Timestamp:       action.Timestamp(),
			CollectedBy:     username,
		}

		// Do some error checking.
		if action.Timestamp() < 0 {
			log.Println("Got action with invalid timestamp (", action.Timestamp(), ") from", username)
			respondWithError(w, http.StatusBadRequest, fmt.Sprint(
				"Invalid timestamp field value of ", action.Timestamp()))
			return
		}

		err = handler.db.AddAction(dbAction)
		if err != nil {
			log.Println("Failed to add action from", username, "to the database:", err)
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to add action to database: ", err))
			return
		}
	}

	stats, err := ConvertActionsToStat2025(request)
	if err != nil {
		log.Println("Failed to add action from", username, "to the database:", err)
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to convert actions to stats: ", err))
		return
	}

	stats.CollectedBy = username

	err = handler.db.AddToStats2025(stats)
	if err != nil {
		log.Println("Failed to submit stats from", username, "to the database:", err)
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to submit stats2025: ", stats, ": ", err))
		return
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&Submit2025ActionsResponseT{}).Pack(builder))
	w.Write(builder.FinishedBytes())

	log.Println("Successfully added stats from", username)
}

type Delete2024DataScoutingHandler struct {
	db Database
}

func (handler Delete2024DataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "Delete2024DataScouting", delete_2024_data_scouting.GetRootAsDelete2024DataScouting)
	if !success {
		return
	}

	err = handler.db.DeleteFromStats2024(
		string(request.CompLevel()),
		request.MatchNumber(),
		request.SetNumber(),
		string(request.TeamNumber()))

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to delete from stats2024: %v", err))
		return
	}

	err = handler.db.DeleteFromActions(
		string(request.CompLevel()),
		request.MatchNumber(),
		request.SetNumber(),
		string(request.TeamNumber()))

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to delete from actions: %v", err))
		return
	}

	var response Delete2024DataScoutingResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type Delete2025DataScoutingHandler struct {
	db Database
}

func (handler Delete2025DataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "Delete2025DataScouting", delete_2025_data_scouting.GetRootAsDelete2025DataScouting)
	if !success {
		return
	}

	err = handler.db.DeleteFromStats2025(
		string(request.CompCode()),
		string(request.CompLevel()),
		request.MatchNumber(),
		request.SetNumber(),
		string(request.TeamNumber()))

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to delete from stats2025: %v", err))
		return
	}

	err = handler.db.DeleteFromActions2025(
		string(request.CompCode()),
		string(request.CompLevel()),
		request.MatchNumber(),
		request.SetNumber(),
		string(request.TeamNumber()))

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to delete from actions: %v", err))
		return
	}

	var response Delete2025DataScoutingResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type DeleteNotes2025Handler struct {
	db Database
}

func (handler DeleteNotes2025Handler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "DeleteNotes2025", delete_notes_2025.GetRootAsDeleteNotes2025)
	if !success {
		return
	}

	err = handler.db.DeleteFromNotesData2025(
		string(request.CompCode()),
		string(request.CompLevel()),
		request.MatchNumber(),
		request.SetNumber(),
		string(request.TeamNumber()))

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to delete from notes 2025: %v", err))
		return
	}

	var response DeleteNotes2025ResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func HandleRequests(db Database, scoutingServer server.ScoutingServer, clock Clock) {
	scoutingServer.HandleFunc("/requests", unknown)
	scoutingServer.Handle("/requests/request/all_matches", requestAllMatchesHandler{db})
	scoutingServer.Handle("/requests/request/all_notes", requestAllNotesHandler{db})
	scoutingServer.Handle("/requests/request/all_notes_2025", requestAllNotes2025Handler{db})
	scoutingServer.Handle("/requests/request/all_driver_rankings", requestAllDriverRankingsHandler{db})
	scoutingServer.Handle("/requests/request/averaged_driver_rankings_2025", RequestAveragedDriverRankings2025Handler{db})
	scoutingServer.Handle("/requests/request/averaged_human_rankings_2025", RequestAveragedHumanRankings2025Handler{db})
	scoutingServer.Handle("/requests/request/2024_data_scouting", request2024DataScoutingHandler{db})
	scoutingServer.Handle("/requests/request/2025_data_scouting", request2025DataScoutingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_notes", submitNoteScoutingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_notes_2025", submitNote2025ScoutingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_pit_image", submitPitImageScoutingHandler{db})
	scoutingServer.Handle("/requests/request/pit_images", requestPitImagesHandler{db})
	scoutingServer.Handle("/requests/request/all_pit_images", requestAllPitImagesHandler{db})
	scoutingServer.Handle("/requests/request/current_scouting", requestCurrentScoutingHandler{make(map[string]map[string]time.Time), db, clock})
	scoutingServer.Handle("/requests/request/notes_for_team", requestNotesForTeamHandler{db})
	scoutingServer.Handle("/requests/request/notes_2025_for_team", requestNotes2025ForTeamHandler{db})
	scoutingServer.Handle("/requests/submit/shift_schedule", submitShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/request/shift_schedule", requestShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/submit/submit_driver_ranking", SubmitDriverRankingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_2024_actions", submit2024ActionsHandler{db})
	scoutingServer.Handle("/requests/submit/submit_2025_actions", submit2025ActionsHandler{db})
	scoutingServer.Handle("/requests/submit/submit_driver_ranking_2025", SubmitDriverRanking2025Handler{db})
	scoutingServer.Handle("/requests/submit/submit_human_ranking_2025", SubmitHumanRanking2025Handler{db})
	scoutingServer.Handle("/requests/delete/delete_2024_data_scouting", Delete2024DataScoutingHandler{db})
	scoutingServer.Handle("/requests/delete/delete_2025_data_scouting", Delete2025DataScoutingHandler{db})
	scoutingServer.Handle("/requests/delete/delete_notes_2025", DeleteNotes2025Handler{db})
}
