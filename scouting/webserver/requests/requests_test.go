package requests

import (
	"bytes"
	"io"
	"net/http"
	"reflect"
	"testing"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/debug"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

// Validates that an unhandled address results in a 404.
func Test404(t *testing.T) {
	db := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scrapeEmtpyMatchList, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	resp, err := http.Get("http://localhost:8080/requests/foo")
	if err != nil {
		t.Fatalf("Failed to get data: %v", err)
	}
	if resp.StatusCode != http.StatusNotFound {
		t.Fatalf("Expected error code 404, but got %d instead", resp.Status)
	}
}

// Validates that we can submit new data scouting data.
func TestSubmitDataScoutingError(t *testing.T) {
	db := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scrapeEmtpyMatchList, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	resp, err := http.Post("http://localhost:8080/requests/submit/data_scouting", "application/octet-stream", bytes.NewReader([]byte("")))
	if err != nil {
		t.Fatalf("Failed to send request: %v", err)
	}
	if resp.StatusCode != http.StatusBadRequest {
		t.Fatal("Unexpected status code. Got", resp.Status)
	}

	responseBytes, err := io.ReadAll(resp.Body)
	if err != nil {
		t.Fatal("Failed to read response bytes:", err)
	}
	errorResponse := error_response.GetRootAsErrorResponse(responseBytes, 0)

	errorMessage := string(errorResponse.ErrorMessage())
	if errorMessage != "Failed to parse SubmitDataScouting: runtime error: index out of range [3] with length 0" {
		t.Fatal("Got mismatched error message:", errorMessage)
	}
}

// Validates that we can submit new data scouting data.
func TestSubmitDataScouting(t *testing.T) {
	db := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scrapeEmtpyMatchList, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_data_scouting.SubmitDataScoutingT{
		Team:            971,
		Match:           1,
		MissedShotsAuto: 9971,
		UpperGoalAuto:   9971,
		LowerGoalAuto:   9971,
		MissedShotsTele: 9971,
		UpperGoalTele:   9971,
		LowerGoalTele:   9971,
		DefenseRating:   9971,
		Climbing:        9971,
	}).Pack(builder))

	response, err := debug.SubmitDataScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit data scouting: ", err)
	}

	// We get an empty response back. Validate that.
	expected := submit_data_scouting_response.SubmitDataScoutingResponseT{}
	if !reflect.DeepEqual(expected, *response) {
		t.Fatal("Expected ", expected, ", but got:", *response)
	}
}

// Validates that we can request the full match list.
func TestRequestAllMatches(t *testing.T) {
	db := MockDatabase{
		matches: []db.Match{
			{
				MatchNumber: 1, Round: 1, CompLevel: "qual",
				R1: 5, R2: 42, R3: 600, B1: 971, B2: 400, B3: 200,
			},
			{
				MatchNumber: 2, Round: 1, CompLevel: "qual",
				R1: 6, R2: 43, R3: 601, B1: 972, B2: 401, B3: 201,
			},
			{
				MatchNumber: 3, Round: 1, CompLevel: "qual",
				R1: 7, R2: 44, R3: 602, B1: 973, B2: 402, B3: 202,
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scrapeEmtpyMatchList, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_matches.RequestAllMatchesT{}).Pack(builder))

	response, err := debug.RequestAllMatches("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	expected := request_all_matches_response.RequestAllMatchesResponseT{
		MatchList: []*request_all_matches_response.MatchT{
			// MatchNumber, Round, CompLevel
			// R1, R2, R3, B1, B2, B3
			{
				1, 1, "qual",
				5, 42, 600, 971, 400, 200,
			},
			{
				2, 1, "qual",
				6, 43, 601, 972, 401, 201,
			},
			{
				3, 1, "qual",
				7, 44, 602, 973, 402, 202,
			},
		},
	}
	if len(expected.MatchList) != len(response.MatchList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.MatchList {
		if !reflect.DeepEqual(*match, *response.MatchList[i]) {
			t.Fatal("Expected for match", i, ":", *match, ", but got:", *response.MatchList[i])
		}
	}

}

// Validates that we can request the full match list.
func TestRequestMatchesForTeam(t *testing.T) {
	db := MockDatabase{
		matches: []db.Match{
			{
				MatchNumber: 1, Round: 1, CompLevel: "qual",
				R1: 5, R2: 42, R3: 600, B1: 971, B2: 400, B3: 200,
			},
			{
				MatchNumber: 2, Round: 1, CompLevel: "qual",
				R1: 6, R2: 43, R3: 601, B1: 972, B2: 401, B3: 201,
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scrapeEmtpyMatchList, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_matches_for_team.RequestMatchesForTeamT{
		Team: 971,
	}).Pack(builder))

	response, err := debug.RequestMatchesForTeam("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	expected := request_matches_for_team_response.RequestMatchesForTeamResponseT{
		MatchList: []*request_matches_for_team_response.MatchT{
			// MatchNumber, Round, CompLevel
			// R1, R2, R3, B1, B2, B3
			{
				1, 1, "qual",
				5, 42, 600, 971, 400, 200,
			},
		},
	}
	if len(expected.MatchList) != len(response.MatchList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.MatchList {
		if !reflect.DeepEqual(*match, *response.MatchList[i]) {
			t.Fatal("Expected for match", i, ":", *match, ", but got:", *response.MatchList[i])
		}
	}
}

// Validates that we can request the stats.
func TestRequestDataScouting(t *testing.T) {
	db := MockDatabase{
		stats: []db.Stats{
			{
				TeamNumber: 971, MatchNumber: 1,
				ShotsMissed: 1, UpperGoalShots: 2, LowerGoalShots: 3,
				ShotsMissedAuto: 4, UpperGoalAuto: 5, LowerGoalAuto: 6,
				PlayedDefense: 7, Climbing: 8,
			},
			{
				TeamNumber: 972, MatchNumber: 1,
				ShotsMissed: 2, UpperGoalShots: 3, LowerGoalShots: 4,
				ShotsMissedAuto: 5, UpperGoalAuto: 6, LowerGoalAuto: 7,
				PlayedDefense: 8, Climbing: 9,
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&db, scrapeEmtpyMatchList, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_data_scouting.RequestDataScoutingT{}).Pack(builder))

	response, err := debug.RequestDataScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	expected := request_data_scouting_response.RequestDataScoutingResponseT{
		StatsList: []*request_data_scouting_response.StatsT{
			// Team, Match,
			// MissedShotsAuto, UpperGoalAuto, LowerGoalAuto,
			// MissedShotsTele, UpperGoalTele, LowerGoalTele,
			// DefenseRating, Climbing,
			{
				971, 1,
				4, 5, 6,
				1, 2, 3,
				7, 8,
			},
			{
				972, 1,
				5, 6, 7,
				2, 3, 4,
				8, 9,
			},
		},
	}
	if len(expected.StatsList) != len(response.StatsList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.StatsList {
		if !reflect.DeepEqual(*match, *response.StatsList[i]) {
			t.Fatal("Expected for stats", i, ":", *match, ", but got:", *response.StatsList[i])
		}
	}
}

// Validates that we can download the schedule from The Blue Alliance.
func TestRefreshMatchList(t *testing.T) {
	scrapeMockSchedule := func(int32, string) ([]scraping.Match, error) {
		return []scraping.Match{
			{
				CompLevel:   "qual",
				MatchNumber: 1,
				Alliances: scraping.Alliances{
					Red: scraping.Alliance{
						TeamKeys: []string{
							"100",
							"200",
							"300",
						},
					},
					Blue: scraping.Alliance{
						TeamKeys: []string{
							"101",
							"201",
							"301",
						},
					},
				},
				WinningAlliance: "",
				EventKey:        "",
				Time:            0,
				PredictedTime:   0,
				ActualTime:      0,
				PostResultTime:  0,
				ScoreBreakdowns: scraping.ScoreBreakdowns{},
			},
		}, nil
	}

	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&database, scrapeMockSchedule, scoutingServer)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&refresh_match_list.RefreshMatchListT{}).Pack(builder))

	response, err := debug.RefreshMatchList("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	// Validate the response.
	expected := refresh_match_list_response.RefreshMatchListResponseT{}
	if !reflect.DeepEqual(expected, *response) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}

	// Make sure that the data made it into the database.
	expectedMatches := []db.Match{
		{
			MatchNumber: 1,
			Round:       1,
			CompLevel:   "qual",
			R1:          100,
			R2:          200,
			R3:          300,
			B1:          101,
			B2:          201,
			B3:          301,
		},
	}
	if !reflect.DeepEqual(expectedMatches, database.matches) {
		t.Fatal("Expected ", expectedMatches, ", but got ", database.matches)
	}
}

// A mocked database we can use for testing. Add functionality to this as
// needed for your tests.

type MockDatabase struct {
	matches []db.Match
	stats   []db.Stats
}

func (database *MockDatabase) AddToMatch(match db.Match) error {
	database.matches = append(database.matches, match)
	return nil
}

func (database *MockDatabase) AddToStats(stats db.Stats) error {
	database.stats = append(database.stats, stats)
	return nil
}

func (database *MockDatabase) ReturnMatches() ([]db.Match, error) {
	return database.matches, nil
}

func (database *MockDatabase) ReturnStats() ([]db.Stats, error) {
	return database.stats, nil
}

func (database *MockDatabase) QueryMatches(requestedTeam int32) ([]db.Match, error) {
	var matches []db.Match
	for _, match := range database.matches {
		for _, team := range []int32{match.R1, match.R2, match.R3, match.B1, match.B2, match.B3} {
			if team == requestedTeam {
				matches = append(matches, match)
				break
			}
		}
	}
	return matches, nil
}

func (database *MockDatabase) QueryStats(int) ([]db.Stats, error) {
	return []db.Stats{}, nil
}

// Returns an empty match list from the fake The Blue Alliance scraping.
func scrapeEmtpyMatchList(int32, string) ([]scraping.Match, error) {
	return nil, nil
}
