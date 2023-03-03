package rankings

import (
	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/scraping/background"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	"net/http"
	"reflect"
	"strings"
	"testing"
	"time"
)

type MockDatabase struct {
	rankings []db.Ranking
}

func (database *MockDatabase) AddOrUpdateRankings(data db.Ranking) error {
	database.rankings = append(database.rankings, data)
	return nil
}

func ServeRankings(t *testing.T, h http.Handler) http.Handler {
	fn := func(w http.ResponseWriter, r *http.Request) {
		// Make sure that the rankings are requested properly.
		if !strings.HasSuffix(r.URL.Path, "/2016nytr/rankings") {
			t.Error("Got unexpected URL: ", r.URL.Path)
		}

		r.URL.Path = "scraping/test_data/2016_nytr_rankings.json"

		h.ServeHTTP(w, r)
	}

	return http.HandlerFunc(fn)
}

func TestGetRankings(t *testing.T) {
	database := MockDatabase{}
	scraper := background.BackgroundScraper{}
	tbaServer := server.NewScoutingServer()
	tbaServer.Handle("/", ServeRankings(t, http.FileServer(http.Dir("../../"))))
	tbaServer.Start(8000)
	defer tbaServer.Stop()

	scraper.Start(func() {
		GetRankings(&database, 0, "", "scouting_test_config.json")
	})
	defer scraper.Stop()

	for {
		if len(database.rankings) > 0 {
			break
		}

		time.Sleep(time.Second)
	}

	beginningThreeExpected := []db.Ranking{
		{TeamNumber: 359, Losses: 1, Wins: 11, Ties: 0, Rank: 1, Dq: 0},
		{TeamNumber: 5254, Losses: 1, Wins: 11, Ties: 0, Rank: 2, Dq: 0},
		{TeamNumber: 3990, Losses: 1, Wins: 11, Ties: 0, Rank: 3, Dq: 0},
	}

	endThreeExpected := []db.Ranking{
		{TeamNumber: 5943, Losses: 10, Wins: 2, Ties: 0, Rank: 34, Dq: 0},
		{TeamNumber: 4203, Losses: 10, Wins: 2, Ties: 0, Rank: 35, Dq: 0},
		{TeamNumber: 5149, Losses: 10, Wins: 2, Ties: 0, Rank: 36, Dq: 0},
	}

	if !reflect.DeepEqual(beginningThreeExpected, database.rankings[0:3]) {
		t.Fatal("Got %#v, but expected %#v.", database.rankings[0:3], beginningThreeExpected)
	}

	if !reflect.DeepEqual(endThreeExpected, database.rankings[33:]) {
		t.Fatal("Got %#v, but expected %#v.", database.rankings[33:], beginningThreeExpected)
	}
}
