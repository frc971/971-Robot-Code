package main

// To run the demo, ensure that you have a file named scouting_config.json at the workspace root with your TBA api key in it.
import (
	"encoding/json"
	"flag"
	"fmt"
	"log"

	"github.com/davecgh/go-spew/spew"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
)

func main() {
	jsonPtr := flag.Bool("json", false, "If set, dump as JSON, rather than Go debug output.")
	demoCategory := flag.String("category", "matches", "Decide whether to demo matches or rankings.")

	flag.Parse()

	if *demoCategory == "rankings" {
		// Get all the rankings.
		rankings, err := scraping.AllRankings(2016, "nytr", "")
		if err != nil {
			log.Fatal("Failed to scrape ranking list: ", err)
		}

		// Dump the rankings.
		if *jsonPtr {
			jsonData, err := json.MarshalIndent(rankings, "", "  ")
			if err != nil {
				log.Fatal("Failed to turn ranking list into JSON: ", err)
			}
			fmt.Println(string(jsonData))
		} else {
			spew.Dump(rankings)
		}
	} else if *demoCategory == "matches" {
		// Get all the matches.
		matches, err := scraping.AllMatches(2016, "nytr", "")
		if err != nil {
			log.Fatal("Failed to scrape match list: ", err)
		}

		// Dump the matches.
		if *jsonPtr {
			jsonData, err := json.MarshalIndent(matches, "", "  ")
			if err != nil {
				log.Fatal("Failed to turn match list into JSON: ", err)
			}
			fmt.Println(string(jsonData))
		} else {
			spew.Dump(matches)
		}
	}
}
