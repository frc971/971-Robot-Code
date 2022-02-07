package main

// To run the demo, ensure that you have a file named scouting_config.json at the workspace root with your TBA api key in it.
import (
	"log"

	"github.com/davecgh/go-spew/spew"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
)

func main() {
	// Get all the matches.
	matches, err := scraping.AllMatches("2016", "nytr", "")
	// Fail on error.
	if err != nil {
		log.Fatal("Error:", err.Error)
	}
	// Dump the matches.
	spew.Dump(matches)
}
