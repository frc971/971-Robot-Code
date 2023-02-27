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

func dumpData[T interface{}](jsonPtr *bool, category string) {
	// Get all the data.
	data, err := scraping.GetAllData[T](2016, "nytr", "", category)
	if err != nil {
		log.Fatal("Failed to scrape ", category, " data: ", err)
	}

	// Dump the data.
	if *jsonPtr {
		jsonData, err := json.MarshalIndent(data, "", "  ")
		if err != nil {
			log.Fatal("Failed to turn ranking list into JSON: ", err)
		}
		fmt.Println(string(jsonData))
	} else {
		spew.Dump(data)
	}
}

func main() {
	jsonPtr := flag.Bool("json", false, "If set, dump as JSON, rather than Go debug output.")
	demoCategory := flag.String("category", "matches", "Decide whether to demo matches or rankings.")

	flag.Parse()

	if *demoCategory == "rankings" {
		dumpData[scraping.EventRanking](jsonPtr, "rankings")
	} else if *demoCategory == "matches" {
		dumpData[[]scraping.Match](jsonPtr, "matches")
	}
}
