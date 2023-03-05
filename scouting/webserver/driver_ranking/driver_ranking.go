package driver_ranking

import (
	"encoding/csv"
	"errors"
	"fmt"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"

	"github.com/bazelbuild/rules_go/go/runfiles"
	"github.com/frc971/971-Robot-Code/scouting/db"
)

const (
	DEFAULT_SCRIPT_PATH = "org_frc971/scouting/DriverRank/src/DriverRank.jl"
)

type Database interface {
	ReturnAllDriverRankings() ([]db.DriverRankingData, error)
	AddParsedDriverRanking(data db.ParsedDriverRankingData) error
}

func writeToCsv(filename string, records [][]string) error {
	file, err := os.Create(filename)
	if err != nil {
		return errors.New(fmt.Sprintf("Failed to create %s: %v", filename, err))
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	writer.WriteAll(records)
	if err := writer.Error(); err != nil {
		return errors.New(fmt.Sprintf("Failed to write to %s: %v", filename, err))
	}

	return nil
}

func readFromCsv(filename string) (records [][]string, err error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, errors.New(fmt.Sprintf("Failed to open %s: %v", filename, err))
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err = reader.ReadAll()
	if err != nil {
		return nil, errors.New(fmt.Sprintf("Failed to parse %s as CSV: %v", filename, err))
	}

	return
}

// Runs the specified script on the DriverRankingData that the scouts collected
// and dumps the results in the ParsedDriverRankingData table. If the script is
// not specified (i.e. empty string) then the
// scouting/DriverRank/src/DriverRank.jl script is called instead.
func GenerateFullDriverRanking(database Database, scriptPath string) {
	rawRankings, err := database.ReturnAllDriverRankings()
	if err != nil {
		log.Println("Failed to get raw driver ranking data: ", err)
		return
	}

	records := [][]string{
		{"Timestamp", "Scout Name", "Match Number", "Alliance", "Rank 1 (best)", "Rank 2", "Rank 3 (worst)"},
	}

	// Populate the CSV data.
	for _, ranking := range rawRankings {
		records = append(records, []string{
			// Most of the data is unused so we just fill in empty
			// strings.
			"", "", "", "",
			strconv.Itoa(int(ranking.Rank1)),
			strconv.Itoa(int(ranking.Rank2)),
			strconv.Itoa(int(ranking.Rank3)),
		})
	}

	dir, err := os.MkdirTemp("", "driver_ranking_eval")
	if err != nil {
		log.Println("Failed to create temporary driver_ranking_eval dir: ", err)
		return
	}
	defer os.RemoveAll(dir)

	inputCsvFile := filepath.Join(dir, "input.csv")
	outputCsvFile := filepath.Join(dir, "output.csv")

	if err := writeToCsv(inputCsvFile, records); err != nil {
		log.Println("Failed to write input CSV: ", err)
		return
	}

	// If the user didn't specify a script, use the default one.
	if scriptPath == "" {
		scriptPath, err = runfiles.Rlocation(DEFAULT_SCRIPT_PATH)
		if err != nil {
			log.Println("Failed to find runfiles entry for ", DEFAULT_SCRIPT_PATH, ": ", err)
			return
		}
	}

	// Run the analysis script.
	cmd := exec.Command(scriptPath, inputCsvFile, outputCsvFile)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		log.Println("Failed to run the driver ranking script: ", err)
		return
	}

	// Grab the output from the analysis script and insert it into the
	// database.
	outputRecords, err := readFromCsv(outputCsvFile)

	for _, record := range outputRecords {
		score, err := strconv.ParseFloat(record[1], 32)
		if err != nil {
			log.Println("Failed to parse score for team ", record[0], ": ", record[1], ": ", err)
			return
		}

		err = database.AddParsedDriverRanking(db.ParsedDriverRankingData{
			TeamNumber: record[0],
			Score:      float32(score),
		})
		if err != nil {
			log.Println("Failed to insert driver ranking score for team ", record[0], ": ", err)
			return
		}
	}
}
