// This binary lets users interact with the scouting web server in order to
// debug it. Run with `--help` to see all the options.

package main

import (
	"flag"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"path/filepath"

	"github.com/davecgh/go-spew/spew"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests/debug"
)

// Returns the absolute path of the specified path. This is an unwrapped
// version of `filepath.Abs`.
func absPath(path string) string {
	result, err := filepath.Abs(path)
	if err != nil {
		log.Fatal("Failed to determine absolute path for ", path, ": ", err)
	}
	return result
}

// Parses the specified JSON file into a binary version (i.e. serialized
// flatbuffer). This uses the `flatc` binary and the JSON's corresponding
// `.fbs` file.
func parseJson(fbsPath string, jsonPath string) []byte {
	// Work inside a temporary directory since `flatc` doesn't allow us to
	// customize the name of the output file.
	dir, err := ioutil.TempDir("", "webserver_debug_cli")
	if err != nil {
		log.Fatal("Failed to create temporary directory: ", err)
	}
	defer os.RemoveAll(dir)

	// Turn these paths absolute so that it everything still works from
	// inside the temporary directory.
	absFlatcPath := absPath("external/com_github_google_flatbuffers/flatc")
	absFbsPath := absPath(fbsPath)

	// Create a symlink to the .fbs file so that the output filename that
	// `flatc` generates is predictable. I.e. `fb.json` gets serialized
	// into `fb.bin`.
	jsonSymlink := filepath.Join(dir, "fb.json")
	os.Symlink(jsonPath, jsonSymlink)

	// Execute the `flatc` command.
	flatcCommand := exec.Command(absFlatcPath, "--binary", absFbsPath, jsonSymlink)
	flatcCommand.Dir = dir
	output, err := flatcCommand.CombinedOutput()
	if err != nil {
		log.Fatal("Failed to execute flatc: ", err, ": ", string(output))
	}

	// Read the serialized flatbuffer and return it.
	binaryPath := filepath.Join(dir, "fb.bin")
	binaryFb, err := os.ReadFile(binaryPath)
	if err != nil {
		log.Fatal("Failed to read flatc output ", binaryPath, ": ", err)
	}
	return binaryFb
}

func maybePerformRequest[T interface{}](fbName, fbsPath, requestJsonPath, address string, requester func(string, []byte) (*T, error)) {
	if requestJsonPath != "" {
		log.Printf("Sending %s to %s", fbName, address)
		binaryRequest := parseJson(fbsPath, requestJsonPath)
		response, err := requester(address, binaryRequest)
		if err != nil {
			log.Fatalf("Failed %s: %v", fbName, err)
		}
		spew.Dump(*response)
	}
}

func main() {
	// Parse command line arguments.
	indentPtr := flag.String("indent", " ",
		"The indentation to use for the result dumping. Default is a space.")
	addressPtr := flag.String("address", "http://localhost:8080",
		"The end point where the server is listening.")
	submitDataScoutingPtr := flag.String("submitDataScouting", "",
		"If specified, parse the file as a SubmitDataScouting JSON request.")
	submitDriverRankingPtr := flag.String("submitDriverRanking", "",
		"If specified, parse the file as a submitDriverRanking JSON request.")
	submitNotesPtr := flag.String("submitNotes", "",
		"If specified, parse the file as a submitNotes JSON request.")
	requestAllMatchesPtr := flag.String("requestAllMatches", "",
		"If specified, parse the file as a RequestAllMatches JSON request.")
	requestDataScoutingPtr := flag.String("requestDataScouting", "",
		"If specified, parse the file as a RequestDataScouting JSON request.")
	requestAllDriverRankingsPtr := flag.String("requestAllDriverRankings", "",
		"If specified, parse the file as a requestAllDriverRankings JSON request.")
	requestAllNotesPtr := flag.String("requestAllNotes", "",
		"If specified, parse the file as a requestAllNotes JSON request.")
	refreshMatchListPtr := flag.String("refreshMatchList", "",
		"If specified, parse the file as a RefreshMatchList JSON request.")
	flag.Parse()

	spew.Config.Indent = *indentPtr

	// Disable pointer addresses. They're not useful for our purposes.
	spew.Config.DisablePointerAddresses = true

	// Handle the actual arguments.
	maybePerformRequest(
		"SubmitDataScouting",
		"scouting/webserver/requests/messages/submit_data_scouting.fbs",
		*submitDataScoutingPtr,
		*addressPtr,
		debug.SubmitDataScouting)

	maybePerformRequest(
		"submitNotes",
		"scouting/webserver/requests/messages/submit_notes.fbs",
		*submitNotesPtr,
		*addressPtr,
		debug.SubmitNotes)

	maybePerformRequest(
		"submitDriverRanking",
		"scouting/webserver/requests/messages/submit_driver_ranking.fbs",
		*submitDriverRankingPtr,
		*addressPtr,
		debug.SubmitDriverRanking)

	maybePerformRequest(
		"RequestAllMatches",
		"scouting/webserver/requests/messages/request_all_matches.fbs",
		*requestAllMatchesPtr,
		*addressPtr,
		debug.RequestAllMatches)

	maybePerformRequest(
		"RequestDataScouting",
		"scouting/webserver/requests/messages/request_data_scouting.fbs",
		*requestDataScoutingPtr,
		*addressPtr,
		debug.RequestDataScouting)

	maybePerformRequest(
		"requestAllDriverRankings",
		"scouting/webserver/requests/messages/request_all_driver_rankings.fbs",
		*requestAllDriverRankingsPtr,
		*addressPtr,
		debug.RequestAllDriverRankings)

	maybePerformRequest(
		"requestAllNotes",
		"scouting/webserver/requests/messages/request_all_notes.fbs",
		*requestAllNotesPtr,
		*addressPtr,
		debug.RequestAllNotes)

	maybePerformRequest(
		"RefreshMatchList",
		"scouting/webserver/requests/messages/refresh_match_list.fbs",
		*refreshMatchListPtr,
		*addressPtr,
		debug.RefreshMatchList)
}
