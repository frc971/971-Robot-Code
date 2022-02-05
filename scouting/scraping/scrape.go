package scraping

// A library to grab match data from The Blue Alliance.
import (
	"encoding/json"
	"errors"
	"io/ioutil"
	"log"
	"net/http"
	"os"
)

// Stores the TBA API key to access the API.
type params struct {
	ApiKey string `json:"api_key"`
}

// Takes in year and FIRST event code and returns all matches in that event according to TBA.
// Also takes in a file path to the JSON config file that contains your TBA API key.
// It defaults to <workspace root>/config.json
// the config is expected to have the following contents:
//{
//    api_key:"myTBAapiKey"
//}
func AllMatches(year, eventCode, filePath string) ([]Match, error) {
	if filePath == "" {
		filePath = os.Getenv("BUILD_WORKSPACE_DIRECTORY") + "/scouting_config.json"
	}
	// Takes the filepath and grabs the api key from the json.
	content, err := ioutil.ReadFile(filePath)
	if err != nil {
		log.Fatal(err)
	}
	// Parses the JSON parameters into a struct.
	var passed_params params
	error := json.Unmarshal([]byte(content), &passed_params)
	if error != nil {
		log.Fatalf("You forgot to add the api_key parameter in the json file")
		log.Fatalf("%s", err)
	}

	// Create the TBA event key for the year and event code.
	eventKey := year + eventCode

	// Create the client for HTTP requests.
	client := &http.Client{}

	// Create a get request for the match info.
	req, err := http.NewRequest("GET", "https://www.thebluealliance.com/api/v3/event/"+eventKey+"/matches", nil)

	if err != nil {
		return nil, errors.New("failed to build http request")
	}

	// Add the auth key header to the request.
	req.Header.Add("X-TBA-Auth-Key", passed_params.ApiKey)

	// Make the API request
	resp, err := client.Do(req)

	if err != nil {
		return nil, err
	}

	if resp.Status != "200 OK" {
		return nil, errors.New("Recieved a status of " + resp.Status + " expected : 200 OK")
	}

	// Wait until the response is done.
	defer resp.Body.Close()

	// Get all bytes from response body.
	bodyBytes, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		return nil, errors.New("failed to read response body with error :" + err.Error())
	}

	var matches []Match
	// Unmarshal json into go usable format.
	jsonError := json.Unmarshal([]byte(bodyBytes), &matches)
	if jsonError != nil {
		return nil, errors.New("failed to unmarshal json recieved from TBA")
	}

	return matches, nil
}
