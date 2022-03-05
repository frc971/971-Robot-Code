package scraping

// A library to grab match data from The Blue Alliance.
import (
	"encoding/json"
	"errors"
	"fmt"
	"io/ioutil"
	"net/http"
	"os"
	"strconv"
)

// Stores the TBA API key to access the API.
type scrapingConfig struct {
	ApiKey  string `json:"api_key"`
	BaseUrl string `json:"base_url"`
}

// Takes in year and FIRST event code and returns all matches in that event according to TBA.
// Also takes in a file path to the JSON config file that contains your TBA API key.
// It defaults to <workspace root>/config.json
// the config is expected to have the following contents:
//{
//    api_key:"myTBAapiKey"
//}
func AllMatches(year int32, eventCode, configPath string) ([]Match, error) {
	if configPath == "" {
		configPath = os.Getenv("BUILD_WORKSPACE_DIRECTORY") + "/scouting_config.json"
	}

	// Takes the filepath and grabs the api key from the json.
	content, err := ioutil.ReadFile(configPath)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to open config at ", configPath, ": ", err))
	}
	// Parses the JSON parameters into a struct.
	var config scrapingConfig
	if err := json.Unmarshal([]byte(content), &config); err != nil {
		return nil, errors.New(fmt.Sprint("Failed to parse config file as JSON: ", err))
	}

	// Perform some basic validation on the data.
	if config.ApiKey == "" {
		return nil, errors.New("Missing 'api_key' in config JSON.")
	}
	if config.BaseUrl == "" {
		config.BaseUrl = "https://www.thebluealliance.com"
	}

	// Create the TBA event key for the year and event code.
	eventKey := strconv.Itoa(int(year)) + eventCode

	// Create a get request for the match info.
	req, err := http.NewRequest("GET", config.BaseUrl+"/api/v3/event/"+eventKey+"/matches", nil)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to build http request: ", err))
	}

	// Add the auth key header to the request.
	req.Header.Add("X-TBA-Auth-Key", config.ApiKey)

	// Make the API request.
	client := &http.Client{}
	resp, err := client.Do(req)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to make TBA API request: ", err))
	}

	defer resp.Body.Close()
	if resp.StatusCode != 200 {
		return nil, errors.New(fmt.Sprint("Got unexpected status code from TBA API request: ", resp.Status))
	}

	// Get all bytes from response body.
	bodyBytes, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to read TBA API response: ", err))
	}

	var matches []Match
	// Unmarshal json into go usable format.
	if err := json.Unmarshal([]byte(bodyBytes), &matches); err != nil {
		return nil, errors.New(fmt.Sprint("Failed to parse JSON received from TBA: ", err))
	}

	return matches, nil
}
