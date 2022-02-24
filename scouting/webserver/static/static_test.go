package static

import (
	"fmt"
	"io/ioutil"
	"net/http"
	"testing"

	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
)

func TestServing(t *testing.T) {
	cases := []struct {
		// The path to request from the server.
		path string
		// The data that the server is expected to return at the
		// specified path.
		expectedData string
	}{
		{"/root.txt", "Hello, this is the root page!"},
		{"/page.txt", "Hello from a page!"},
	}

	scoutingServer := server.NewScoutingServer()
	ServePages(scoutingServer, "test_pages")
	scoutingServer.Start(8080)

	// Go through all the test cases, and run them against the running webserver.
	for _, c := range cases {
		dataReceived := getData(c.path, t)
		if dataReceived != c.expectedData {
			t.Errorf("Got %q, but expected %q", dataReceived, c.expectedData)
		}
	}

	scoutingServer.Stop()
}

// Retrieves the data at the specified path. If an error occurs, the test case
// is terminated and failed.
func getData(path string, t *testing.T) string {
	resp, err := http.Get(fmt.Sprintf("http://localhost:8080/%s", path))
	if err != nil {
		t.Fatalf("Failed to get data ", err)
	}
	// Error out if the return status is anything other than 200 OK.
	if resp.Status != "200 OK" {
		t.Fatalf("Received a status code other than 200")
	}
	// Read the response body.
	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		t.Fatalf("Failed to read body")
	}
	// Decode the body and return it.
	return string(body)
}
