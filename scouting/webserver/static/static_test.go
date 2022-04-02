package static

import (
	"fmt"
	"io/ioutil"
	"net/http"
	"testing"

	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
)

func expectEqual(t *testing.T, actual string, expected string) {
	if actual != expected {
		t.Error("Expected ", actual, " to equal ", expected)
	}
}

func TestServing(t *testing.T) {
	cases := []struct {
		// The path to request from the server.
		path string
		// The data that the server is expected to return at the
		// specified path.
		expectedData string
	}{
		{"/", "<h1>This is the index</h1>\n"},
		{"/root.txt", "Hello, this is the root page!"},
		{"/page.txt", "Hello from a page!"},
	}

	scoutingServer := server.NewScoutingServer()
	ServePages(scoutingServer, "test_pages")
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	// Go through all the test cases, and run them against the running webserver.
	for _, c := range cases {
		dataReceived := getData(c.path, t)
		expectEqual(t, dataReceived, c.expectedData)
	}
}

// Makes sure that requesting / sets the proper headers so it doesn't get
// cached.
func TestDisallowedCache(t *testing.T) {
	scoutingServer := server.NewScoutingServer()
	ServePages(scoutingServer, "test_pages")
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	resp, err := http.Get("http://localhost:8080/")
	if err != nil {
		t.Fatal("Failed to get data ", err)
	}
	expectEqual(t, resp.Header.Get("Expires"), "Thu, 01 Jan 1970 00:00:00 UTC")
	expectEqual(t, resp.Header.Get("Cache-Control"), "no-cache, private, max-age=0")
	expectEqual(t, resp.Header.Get("Pragma"), "no-cache")
	expectEqual(t, resp.Header.Get("X-Accel-Expires"), "0")
}

// Makes sure that requesting anything other than / doesn't set the "do not
// cache" headers.
func TestAllowedCache(t *testing.T) {
	scoutingServer := server.NewScoutingServer()
	ServePages(scoutingServer, "test_pages")
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	resp, err := http.Get("http://localhost:8080/root.txt")
	if err != nil {
		t.Fatalf("Failed to get data ", err)
	}
	expectEqual(t, resp.Header.Get("Expires"), "")
	expectEqual(t, resp.Header.Get("Cache-Control"), "")
	expectEqual(t, resp.Header.Get("Pragma"), "")
	expectEqual(t, resp.Header.Get("X-Accel-Expires"), "")
}

func TestSha256(t *testing.T) {
	scoutingServer := server.NewScoutingServer()
	ServePages(scoutingServer, "test_pages")
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	// Validate a valid checksum.
	dataReceived := getData("sha256/553b9b29647a112136986cf93c57b988d1f12dc43d3b774f14a24e58d272dbff/root.txt", t)
	expectEqual(t, dataReceived, "Hello, this is the root page!")

	// Make a request with an invalid checksum and make sure we get a 404.
	resp, err := http.Get("http://localhost:8080/sha256/0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef/root.txt")
	if err != nil {
		t.Fatal("Failed to get data ", err)
	}
	expectEqual(t, resp.Status, "404 Not Found")
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
		t.Fatal("Received a status code other than 200:", resp.Status)
	}
	// Read the response body.
	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		t.Fatalf("Failed to read body")
	}
	// Decode the body and return it.
	return string(body)
}
