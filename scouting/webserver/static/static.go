package static

// A year agnostic way to serve static http files.
import (
	"net/http"

	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
)

// Serve pages given a port, directory to serve from, and an channel to pass the errors back to the caller.
func ServePages(scoutingServer server.ScoutingServer, directory string) {
	// Serve the / endpoint given a folder of pages.
	scoutingServer.Handle("/", http.FileServer(http.Dir(directory)))
}
