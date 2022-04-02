package static

// A year agnostic way to serve static http files.
import (
	"net/http"
	"time"

	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
)

// We want the static files (which include JS that is modified over time), to not be cached.
// This ensures users get updated versions when uploaded to the server.
// Based on https://stackoverflow.com/a/33881296, this disables cache for most browsers.
var epoch = time.Unix(0, 0).Format(time.RFC1123)

var noCacheHeaders = map[string]string{
	"Expires":         epoch,
	"Cache-Control":   "no-cache, private, max-age=0",
	"Pragma":          "no-cache",
	"X-Accel-Expires": "0",
}

func NoCache(h http.Handler) http.Handler {
	fn := func(w http.ResponseWriter, r *http.Request) {
		for k, v := range noCacheHeaders {
			w.Header().Set(k, v)
		}

		h.ServeHTTP(w, r)
	}

	return http.HandlerFunc(fn)
}

// Serve pages given a port, directory to serve from, and an channel to pass the errors back to the caller.
func ServePages(scoutingServer server.ScoutingServer, directory string) {
	// Serve the / endpoint given a folder of pages.
	scoutingServer.Handle("/", NoCache(http.FileServer(http.Dir(directory))))
}
