package static

// A year agnostic way to serve static http files.
import (
	"crypto/sha256"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"
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

func MaybeNoCache(h http.Handler) http.Handler {
	fn := func(w http.ResponseWriter, r *http.Request) {
		// We force the browser not to cache index.html so that
		// browsers will notice when the bundle gets updated.
		if r.URL.Path == "/" || r.URL.Path == "/index.html" {
			for k, v := range noCacheHeaders {
				w.Header().Set(k, v)
			}
		}

		h.ServeHTTP(w, r)
	}

	return http.HandlerFunc(fn)
}

// Computes the sha256 of the specified file.
func computeSha256(path string) (string, error) {
	file, err := os.Open(path)
	if err != nil {
		return "", errors.New(fmt.Sprint("Failed to open ", path, ": ", err))
	}
	defer file.Close()

	hash := sha256.New()
	if _, err := io.Copy(hash, file); err != nil {
		return "", errors.New(fmt.Sprint("Failed to compute sha256 of ", path, ": ", err))
	}
	return fmt.Sprintf("%x", hash.Sum(nil)), nil
}

// Finds the checksums for all the files in the specified directory. This is a
// best effort only. If for some reason we fail to compute the checksum of
// something, we just move on.
func findAllFileShas(directory string) map[string]string {
	shaSums := make(map[string]string)

	// Find the checksums for all the files.
	err := filepath.Walk(directory, func(path string, info os.FileInfo, err error) error {
		if err != nil {
			log.Println("Walk() didn't want to deal with ", path, ":", err)
			return nil
		}
		if info.IsDir() {
			// We only care about computing checksums of files.
			// Ignore directories.
			return nil
		}
		hash, err := computeSha256(path)
		if err != nil {
			log.Println(err)
			return nil
		}
		shaSums[hash] = "/" + strings.TrimPrefix(path, directory)
		return nil
	})
	if err != nil {
		log.Fatal("Got unexpected error from Walk(): ", err)
	}

	return shaSums
}

func HandleShaUrl(directory string, h http.Handler) http.Handler {
	shaSums := findAllFileShas(directory)

	fn := func(w http.ResponseWriter, r *http.Request) {
		// We expect the path portion to look like this:
		// /sha256/<checksum>/path...
		// Splitting on / means we end up with this list:
		// [0] ""
		// [1] "sha256"
		// [2] "<checksum>"
		// [3-] path...
		parts := strings.Split(r.URL.Path, "/")
		if len(parts) < 4 {
			w.WriteHeader(http.StatusNotFound)
			return
		}
		if parts[0] != "" || parts[1] != "sha256" {
			// Something is fundamentally wrong. We told the
			// framework to only give is /sha256/ requests.
			log.Fatal("This handler should not be called for " + r.URL.Path)
		}
		hash := parts[2]
		if path, ok := shaSums[hash]; ok {
			// We found a file with this checksum. Serve that file.
			r.URL.Path = path
		} else {
			// No file with this checksum found.
			w.WriteHeader(http.StatusNotFound)
			return
		}

		h.ServeHTTP(w, r)
	}

	return http.HandlerFunc(fn)
}

// Serve pages in the specified directory.
func ServePages(scoutingServer server.ScoutingServer, directory string) {
	// Serve the / endpoint given a folder of pages.
	scoutingServer.Handle("/", MaybeNoCache(http.FileServer(http.Dir(directory))))

	// Also serve files in a checksum-addressable manner.
	scoutingServer.Handle("/sha256/", HandleShaUrl(directory, http.FileServer(http.Dir(directory))))
}
