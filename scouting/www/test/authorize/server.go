package main

import (
	"flag"
	"fmt"
	"log"
	"net/http"
)

var authorized = false
var authorizedCounter = 0

// Special handler for responding to /submit requests.
func handleSubmission(w http.ResponseWriter, r *http.Request) {
	if !authorized {
		log.Println("Replying with 'Unauthorized'.")
		w.WriteHeader(http.StatusUnauthorized)
		w.Write([]byte{})
		return
	}

	log.Println("Replying with success.")
	authorizedCounter += 1
	w.Write([]byte(fmt.Sprintf("Successful submission %d!", authorizedCounter)))
}

// Default handler for all other requests.
func createDefaultHandler(directory string) http.HandlerFunc {
	handler := http.FileServer(http.Dir(directory))

	fn := func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/authorize" {
			authorized = true
		}
		handler.ServeHTTP(w, r)
	}

	return http.HandlerFunc(fn)
}

func main() {
	directoryPtr := flag.String("directory", ".", "The directory to serve")
	flag.Parse()

	http.HandleFunc("/", createDefaultHandler(*directoryPtr))
	http.HandleFunc("/submit", handleSubmission)

	fmt.Println("Server listening on port 8000...")
	http.ListenAndServe(":8000", nil)
}
