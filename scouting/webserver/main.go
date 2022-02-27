package main

import (
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	"github.com/frc971/971-Robot-Code/scouting/webserver/static"
)

func getDefaultDatabasePath() string {
	// If using `bazel run`, let's create the database in the root of the
	// workspace.
	workspaceDir := os.Getenv("BUILD_WORKSPACE_DIRECTORY")
	if workspaceDir != "" {
		return filepath.Join(workspaceDir, "scouting.db")
	}
	// If we're inside a `bazel test`, then we will create the database in
	// a temporary directory.
	testTempDir := os.Getenv("TEST_TMPDIR")
	if testTempDir != "" {
		tempDir, err := ioutil.TempDir(testTempDir, "db")
		if err != nil {
			log.Fatal("Failed to create temporary directory in TEST_TMPDIR: ", err)
		}
		return filepath.Join(tempDir, "scouting.db")
	}
	return filepath.Join(".", "scouting.db")
}

func main() {
	portPtr := flag.Int("port", 8080, "The port number to bind to.")
	dirPtr := flag.String("directory", ".", "The directory to serve at /.")
	dbPathPtr := flag.String("database", getDefaultDatabasePath(), "The path to the database.")
	flag.Parse()

	database, err := db.NewDatabase(*dbPathPtr)
	if err != nil {
		log.Fatal("Failed to connect to database: ", err)
	}

	scoutingServer := server.NewScoutingServer()
	static.ServePages(scoutingServer, *dirPtr)
	requests.HandleRequests(database, scoutingServer)
	scoutingServer.Start(*portPtr)
	fmt.Println("Serving", *dirPtr, "on port", *portPtr)

	// Block until the user hits Ctrl-C.
	sigint := make(chan os.Signal, 1)
	signal.Notify(sigint, syscall.SIGINT)
	fmt.Println("Waiting for CTRL-C or SIGINT.")
	<-sigint

	fmt.Println("Shutting down.")
	scoutingServer.Stop()
	fmt.Println("Successfully shut down.")
}
