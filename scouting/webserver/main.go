package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"syscall"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	"github.com/frc971/971-Robot-Code/scouting/webserver/static"
)

func main() {
	portPtr := flag.Int("port", 8080, "The port number to bind to.")
	dirPtr := flag.String("directory", ".", "The directory to serve at /.")
	flag.Parse()

	database, err := db.NewDatabase()
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
