// This file implements a web server that can be used by the main scouting
// application. It can also be used in unit tests for individual components of
// the web server.

package server

import (
	"context"
	"fmt"
	"log"
	"net"
	"net/http"
	"time"
)

type ScoutingServer interface {
	// Add a handler for a particular path. See upstream docs for this:
	// https://pkg.go.dev/net/http#ServeMux.Handle
	Handle(string, http.Handler)
	// Add a handler function for a particular path. See upstream docs:
	// https://pkg.go.dev/net/http#ServeMux.HandleFunc
	HandleFunc(string, func(http.ResponseWriter, *http.Request))
	// Starts the server on the specified port. Handlers cannot be added
	// once this function is called.
	Start(int)
	// Stops the server.
	Stop()
}

// This is a collection of data we'll need to implement the ScoutingServer
// interface.
type scoutingServer struct {
	mux        *http.ServeMux
	httpServer *http.Server
	doneChan   <-chan bool
	// Denotes whether the server has ever been Start()ed before.
	started bool
	// Denotes whether or not the server is currently running.
	running bool
}

// Instantiates a new ScoutingServer.
func NewScoutingServer() ScoutingServer {
	return &scoutingServer{
		mux:        http.NewServeMux(),
		httpServer: nil,
		doneChan:   nil,
		started:    false,
		running:    false,
	}
}

func (server *scoutingServer) Handle(path string, handler http.Handler) {
	if server.started {
		log.Fatal("Cannot add handlers once server has started.")
	}
	server.mux.Handle(path, handler)
}

func (server *scoutingServer) HandleFunc(path string, handler func(http.ResponseWriter, *http.Request)) {
	if server.started {
		log.Fatal("Cannot add handlers once server has started.")
	}
	server.mux.HandleFunc(path, handler)
}

func (server *scoutingServer) Start(port int) {
	if server.started {
		log.Fatal("Cannot Start() a server a second time.")
	}
	server.started = true
	server.running = true

	addressStr := fmt.Sprintf(":%d", port)
	server.httpServer = &http.Server{
		Addr:    addressStr,
		Handler: server.mux,
	}

	doneChan := make(chan bool, 1)
	server.doneChan = doneChan

	// Start the server in the background since the ListenAndServe() call
	// blocks.
	go func() {
		if err := server.httpServer.ListenAndServe(); err != http.ErrServerClosed {
			log.Fatalf("Error calling ListenAndServe(): %v", err)
		}
		close(doneChan)
	}()

	// Wait until the server is ready.
	for {
		dial, err := net.Dial("tcp", addressStr)
		if err != nil {
			time.Sleep(100 * time.Millisecond)
		} else {
			dial.Close()
			break
		}
	}
}

func (server *scoutingServer) Stop() {
	if !server.running {
		log.Fatal("Cannot Stop() a server that's not running.")
	}
	server.running = false

	if err := server.httpServer.Shutdown(context.Background()); err != nil {
		log.Fatalf("Error shutting down the server: %v", err)
	}
	<-server.doneChan
}
