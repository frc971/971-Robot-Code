// This binary starts up a postgres server in the background.
//
// The server will be reachable at localhost:5432 with username and password of
// "test" and "password", respectively.

package main

import (
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"

	"github.com/bazelbuild/rules_go/go/runfiles"
)

func check(err error, message string) {
	if err != nil {
		log.Fatal(message, ": ", err)
	}
}

// Loads the runfiles environment variables so that subprocesses can access
// their own runfiles properly.
func loadRunfilesEnvironment() {
	values, err := runfiles.Env()
	check(err, "Failed to retrieve runfiles environment")
	for _, value := range values {
		parts := strings.SplitN(value, "=", 2)
		if len(parts) != 2 {
			log.Fatalf("Failed to split \"%s\" on \"=\".", value)
		}
		err = os.Setenv(parts[0], parts[1])
		check(err, "Failed to set environment "+value)
	}
}

func getRunfile(path string) string {
	result, err := runfiles.Rlocation(path)
	check(err, fmt.Sprint("Failed to find runfile path for ", path))
	return result
}

func copyTestConfig(sourceFile, destinationFile string, port int, socketDir string) {
	input, err := ioutil.ReadFile(sourceFile)
	check(err, fmt.Sprint("Failed to read from ", sourceFile))

	config := strings.ReplaceAll(string(input), "{POSTGRES_PORT}", strconv.Itoa(port))
	config = strings.ReplaceAll(config, "{POSTGRES_UNIX_SOCKET_DIR}", socketDir)

	err = ioutil.WriteFile(destinationFile, []byte(config), 0644)
	check(err, fmt.Sprint("Failed to write to ", destinationFile))
}

func main() {
	portPtr := flag.Int("port", 5432, "The port number to bind to.")
	flag.Parse()

	loadRunfilesEnvironment()

	// Set up temporary directories for postgres. These cannot live in
	// TEST_TMPDIR because socket paths have a maximum of 107 bytes.
	tmpdir, err := ioutil.TempDir("", "testdb_server")
	check(err, "Failed to create tmpdir")
	defer os.RemoveAll(tmpdir)

	passwordPath := filepath.Join(tmpdir, "password.txt")
	d1 := []byte("password\n")
	err = os.WriteFile(passwordPath, d1, 0644)
	check(err, "Failed to create password file")

	dbDir := filepath.Join(tmpdir, "db")
	initdb := exec.Command(
		getRunfile("postgresql_amd64/initdb"),
		"-D", dbDir,
		"--username=test", "--pwfile="+passwordPath)
	initdb.Stdout = os.Stdout
	initdb.Stderr = os.Stderr
	err = initdb.Run()
	check(err, "Failed to run initdb")

	// Set up a directory for postgres to put its socket file.
	socketDir := filepath.Join(tmpdir, "postgres_socket")
	err = os.MkdirAll(socketDir, 0755)
	check(err, "Failed to create socket directory "+socketDir)

	copyTestConfig(
		getRunfile("org_frc971/scouting/db/testdb_server/postgres_test.conf"),
		filepath.Join(dbDir, "postgresql.conf"),
		*portPtr,
		socketDir)

	log.Println("Starting up postgres.")
	server := exec.Command(getRunfile("postgresql_amd64/postgres"), "-D", dbDir)
	server.Stdout = os.Stdout
	server.Stderr = os.Stderr
	err = server.Start()
	check(err, "Failed to run postgres")
	defer func() {
		log.Println("Shutting down postgres")
		server.Process.Signal(os.Interrupt)
		server.Process.Wait()
	}()

	// Block until the user hits Ctrl-C.
	sigint := make(chan os.Signal, 1)
	signal.Notify(sigint, syscall.SIGTERM)
	signal.Notify(sigint, syscall.SIGINT)
	fmt.Println("Waiting for CTRL-C or SIGINT.")
	<-sigint
}
