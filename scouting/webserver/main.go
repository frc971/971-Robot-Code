package main

import (
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/frc971/971-Robot-Code/scouting/db"
	"github.com/frc971/971-Robot-Code/scouting/scraping"
	"github.com/frc971/971-Robot-Code/scouting/webserver/requests"
	"github.com/frc971/971-Robot-Code/scouting/webserver/server"
	"github.com/frc971/971-Robot-Code/scouting/webserver/static"
)

type DatabaseConfig struct {
	Username string `json:"username"`
	Password string `json:"password"`
	Port     int    `json:"port"`
}

func readDatabaseConfig(configPath string) (*DatabaseConfig, error) {
	content, err := ioutil.ReadFile(configPath)
	if err != nil {
		return nil, errors.New(fmt.Sprint("Failed to open database config at ", configPath, ": ", err))
	}

	var config DatabaseConfig
	if err := json.Unmarshal([]byte(content), &config); err != nil {
		return nil, errors.New(fmt.Sprint("Failed to parse database config file as JSON: ", err))
	}

	return &config, nil
}

// Gets the default port to use for the webserver. If wrapped by
// apache_wrapper(), we use the port dictated by the wrapper.
func getDefaultPort() int {
	port_str := os.Getenv("APACHE_WRAPPED_PORT")
	if port_str != "" {
		port, err := strconv.Atoi(port_str)
		if err != nil {
			log.Fatalf("Failed to parse \"%s\" as integer: %v", port_str, err)
		}
		return port
	}

	return 8080
}

func main() {
	portPtr := flag.Int("port", getDefaultPort(), "The port number to bind to.")
	dirPtr := flag.String("directory", ".", "The directory to serve at /.")
	dbConfigPtr := flag.String("db_config", "",
		"The postgres database JSON config. It needs the following keys: "+
			"\"username\", \"password\", and \"port\". "+
			"This option cannot be used in conjunction with -testdb_port.")
	testDbPortPtr := flag.Int("testdb_port", 0,
		"If set, connects to an instance of //scouting/db/testdb_server at the "+
			"specified port. This option cannot be used in conjunction with "+
			"-db_config.")
	dbConnectRetries := flag.Int("db_retries", 5,
		"The number of seconds to retry connecting to the database on startup.")
	blueAllianceConfigPtr := flag.String("tba_config", "",
		"The path to your The Blue Alliance JSON config. "+
			"It needs an \"api_key\" field with your TBA API key. "+
			"Optionally, it can have a \"base_url\" field with the TBA API base URL.")
	flag.Parse()

	// Set up the database config. It's either specified via a JSON file on
	// disk (-db_config) or we can generate an in-memory config when the
	// user wants to connect to a `testdb_server` instance (-testdb_port).
	var dbConfig *DatabaseConfig
	var err error
	if *dbConfigPtr != "" {
		if *testDbPortPtr != 0 {
			log.Fatal("Cannot specify both -db_config and -testdb_port. Choose one.")
		}
		dbConfig, err = readDatabaseConfig(*dbConfigPtr)
		if err != nil {
			log.Fatal("Failed to read ", *dbConfigPtr, ": ", err)
		}
	} else if *testDbPortPtr != 0 {
		dbConfig = &DatabaseConfig{
			Username: "test",
			Password: "password",
			Port:     *testDbPortPtr,
		}
	} else {
		log.Fatal("Must specify one of -db_config and -testdb_port. See " +
			"https://github.com/frc971/971-Robot-Code/blob/master/scouting/README.md " +
			"for more information.")
	}

	// Connect to the database. It may still be starting up so we do a
	// bunch of retries here. The number of retries can be set on the
	// command line.
	var database *db.Database
	for i := 0; i < *dbConnectRetries*10; i++ {
		database, err = db.NewDatabase(dbConfig.Username, dbConfig.Password, dbConfig.Port)
		if err == nil {
			break
		}
		if i%10 == 0 {
			log.Println("Failed to connect to the database. Retrying in a bit.")
		}
		time.Sleep(1 * time.Millisecond)
	}
	if err != nil {
		log.Fatal("Failed to connect to database: ", err)
	}
	defer database.Close()

	scrapeMatchList := func(year int32, eventCode string) ([]scraping.Match, error) {
		if *blueAllianceConfigPtr == "" {
			return nil, errors.New("Cannot scrape TBA's match list without a config file.")
		}
		return scraping.AllMatches(year, eventCode, *blueAllianceConfigPtr)
	}

	scoutingServer := server.NewScoutingServer()
	static.ServePages(scoutingServer, *dirPtr)
	requests.HandleRequests(database, scrapeMatchList, scoutingServer)
	scoutingServer.Start(*portPtr)
	fmt.Println("Serving", *dirPtr, "on port", *portPtr)

	// Block until the user hits Ctrl-C.
	sigint := make(chan os.Signal, 1)
	signal.Notify(sigint, syscall.SIGINT)
	signal.Notify(sigint, syscall.SIGTERM)
	fmt.Println("Waiting for CTRL-C or SIGTERM.")
	<-sigint

	fmt.Println("Shutting down.")
	scoutingServer.Stop()
	fmt.Println("Successfully shut down.")
}
