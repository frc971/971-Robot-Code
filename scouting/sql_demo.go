package main

import (
	"database/sql"
	"fmt"
	"strconv"

	_ "github.com/mattn/go-sqlite3"
)

func main() {
	var alliancecolour string
	var teamnumber int
	var id int

	database, _ := sql.Open("sqlite3", "./bogo.db")
	statement, _ := database.Prepare("DROP TABLE IF EXISTS robots")
	statement.Exec()
	statement, _ = database.Prepare("CREATE TABLE robots (id INTEGER PRIMARY KEY, alliancecolour TEXT, teamnumber TEXT)")
	statement.Exec()
	for i := 0; i < 6; i++ {
		if i < 3 {
			alliancecolour = "red" + strconv.Itoa(i+1)
		} else if i >= 3 {
			alliancecolour = "blue" + strconv.Itoa(i-2)
		}
		statement, _ = database.Prepare("INSERT INTO robots (alliancecolour, teamnumber) VALUES (?, ?)")
		statement.Exec(alliancecolour, "971")
	}
	rows, _ := database.Query("SELECT id, alliancecolour, teamnumber FROM robots")

	for rows.Next() {
		rows.Scan(&id, &alliancecolour, &teamnumber)
		fmt.Println(alliancecolour + ": " + strconv.Itoa(teamnumber))
	}
}
