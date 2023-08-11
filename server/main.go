//////////////////////////////////////////////////////////////////////
// Salt Sensor Server

package main

import (
	"database/sql"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"net/http"
	"net/url"
	"os"
	"strconv"
	"strings"

	"github.com/go-sql-driver/mysql"
)

//////////////////////////////////////////////////////////////////////
// HTTP response body

type Response struct {
	Status string   `json:"status"`
	Info   []string `json:"info"`
}

//////////////////////////////////////////////////////////////////////
// Database credentials

type Credentials struct {
	Username string `json:"username"`
	Password string `json:"password"`
}

//////////////////////////////////////////////////////////////////////
// http ResponseWriter wrapper for UFCS

type Responder struct {
	writer http.ResponseWriter
}

//////////////////////////////////////////////////////////////////////

type Loggers struct {
	Debug   *log.Logger
	Verbose *log.Logger
	Info    *log.Logger
	Warning *log.Logger
	Error   *log.Logger
	Fatal   *log.Logger
	None    *log.Logger
}

//////////////////////////////////////////////////////////////////////

var db_credentials Credentials
var db_DSNString string

var Log Loggers

//////////////////////////////////////////////////////////////////////

func set_log_level(log_level_name string) (e error) {

	const num_log_levels = 7

	log_names := [num_log_levels]string{
		"debug",
		"verbose",
		"info",
		"warning",
		"error",
		"fatal",
		"none",
	}

	log_writers := [num_log_levels]io.Writer{
		os.Stdout,
		os.Stdout,
		os.Stdout,
		os.Stdout,
		os.Stderr,
		os.Stderr,
		io.Discard,
	}

	var log_level = 2 // level is info by default

	found := true

	if len(log_level_name) != 0 {

		found = false

		for i, name := range log_names {

			if strings.EqualFold(log_level_name, name) {

				log_level = i
				found = true
				break
			}
		}
	}

	var logs []*log.Logger = make([]*log.Logger, num_log_levels)

	for i := 0; i < log_level; i++ {
		logs[i] = log.New(io.Discard, "", 0)
	}

	for i := log_level; i < num_log_levels; i++ {
		name := strings.ToUpper(log_names[i]) + strings.Repeat(" ", 8-len(log_names[i])) + ":"
		logs[i] = log.New(log_writers[i], name, log.Ldate|log.Ltime|log.Lmicroseconds)
	}

	Log = Loggers{}

	Log.Debug = logs[0]
	Log.Verbose = logs[1]
	Log.Info = logs[2]
	Log.Warning = logs[3]
	Log.Error = logs[4]
	Log.Fatal = logs[5]
	Log.None = logs[6]

	if !found {
		return fmt.Errorf("unknown log level \"%s\"", log_level_name)
	}

	return nil
}

//////////////////////////////////////////////////////////////////////

func load_credentials(credentials_filename string) error {

	if len(credentials_filename) == 0 {

		return errors.New("missing credentials filename")
	}

	var err error
	var content []byte

	if content, err = os.ReadFile(credentials_filename); err != nil {

		return fmt.Errorf("loading credentials, can't %s", err)
	}

	if err = json.Unmarshal(content, &db_credentials); err != nil {

		return fmt.Errorf("Error parsing %s: %s", credentials_filename, err)
	}

	Log.Verbose.Printf("Loaded credentials file %s", credentials_filename)
	return nil
}

//////////////////////////////////////////////////////////////////////

func get_json(r Response) []byte {

	if s, err := json.MarshalIndent(r, "", "  "); err == nil {
		return s
	}

	return []byte(`{"huh":"say wha?"}`)
}

//////////////////////////////////////////////////////////////////////

func (w Responder) Respond(status int, status_text string, info string, args ...any) {

	w.writer.WriteHeader(status)
	w.writer.Write(get_json(Response{Status: status_text, Info: []string{fmt.Sprintf(info, args...)}}))
}

//////////////////////////////////////////////////////////////////////

func (w Responder) Error(status int, info string, args ...any) {

	w.Respond(status, "error", info, args...)
}

//////////////////////////////////////////////////////////////////////

func (w Responder) OK(status int, info string, args ...any) {

	w.Respond(status, "OK", info, args...)
}

//////////////////////////////////////////////////////////////////////

func parse_param(name string, base int, bits int, values *url.Values) (uint64, error) {

	param := (*values)[name]

	if len(param) < 1 {
		return 0, fmt.Errorf("missing parameter %s", name)
	}

	if len(param) > 1 {
		return 0, fmt.Errorf("duplicate parameter %s", name)
	}

	v, err := strconv.ParseUint(param[0], base, bits)

	if err != nil {
		return 0, fmt.Errorf("bad value for '%s' (%s)", name, err.Error())
	}
	return v, nil
}

//////////////////////////////////////////////////////////////////////

func http_reading_handler(w http.ResponseWriter, r *http.Request) {

	var response Responder = Responder{writer: w}

	w.Header().Set("Content-Type", "application/json")

	// initial checks

	if r.Method != "GET" {
		response.Error(http.StatusMethodNotAllowed, "Method %s not allowed", r.Method)
		return
	}

	Log.Verbose.Printf("GET %s", r.URL)

	// parse the query string

	var err error

	var errors = make([]string, 0)

	var vbat uint64
	var distance uint64
	var device uint64
	var flags uint64

	q := r.URL.Query()

	if vbat, err = parse_param("vbat", 10, 16, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if distance, err = parse_param("distance", 10, 16, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if device, err = parse_param("device", 16, 48, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if flags, err = parse_param("flags", 10, 16, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if len(errors) != 0 {
		w.WriteHeader(http.StatusBadRequest)
		w.Write(get_json(Response{Status: "error", Info: errors}))
		return
	}

	Log.Debug.Printf("vbat: %d, distance: %d, flags: %d, device: %012x", vbat, distance, flags, device)

	// open the database

	var db *sql.DB

	db, err = sql.Open("mysql", db_DSNString)
	if err != nil {
		response.Error(http.StatusInternalServerError, "Error opening database: %s", err.Error())
		return
	}

	Log.Debug.Println("Database opened")

	defer db.Close()

	// get device id (could be a new one or an existing one)

	device_address := fmt.Sprintf("%012x", device)

	Log.Debug.Printf("Get device id for %s", device_address)

	_, err = db.Exec(`INSERT INTO devices (device_address)
						SELECT * FROM (SELECT ?) AS tmp
						WHERE NOT EXISTS
						(SELECT * FROM devices WHERE device_address=?) LIMIT 1;`, device_address, device_address)
	if err != nil {

		response.Error(http.StatusInternalServerError, "Database error adding device %s: %s", device_address, err.Error())
		return
	}

	var device_id int64

	if err := db.QueryRow(`SELECT device_id FROM devices WHERE device_address = ?;`, device_address).Scan(&device_id); err != nil {

		response.Error(http.StatusInternalServerError, "Database error getting device_id for %s: %s", device_address, err.Error())
		return
	}

	Log.Debug.Printf("Device ID %d", device_id)

	// add the reading

	res, err := db.Exec(`INSERT INTO readings (device_id, reading_vbat, reading_distance, reading_flags, reading_timestamp)
                         VALUES (?,?,?,?,CURRENT_TIMESTAMP());`, device_id, vbat, distance, flags)

	if err != nil {

		response.Error(http.StatusInternalServerError, "Error adding reading to database(1): %s", err.Error())
		return
	}

	var reading_id int64

	reading_id, err = res.LastInsertId()

	if err != nil {

		response.Error(http.StatusInternalServerError, "Error adding reading to database(2): %s", err.Error())
		return
	}

	Log.Debug.Printf("Reading ID %d", reading_id)

	// done

	response.OK(http.StatusOK, "reading id %d", reading_id)

	Log.Debug.Printf("OK")
}

//////////////////////////////////////////////////////////////////////

func main() {

	// command line parameters

	var credentials_filename string
	var log_level_name string

	flag.StringVar(&credentials_filename, "credentials", "", "Specify credentials filename (required)")
	flag.StringVar(&log_level_name, "log_level", "info", "Specify log level (debug|verbose|info|warning|error|fatal)")
	flag.Parse()

	// check and action parameters

	errs := make([]error, 0)

	if err := set_log_level(log_level_name); err != nil {
		errs = append(errs, err)
	}

	if err := load_credentials(credentials_filename); err != nil {
		errs = append(errs, err)
	}

	if len(errs) != 0 {

		for _, e := range errs {
			fmt.Fprintf(os.Stderr, "%s\n", e.Error())
		}
		fmt.Fprintf(os.Stderr, "Usage:\n")
		flag.CommandLine.SetOutput(os.Stderr)
		flag.PrintDefaults()

		exit_code := 0
		if len(errs) != 0 {
			exit_code = 1
		}
		os.Exit(exit_code)
	}

	Log.Info.Printf("======= SALT SENSOR SERVICE BEGINS =======")

	Log.Debug.Printf("Log level set to '%s'", log_level_name)

	// database config

	cfg := mysql.Config{
		User:                 db_credentials.Username,
		Passwd:               db_credentials.Password,
		Net:                  "tcp",
		Addr:                 "localhost:3306",
		DBName:               "salt_sensor",
		AllowNativePasswords: true,
	}

	Log.Debug.Printf("Database is '%s', Username is '%s'", cfg.DBName, cfg.User)

	db_DSNString = cfg.FormatDSN()

	// HTTP server

	Log.Verbose.Printf("Serving HTTP...")

	http.HandleFunc("/reading", http_reading_handler)

	err := http.ListenAndServe("0.0.0.0:5002", nil)

	if errors.Is(err, http.ErrServerClosed) {

		Log.Verbose.Printf("Server closed")

	} else if err != nil {

		Log.Fatal.Printf("Error starting server: %s", err)
		os.Exit(1)
	}
}
