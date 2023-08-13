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
	"github.com/julienschmidt/httprouter"
)

//////////////////////////////////////////////////////////////////////
// HTTP response body

type Response struct {
	Status string   `json:"status"`
	Info   []string `json:"info"`
}

//////////////////////////////////////////////////////////////////////

type RouteHandler = func(w http.ResponseWriter, r *http.Request, params httprouter.Params)

//////////////////////////////////////////////////////////////////////
// Database credentials

type Credentials struct {
	Username string `json:"username"`
	Password string `json:"password"`
}

//////////////////////////////////////////////////////////////////////
// Logging admin

const (
	log_level_debug int = iota
	log_level_verbose
	log_level_info
	log_level_warning
	log_level_error
	log_level_fatal
	num_log_levels
)

var log_level_names = [num_log_levels]string{
	"debug",
	"verbose",
	"info",
	"warning",
	"error",
	"fatal",
}

var log_level = log_level_info

type Loggers struct {
	Debug   *log.Logger
	Verbose *log.Logger
	Info    *log.Logger
	Warning *log.Logger
	Error   *log.Logger
	Fatal   *log.Logger
}

var Log Loggers

//////////////////////////////////////////////////////////////////////
// DB admin

var db_credentials Credentials

var db_DSNString string

//////////////////////////////////////////////////////////////////////

func LongestStringLength(s []string) int {

	longest := 0
	for _, str := range s {
		l := len(str)
		if l > longest {
			longest = l
		}
	}
	return longest
}

//////////////////////////////////////////////////////////////////////

func SetLogLevel(log_level_name string) (e error) {

	log_writers := [num_log_levels]io.Writer{
		os.Stdout,
		os.Stdout,
		os.Stdout,
		os.Stderr,
		os.Stderr,
		os.Stderr,
	}

	longest_name := LongestStringLength(log_level_names[:])

	// find the log level by name if they asked for one

	found := true

	if len(log_level_name) != 0 {

		found = false

		for i, name := range log_level_names {

			if strings.EqualFold(log_level_name, name) {

				log_level = i
				found = true
				break
			}
		}
	}

	if !found {
		return fmt.Errorf("unknown log level \"%s\"", log_level_name)
	}

	// create a logger for each log_level

	var logs []*log.Logger = make([]*log.Logger, num_log_levels)

	// use io.Discard for loggers below requested level

	for i := 0; i < log_level; i++ {

		logs[i] = log.New(io.Discard, "", 0)
	}

	// stdout or stderr for enabled levels

	for i := log_level; i < num_log_levels; i++ {

		name := log_level_names[i]
		padded := name + strings.Repeat(" ", longest_name-len(name)) + ":"
		logs[i] = log.New(log_writers[i], padded, log.Ldate|log.Ltime|log.Lmicroseconds)
	}

	// setup for use

	Log = Loggers{
		Debug:   logs[log_level_debug],
		Verbose: logs[log_level_verbose],
		Info:    logs[log_level_info],
		Warning: logs[log_level_warning],
		Error:   logs[log_level_error],
		Fatal:   logs[log_level_fatal],
	}

	return nil
}

//////////////////////////////////////////////////////////////////////

func LoadCredentials(credentials_filename string) error {

	if len(credentials_filename) == 0 {

		return errors.New("missing credentials filename")
	}

	var err error
	var content []byte

	if content, err = os.ReadFile(credentials_filename); err != nil {

		return fmt.Errorf("loading credentials, can't %s", err)
	}

	if err = json.Unmarshal(content, &db_credentials); err != nil {

		return fmt.Errorf("error parsing %s: %s", credentials_filename, err)
	}
	return nil
}

//////////////////////////////////////////////////////////////////////
// HTTP Body is always a json Response struct

func Respond(w http.ResponseWriter, status int, info ...string) {

	response := Response{Status: http.StatusText(status), Info: info}

	var err error
	var body []byte

	if body, err = json.MarshalIndent(response, "", "  "); err != nil {

		Log.Fatal.Panicf("Can't marshal json response: %s", err.Error())
	}

	w.Header().Set("Content-Type", "application/json")

	w.WriteHeader(status)

	if _, err = w.Write(body); err != nil {

		Log.Error.Printf("Can't write response: %s", err.Error())
	}

	for i, info := range response.Info {
		Log.Debug.Printf("Info[%d] = %s", i, info)
	}

	Log.Verbose.Printf("Status: %s", response.Status)
}

//////////////////////////////////////////////////////////////////////

func NotFound(w http.ResponseWriter, r *http.Request) {

	Respond(w, http.StatusNotFound)
}

//////////////////////////////////////////////////////////////////////

func MethodNotAllowed(w http.ResponseWriter, r *http.Request) {

	Respond(w, http.StatusMethodNotAllowed)
}

//////////////////////////////////////////////////////////////////////

func ParseQueryParam(name string, base int, bits int, values *url.Values) (uint64, error) {

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

func Logged(h RouteHandler) RouteHandler {

	return func(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

		Log.Debug.Printf("%s %s", r.Method, r.URL)
		h(w, r, params)
	}
}

//////////////////////////////////////////////////////////////////////

func InsertReading(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var err error

	// parse the query string

	var errors = make([]string, 0)

	var vbat uint64
	var distance uint64
	var device uint64
	var flags uint64

	q := r.URL.Query()

	if vbat, err = ParseQueryParam("vbat", 10, 16, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if distance, err = ParseQueryParam("distance", 10, 16, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if device, err = ParseQueryParam("device", 16, 48, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if flags, err = ParseQueryParam("flags", 10, 16, &q); err != nil {
		errors = append(errors, err.Error())
	}

	if len(errors) != 0 {
		Respond(w, http.StatusBadRequest, errors...)
		return
	}

	Log.Debug.Printf("vbat: %d, distance: %d, flags: %d, device: %012x", vbat, distance, flags, device)

	// open the database

	var db *sql.DB

	db, err = sql.Open("mysql", db_DSNString)
	if err != nil {
		Respond(w, http.StatusInternalServerError, fmt.Sprintf("Error opening database: %s", err.Error()))
		return
	}

	Log.Debug.Println("Database opened")

	defer db.Close()

	// get device id or insert new device

	device_address := fmt.Sprintf("%012x", device)

	Log.Debug.Printf("Get device id for %s", device_address)

	var device_id int64
	var insert sql.Result

	if err = db.QueryRow(`SELECT device_id FROM devices WHERE device_address = ?;`, device_address).Scan(&device_id); err != nil {

		if err == sql.ErrNoRows {

			Log.Info.Printf("New device: %s", device_address)

			if insert, err = db.Exec(`INSERT INTO devices (device_address) VALUES (?)`, device_address); err == nil {

				device_id, err = insert.LastInsertId()
			}
		}
	}

	if err != nil {
		Respond(w, http.StatusInternalServerError, "Database error adding device %s: %s", device_address, err.Error())
		return
	}

	Log.Debug.Printf("Device ID %d", device_id)

	// add the reading

	var reading_id int64

	if insert, err = db.Exec(`INSERT INTO readings (device_id, reading_vbat, reading_distance, reading_flags, reading_timestamp)
                         	  VALUES (?,?,?,?,CURRENT_TIMESTAMP());`, device_id, vbat, distance, flags); err == nil {

		reading_id, err = insert.LastInsertId()
	}

	if err != nil {
		Respond(w, http.StatusInternalServerError, fmt.Sprintf("Database error adding reading: %s", err.Error()))
		return
	}

	Log.Debug.Printf("Reading ID %d", reading_id)

	// done

	Respond(w, http.StatusOK, fmt.Sprintf("Reading id: %d", reading_id))
}

//////////////////////////////////////////////////////////////////////

func main() {

	// command line parameters

	var credentials_filename string
	var log_level_name string

	flag.StringVar(&credentials_filename, "credentials", "", "Specify credentials filename (required)")
	flag.StringVar(&log_level_name, "log_level", log_level_names[log_level], fmt.Sprintf("Specify log level (%s)", strings.Join(log_level_names[:], "|")))
	flag.Parse()

	// check and action command line parameters

	errs := make([]error, 0)

	if err := SetLogLevel(log_level_name); err != nil {
		errs = append(errs, err)
	}

	if err := LoadCredentials(credentials_filename); err != nil {
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

	// command line OK, here we go

	Log.Info.Println()
	Log.Info.Printf("========== SALT SENSOR SERVICE BEGINS ==========")
	Log.Info.Println()

	Log.Debug.Printf("Log level: %s", log_level_names[log_level])

	// database config

	Log.Verbose.Printf("Loaded credentials file %s", credentials_filename)

	cfg := mysql.Config{
		User:                 db_credentials.Username,
		Passwd:               db_credentials.Password,
		Net:                  "tcp",
		Addr:                 "localhost:3306",
		DBName:               "salt_sensor",
		AllowNativePasswords: true,
	}

	Log.Debug.Printf("Database is '%s'", cfg.DBName)
	Log.Debug.Printf("Username is '%s'", cfg.User)

	db_DSNString = cfg.FormatDSN()

	// HTTP server

	Log.Verbose.Printf("HTTP server starting")

	router := httprouter.New()

	router.NotFound = http.HandlerFunc(NotFound)
	router.MethodNotAllowed = http.HandlerFunc(MethodNotAllowed)

	router.PUT("/reading", Logged(InsertReading))

	err := http.ListenAndServe("0.0.0.0:5002", router)

	if errors.Is(err, http.ErrServerClosed) {

		Log.Verbose.Printf("HTTP server closed")

	} else if err != nil {

		Log.Fatal.Printf("Error starting HTTP server: %s", err)
		os.Exit(1)
	}
}
