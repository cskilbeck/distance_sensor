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
	"sync"
	"time"

	"github.com/go-sql-driver/mysql"
	"github.com/julienschmidt/httprouter"
)

//////////////////////////////////////////////////////////////////////
// Settings to send back to gadget

type sensorSettings struct {
	Sleep_count int `json:"sleep_count"`
}

//////////////////////////////////////////////////////////////////////
// HTTP error response

type errorResponse struct {
	Info []string `json:"info,omitempty"`
}

//////////////////////////////////////////////////////////////////////
// HTTP PUT /reading response

type putReadingResponse struct {
	Settings sensorSettings `json:"settings"`
}

//////////////////////////////////////////////////////////////////////
// HTTPS GET /readings response

type getReadingsResponse struct {
	Rows     int      `json:"rows"`     // # of results
	Time     []int64  `json:"time"`     // seconds since unix epoch
	Vbat     []uint16 `json:"vbat"`     // mV * 10
	Distance []uint16 `json:"distance"` // millimetres
	Rssi     []int8   `json:"rssi"`     // dBm
}

//////////////////////////////////////////////////////////////////////

type routeHandler = func(w http.ResponseWriter, r *http.Request, params httprouter.Params)

//////////////////////////////////////////////////////////////////////
// Database credentials

type credentials struct {
	Username string `json:"username"`
	Password string `json:"password"`
}

//////////////////////////////////////////////////////////////////////
// Logging admin

const (
	logLevelDebug int = iota
	logLevelVerbose
	logLevelInfo
	logLevelWarning
	logLevelError
	logLevelFatal
	numLogLevels
)

var logLevelNames = [numLogLevels]string{
	"debug",
	"verbose",
	"info",
	"warning",
	"error",
	"fatal",
}

var logLevel = logLevelInfo

type loggers struct {
	Debug   *log.Logger
	Verbose *log.Logger
	Info    *log.Logger
	Warning *log.Logger
	Error   *log.Logger
	Fatal   *log.Logger
}

var logger loggers

//////////////////////////////////////////////////////////////////////
// DB admin

var dbCredentials credentials

var dbDSNString string

//////////////////////////////////////////////////////////////////////
// get len of longest string in a slice of strings

func longestStringLength(s []string) int {

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
// setup log level by name

func setLogLevel(log_level_name string) (e error) {

	log_writers := [numLogLevels]io.Writer{
		os.Stdout,
		os.Stdout,
		os.Stdout,
		os.Stderr,
		os.Stderr,
		os.Stderr,
	}

	longest_name := longestStringLength(logLevelNames[:])

	// find the log level by name if they asked for one

	found := true

	if len(log_level_name) != 0 {

		found = false

		for i, name := range logLevelNames {

			if strings.EqualFold(log_level_name, name) {

				logLevel = i
				found = true
				break
			}
		}
	}

	if !found {
		return fmt.Errorf("unknown log level \"%s\"", log_level_name)
	}

	// create a logger for each log_level

	var logs []*log.Logger = make([]*log.Logger, numLogLevels)

	// use io.Discard for loggers below requested level

	for i := 0; i < logLevel; i++ {

		logs[i] = log.New(io.Discard, "", 0)
	}

	// stdout or stderr for enabled levels

	for i := logLevel; i < numLogLevels; i++ {

		name := logLevelNames[i]
		padded := name + strings.Repeat(" ", longest_name-len(name)) + ":"
		logs[i] = log.New(log_writers[i], padded, log.Ldate|log.Ltime|log.Lmicroseconds)
	}

	// setup for use

	logger = loggers{
		Debug:   logs[logLevelDebug],
		Verbose: logs[logLevelVerbose],
		Info:    logs[logLevelInfo],
		Warning: logs[logLevelWarning],
		Error:   logs[logLevelError],
		Fatal:   logs[logLevelFatal],
	}

	return nil
}

//////////////////////////////////////////////////////////////////////
// Load a JSON object from a file

func loadJSON[T any](filename string, result *T) error {

	var err error
	var content []byte

	if content, err = os.ReadFile(filename); err != nil {

		return fmt.Errorf("loading %s, can't %s", filename, err)
	}

	if err = json.Unmarshal(content, result); err != nil {

		return fmt.Errorf("error parsing %s: %s", filename, err)
	}
	return nil
}

//////////////////////////////////////////////////////////////////////
// HTTP Body is always a json Response struct

func sendResponse[T any](w http.ResponseWriter, status int, response *T) {

	var err error
	var body []byte

	if body, err = json.MarshalIndent(response, "", "  "); err != nil {

		logger.Fatal.Panicf("Can't marshal json response: %s", err.Error())
	}

	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET, PUT")
	w.Header().Set("Access-Control-Allow-Headers", "Origin, Content-Type")
	w.Header().Set("Content-Type", "application/json")

	w.WriteHeader(status)

	if _, err = w.Write(body); err != nil {

		logger.Error.Printf("Can't write response: %s", err.Error())
	}
}

//////////////////////////////////////////////////////////////////////

func respondError(w http.ResponseWriter, status int, info ...string) {

	sendResponse(w, status, &errorResponse{Info: info})
}

//////////////////////////////////////////////////////////////////////

func notFound(w http.ResponseWriter, r *http.Request) {

	respondError(w, http.StatusNotFound, fmt.Sprintf("%s %s not found", r.Method, r.URL))
}

//////////////////////////////////////////////////////////////////////

func methodNotAllowed(w http.ResponseWriter, r *http.Request) {

	respondError(w, http.StatusMethodNotAllowed, fmt.Sprintf("Method %s not allowed", r.Method))
}

//////////////////////////////////////////////////////////////////////

func checkParam(name string, values url.Values) (string, error) {

	params := values[name]

	if len(params) < 1 {
		return "", fmt.Errorf("missing parameter %s", name)
	}

	if len(params) > 1 {
		return "", fmt.Errorf("duplicate parameter %s", name)
	}

	if len(params[0]) == 0 {
		return "", fmt.Errorf("missing value for parameter %s", name)
	}

	return params[0], nil
}

//////////////////////////////////////////////////////////////////////

func parseQueryParamInt[T int64 | uint64](name string, values url.Values, parser func(string) (T, error)) (T, error) {

	param, err := checkParam(name, values)

	if err != nil {
		return 0, err
	}

	v, err := parser(param)

	if err != nil {
		return 0, fmt.Errorf("bad value for '%s' (%s)", name, err.Error())
	}

	return v, nil
}

//////////////////////////////////////////////////////////////////////

func parseQueryParamUnsigned(name string, base int, bits int, values url.Values) (uint64, error) {

	return parseQueryParamInt[uint64](name, values, func(s string) (uint64, error) {

		return strconv.ParseUint(s, base, bits)
	})
}

//////////////////////////////////////////////////////////////////////

func parseQueryParamSigned(name string, base int, bits int, values url.Values) (int64, error) {

	return parseQueryParamInt[int64](name, values, func(s string) (int64, error) {

		return strconv.ParseInt(s, base, bits)
	})
}

//////////////////////////////////////////////////////////////////////

func parseQueryParamTime(name string, values url.Values) (time.Time, error) {

	param, err := checkParam(name, values)

	if err != nil {
		return time.Time{}, err
	}

	return time.Parse(time.RFC3339, param)
}

//////////////////////////////////////////////////////////////////////

func logged(h routeHandler) routeHandler {

	return func(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

		logger.Verbose.Printf("%s %s", r.Method, r.URL)
		h(w, r, params)
	}
}

//////////////////////////////////////////////////////////////////////
// Get report of readings for time span

func getReadings(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var err error

	// parse the query string

	var errors = make([]string, 0)

	q := r.URL.Query()

	var from time.Time
	var to time.Time
	var device uint64

	if device, err = parseQueryParamUnsigned("device", 16, 48, q); err != nil {
		errors = append(errors, err.Error())
	}

	if from, err = parseQueryParamTime("from", q); err != nil {
		errors = append(errors, err.Error())
	}

	if to, err = parseQueryParamTime("to", q); err != nil {
		errors = append(errors, err.Error())
	}

	if len(errors) != 0 {
		respondError(w, http.StatusBadRequest, errors...)
		return
	}

	// open the database

	var db *sql.DB

	if db, err = sql.Open("mysql", dbDSNString); err != nil {

		respondError(w, http.StatusInternalServerError, fmt.Sprintf("Error opening database: %s", err.Error()))
		return
	}

	logger.Debug.Println("Database opened")

	defer db.Close()

	var device_id int64

	device_address := fmt.Sprintf("%012x", device)

	if err = db.QueryRow(`SELECT device_id FROM devices WHERE device_address = ?;`, device_address).Scan(&device_id); err != nil {

		if err == sql.ErrNoRows {
			respondError(w, http.StatusBadRequest, fmt.Sprintf("Device %s not found", device_address))
			return
		}
	}

	logger.Debug.Printf("Device ID %d", device_id)

	if len(errors) != 0 {
		respondError(w, http.StatusBadRequest, errors...)
		return
	}

	logger.Debug.Printf("From: %s, To: %s", from.Format(time.RFC3339), to.Format(time.RFC3339))

	query_string := `SELECT
						reading_timestamp, reading_vbat, reading_distance, reading_rssi
						FROM readings
						WHERE device_id = ?
							AND reading_timestamp >= ?
							AND reading_timestamp <= ?
						ORDER BY reading_timestamp ASC`

	var stmt *sql.Stmt

	if stmt, err = db.Prepare(query_string); err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}

	defer stmt.Close()

	var query *sql.Rows

	if query, err = stmt.Query(device_id, from.Format(time.RFC3339), to.Format(time.RFC3339)); err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}

	response := getReadingsResponse{}

	rows := 0

	for query.Next() {
		var tim time.Time
		var vbat uint16
		var distance uint16
		var rssi int8
		err = query.Scan(&tim, &vbat, &distance, &rssi)
		if err != nil {
			respondError(w, http.StatusInternalServerError, err.Error())
			return
		}
		if distance != 0 {
			response.Time = append(response.Time, tim.Unix())
			response.Vbat = append(response.Vbat, vbat)
			response.Distance = append(response.Distance, distance)
			response.Rssi = append(response.Rssi, rssi)
			rows += 1
		}
		logger.Debug.Printf("Fetched %d rows", rows)
	}

	response.Rows = rows

	sendResponse(w, http.StatusOK, &response)
}

//////////////////////////////////////////////////////////////////////

func putReading(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var err error

	// parse the query string

	var errors = make([]string, 0)

	var vbat uint64
	var distance uint64
	var device uint64
	var flags uint64
	var rssi int64

	q := r.URL.Query()

	if vbat, err = parseQueryParamUnsigned("vbat", 10, 16, q); err != nil {
		errors = append(errors, err.Error())
	}

	if distance, err = parseQueryParamUnsigned("distance", 10, 16, q); err != nil {
		errors = append(errors, err.Error())
	}

	if device, err = parseQueryParamUnsigned("device", 16, 48, q); err != nil {
		errors = append(errors, err.Error())
	}

	if flags, err = parseQueryParamUnsigned("flags", 10, 16, q); err != nil {
		errors = append(errors, err.Error())
	}

	if rssi, err = parseQueryParamSigned("rssi", 10, 8, q); err != nil {
		errors = append(errors, err.Error())
	}

	if len(errors) != 0 {
		respondError(w, http.StatusBadRequest, errors...)
		return
	}

	logger.Debug.Printf("vbat: %d, distance: %d, flags: %d, device: %012x, rssi: %d", vbat, distance, flags, device, rssi)

	// open the database

	var db *sql.DB

	if db, err = sql.Open("mysql", dbDSNString); err != nil {

		respondError(w, http.StatusInternalServerError, fmt.Sprintf("Error opening database: %s", err.Error()))
		return
	}

	logger.Debug.Println("Database opened")

	defer db.Close()

	// get device id or insert new device

	device_address := fmt.Sprintf("%012x", device)

	logger.Debug.Printf("Get device id for %s", device_address)

	var device_id int64
	var sleep_count int
	var insert sql.Result

	if err = db.QueryRow(`SELECT device_id, sleep_count FROM devices WHERE device_address = ?;`, device_address).Scan(&device_id, &sleep_count); err != nil {

		if err == sql.ErrNoRows {

			logger.Info.Printf("New device: %s", device_address)

			if insert, err = db.Exec(`INSERT INTO devices (device_address) VALUES (?)`, device_address); err == nil {

				device_id, err = insert.LastInsertId()
			}
		}
	}

	if err != nil {
		respondError(w, http.StatusInternalServerError, "Database error adding device %s: %s", device_address, err.Error())
		return
	}

	logger.Debug.Printf("Device ID %d", device_id)

	// add the reading

	var reading_id int64

	if insert, err = db.Exec(`INSERT INTO readings (device_id, reading_vbat, reading_distance, reading_flags, reading_rssi, reading_timestamp)
                               VALUES (?, ?, ?, ?, ?, CURRENT_TIMESTAMP());`, device_id, vbat, distance, flags, rssi); err == nil {

		reading_id, err = insert.LastInsertId()
	}

	if err != nil {
		respondError(w, http.StatusInternalServerError, fmt.Sprintf("Database error adding reading: %s", err.Error()))
		return
	}

	logger.Debug.Printf("Reading ID %d", reading_id)

	// done

	sendResponse(w, http.StatusOK, &putReadingResponse{Settings: sensorSettings{Sleep_count: sleep_count}})
}

//////////////////////////////////////////////////////////////////////

func main() {

	// command line parameters

	var credentials_filename string
	var key_filename string
	var cert_filename string
	var log_level_name string

	flag.StringVar(&credentials_filename, "credentials", "", "Specify credentials filename (required)")
	flag.StringVar(&key_filename, "key", "", "Specify TLS key filename (required)")
	flag.StringVar(&cert_filename, "cert", "", "Specify TLS cert filename (required)")
	flag.StringVar(&log_level_name, "log_level", logLevelNames[logLevel], fmt.Sprintf("Specify log level (%s)", strings.Join(logLevelNames[:], "|")))
	flag.Parse()

	// check command line parameters

	errs := make([]error, 0)

	if err := setLogLevel(log_level_name); err != nil {
		errs = append(errs, err)
	}

	if len(key_filename) == 0 {
		errs = append(errs, fmt.Errorf("missing key_filename"))
	}

	if len(cert_filename) == 0 {
		errs = append(errs, fmt.Errorf("missing cert_filename"))
	}

	if len(credentials_filename) == 0 {
		errs = append(errs, fmt.Errorf("missing credentials_filename"))
	}

	// load database credentials

	if err := loadJSON[credentials](credentials_filename, &dbCredentials); err != nil {
		errs = append(errs, err)
	}

	// report any startup errors

	if len(errs) != 0 {

		for _, e := range errs {
			fmt.Fprintf(os.Stderr, "%s\n", e.Error())
		}
		flag.CommandLine.SetOutput(os.Stderr)
		flag.Usage()
		os.Exit(1)
	}

	// startup ok, here we go

	logger.Info.Println()
	logger.Info.Printf("<<<<<<<<<< SALT SENSOR SERVICE BEGINS >>>>>>>>>>")

	logger.Verbose.Printf("Log level: %s", logLevelNames[logLevel])
	logger.Verbose.Printf("Credentials file: %s", credentials_filename)
	logger.Verbose.Printf("Key file: %s", key_filename)
	logger.Verbose.Printf("Cert file: %s", cert_filename)

	// database config

	cfg := mysql.Config{
		User:                 dbCredentials.Username,
		Passwd:               dbCredentials.Password,
		Net:                  "tcp",
		Addr:                 "localhost:3306",
		DBName:               "salt_sensor",
		AllowNativePasswords: true,
		ParseTime:            true,
	}

	logger.Debug.Printf("Database is '%s'", cfg.DBName)
	logger.Debug.Printf("Username is '%s'", cfg.User)

	dbDSNString = cfg.FormatDSN()

	// HTTP server

	http_router := httprouter.New()
	http_router.NotFound = http.HandlerFunc(notFound)
	http_router.MethodNotAllowed = http.HandlerFunc(methodNotAllowed)

	http_router.PUT("/reading", logged(putReading))

	// HTTPS server

	https_router := httprouter.New()
	https_router.NotFound = http.HandlerFunc(notFound)
	https_router.MethodNotAllowed = http.HandlerFunc(methodNotAllowed)

	https_router.GET("/readings", logged(getReadings))

	// run them both

	var wg sync.WaitGroup

	wg.Add(2)

	run_server := func(name string, listener func() error) {

		logger.Verbose.Printf("%s server starting", name)

		err := listener()

		if errors.Is(err, http.ErrServerClosed) {
			logger.Verbose.Printf("%s server closed", name)

		} else if err != nil {
			logger.Error.Printf("Error starting %s server: %s", name, err.Error())
		}
		wg.Done()
	}

	go run_server("HTTP", func() error {
		return http.ListenAndServe("0.0.0.0:5002", http_router)
	})

	go run_server("HTTPS", func() error {
		return http.ListenAndServeTLS(":443", cert_filename, key_filename, https_router)
	})

	wg.Wait()

	logger.Info.Printf("---------- SALT SENSOR SERVICE ENDS ----------")
}
