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
	SleepCount int `json:"sleep_count"`
}

//////////////////////////////////////////////////////////////////////
// Device identifier

type device struct {
	Address string `json:"address"` // mac address
	Name    string `json:"name"`    // user defined name
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

type getDevicesResponse struct {
	Rows   int      `json:"rows"`   // # of results
	Device []device `json:"device"` // devices
}

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

var logLevel = logLevelInfo // default log level is info

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

func setupLogging(log_level_name string) (e error) {

	log_writers := [numLogLevels]io.Writer{
		os.Stdout,
		os.Stdout,
		os.Stdout,
		os.Stderr,
		os.Stderr,
		os.Stderr,
	}

	longest_name := longestStringLength(logLevelNames[:])

	// find the log level by name

	found := false

	if len(log_level_name) != 0 {

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

func parseQueryParamInt[T int64 | uint64](s string, name string, parser func(string) (T, error)) (T, error) {

	v, err := parser(s)

	if err != nil {
		return 0, fmt.Errorf("bad value '%s' (%s)", name, err.Error())
	}

	return v, nil
}

//////////////////////////////////////////////////////////////////////

func parseQueryParamUnsigned(name string, base int, bits int, values url.Values) (uint64, error) {

	param, err := checkParam(name, values)

	if err != nil {
		return 0, err
	}

	return parseQueryParamInt[uint64](param, name, func(s string) (uint64, error) {

		return strconv.ParseUint(s, base, bits)
	})
}

//////////////////////////////////////////////////////////////////////

func parseQueryParamSigned(name string, base int, bits int, values url.Values) (int64, error) {

	param, err := checkParam(name, values)

	if err != nil {
		return 0, err
	}

	return parseQueryParamInt[int64](param, name, func(s string) (int64, error) {

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

type routeHandler = func(w http.ResponseWriter, r *http.Request, params httprouter.Params)

func logged(h routeHandler) routeHandler {

	return func(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

		logger.Verbose.Printf("%s %s", r.Method, r.URL)
		h(w, r, params)
	}
}

//////////////////////////////////////////////////////////////////////

func openDatabase(w http.ResponseWriter) (*sql.DB, error) {

	var err error
	var db *sql.DB

	if db, err = sql.Open("mysql", dbDSNString); err != nil {

		respondError(w, http.StatusInternalServerError, fmt.Sprintf("Error opening database: %s", err.Error()))
		return nil, err
	}

	logger.Debug.Println("Database opened")

	return db, nil
}

//////////////////////////////////////////////////////////////////////
// Get report of readings for time span
// Inputs
//	device=AABBCCDDEEFF		// device mac address (required)
//	from=RFC3339			// readings from this time (optional, default = beginning of time)
//	to=RFC3339				// readings from this time (optional, default = now + 24 hours (because timezones))
//  count=uint16			// max # of readings to get (optional, default = 1000)

func getReadings(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var err error

	// parse the query string

	var errors = make([]string, 0)

	queryParams := r.URL.Query()

	// device is required

	var device uint64

	// these are optional with defaults

	from := time.Time{}
	to := time.Now().Add(time.Hour * 24)
	count := uint64(1000)

	// scan the parameters

	if device, err = parseQueryParamUnsigned("device", 16, 48, queryParams); err != nil {
		errors = append(errors, err.Error())
	}

	if len(queryParams["from"]) != 0 {
		if from, err = parseQueryParamTime("from", queryParams); err != nil {
			errors = append(errors, err.Error())
		}
	}

	if len(queryParams["to"]) != 0 {
		if to, err = parseQueryParamTime("to", queryParams); err != nil {
			errors = append(errors, err.Error())
		}
	}

	if len(queryParams["count"]) != 0 {
		if count, err = parseQueryParamUnsigned("count", 10, 16, queryParams); err != nil {
			errors = append(errors, err.Error())
		} else if count > 10000 {
			errors = append(errors, fmt.Sprintf("count %d out of range, must be < 10000", count))
		}
	}

	// send error(s) if anything wrong

	if len(errors) != 0 {
		respondError(w, http.StatusBadRequest, errors...)
		return
	}

	// open the database

	var db *sql.DB

	if db, err = openDatabase(w); err != nil {
		return
	}
	defer db.Close()

	// get the device id

	var deviceId int64

	deviceAddress := fmt.Sprintf("%012x", device)

	if err = db.QueryRow(`SELECT device_id
							FROM devices
							WHERE device_address = ?;`, deviceAddress).Scan(&deviceId); err != nil {

		if err == sql.ErrNoRows {
			respondError(w, http.StatusBadRequest, fmt.Sprintf("Device %s not found", deviceAddress))
			return
		}
	}

	logger.Debug.Printf("Device ID %d", deviceId)

	// get the readings

	logger.Debug.Printf("From: %s, To: %s, Count: %d", from.Format(time.RFC3339), to.Format(time.RFC3339), count)

	var sqlStatement *sql.Stmt

	if sqlStatement, err = db.Prepare(`
						SELECT reading_timestamp, reading_vbat, reading_distance, reading_rssi
						FROM readings
						WHERE device_id = ?
							AND reading_timestamp >= ?
							AND reading_timestamp <= ?
						ORDER BY reading_timestamp DESC
						LIMIT ?`); err != nil {

		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}

	defer sqlStatement.Close()

	var query *sql.Rows

	if query, err = sqlStatement.Query(deviceId, from.Format(time.RFC3339), to.Format(time.RFC3339), count); err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}

	// put the readings in an object for json output response

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
	}
	logger.Debug.Printf("Fetched %d rows", rows)
	response.Rows = rows
	sendResponse(w, http.StatusOK, &response)
}

//////////////////////////////////////////////////////////////////////
// get all the devices

func getDevices(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	// open the database

	var err error

	var db *sql.DB

	if db, err = openDatabase(w); err != nil {
		return
	}
	defer db.Close()

	// get the devices

	var query *sql.Rows

	if query, err = db.Query(`SELECT device_address, IFNULL(device_name, '-') FROM devices`); err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}

	// put the devices in an object for json output response

	response := getDevicesResponse{}

	rows := 0

	for query.Next() {
		var addr string
		var name string
		if err = query.Scan(&addr, &name); err != nil {
			respondError(w, http.StatusInternalServerError, err.Error())
			return
		}
		rows += 1
		response.Device = append(response.Device, device{Address: addr, Name: name})
	}
	logger.Debug.Printf("Fetched %d rows", rows)
	response.Rows = rows
	sendResponse(w, http.StatusOK, &response)
}

//////////////////////////////////////////////////////////////////////
// put a new reading
// parameters:
// vbat uint64			vbat in 100mV units (i.e. 843 = 8.43 volts)
// distance uint64		distance in counter units (see resolution)
// device uint64		device mac address in hex
// flags uint64			flags for reference
// rssi int64			WiFi strength in dBm
// resolution int64		resolution of distance timer counter
//
// distance in mm is calculated as :
// 48MHz / resolution = counter hertz (e.g. countscale = 8, 48mHz / 8 = 6MHz Counter Scale)
// distance is in 'counter values', and should be halved because it's a reflection ping thing, so
//
// mm = counter_reading / 2 * (speed_of_sound_in_mm_per_sec) / counter_scale
//
// e.g. for
// distance = 4025
// resolution = 6
// clock_speed = 48000000
// speed_of_sound_in_mm_per_sec = 346000 mm/sec
// distance / 2 = 2012.5
// counter_scale = clock_speed (48000000) / resolution (6) = 8000000
// unscaled_distance = distance / 2 (2012.5) * speed_of_sound_in_mm_per_sec (346000) = 696325000
// final_distance = unscaled_distance (696325000) / counter_scale (8000000) = 87.04mm, call it 87

func putReading(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var err error

	// parse the query string

	var errors = make([]string, 0)

	var vbat uint64
	var distance uint64
	var device uint64
	var flags uint64
	var rssi int64
	var resolution int64

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

	if resolution, err = parseQueryParamSigned("resolution", 10, 32, q); err != nil {
		errors = append(errors, err.Error())
	}

	if len(errors) != 0 {
		respondError(w, http.StatusBadRequest, errors...)
		return
	}

	// counter counts at this many per second (eg 8000000)

	time_scale := 48000000.0 / float32(resolution)

	const speed_of_sound_mm_per_sec = 346000.0

	distance_mm := int32(float32(distance) / time_scale * speed_of_sound_mm_per_sec / 2.0)

	logger.Verbose.Printf("vbat: %d, reading: %d, flags: %d, device: %012x, rssi: %d, distance: %d", vbat, distance, flags, device, rssi, distance_mm)

	var db *sql.DB

	if db, err = openDatabase(w); err != nil {
		return
	}
	defer db.Close()

	// get device id or insert new device

	deviceAddress := fmt.Sprintf("%012x", device)

	logger.Debug.Printf("Get device id for %s", deviceAddress)

	var deviceId int64
	var sleepCount int
	var insert sql.Result

	if err = db.QueryRow(`SELECT
							device_id, sleep_count
							FROM devices
							WHERE device_address = ?;`, deviceAddress).Scan(&deviceId, &sleepCount); err != nil {

		if err == sql.ErrNoRows {

			logger.Info.Printf("New device: %s", deviceAddress)

			if insert, err = db.Exec(`INSERT INTO devices
										(device_address) VALUES (?)`, deviceAddress); err == nil {

				deviceId, err = insert.LastInsertId()
			}
		}
	}

	if err != nil {
		respondError(w, http.StatusInternalServerError, "Database error adding device %s: %s", deviceAddress, err.Error())
		return
	}

	logger.Debug.Printf("Device ID %d", deviceId)

	// add the reading

	var readingId int64

	if insert, err = db.Exec(`INSERT INTO readings
								(device_id, reading_vbat, reading_distance, reading_flags, reading_rssi, reading_timestamp)
                               VALUES (?, ?, ?, ?, ?, CURRENT_TIMESTAMP());`, deviceId, vbat, distance_mm, flags, rssi); err == nil {

		readingId, err = insert.LastInsertId()
	}

	if err != nil {
		respondError(w, http.StatusInternalServerError, fmt.Sprintf("Database error adding reading: %s", err.Error()))
		return
	}

	logger.Debug.Printf("Reading ID %d", readingId)

	sendResponse(w, http.StatusOK, &putReadingResponse{Settings: sensorSettings{SleepCount: sleepCount}})
}

//////////////////////////////////////////////////////////////////////

func makeRouter() *httprouter.Router {

	router := httprouter.New()
	router.NotFound = http.HandlerFunc(notFound)
	router.MethodNotAllowed = http.HandlerFunc(methodNotAllowed)
	return router
}

//////////////////////////////////////////////////////////////////////

func main() {

	// command line parameters

	var credentialsFilename string
	var keyFilename string
	var certFilename string
	var logLevelName string
	var httpOnly bool

	flag.StringVar(&credentialsFilename, "credentials", "", "Specify credentials filename (required)")
	flag.StringVar(&keyFilename, "key", "", "Specify TLS key filename (required)")
	flag.StringVar(&certFilename, "cert", "", "Specify TLS cert filename (required)")
	flag.StringVar(&logLevelName, "log_level", logLevelNames[logLevel], fmt.Sprintf("Specify log level (%s)", strings.Join(logLevelNames[:], "|")))
	flag.BoolVar(&httpOnly, "http_only", false, "Only serve through HTTP:5002 (DEBUG ONLY)")
	flag.Parse()

	// check command line parameters, setup logging and database credentials

	errs := make([]error, 0)

	if len(logLevelName) == 0 {
		errs = append(errs, fmt.Errorf("missing -log_level"))
	}

	if len(keyFilename) == 0 && !httpOnly {
		errs = append(errs, fmt.Errorf("missing -key"))
	}

	if len(certFilename) == 0 && !httpOnly {
		errs = append(errs, fmt.Errorf("missing -cert"))
	}

	if len(credentialsFilename) == 0 {
		errs = append(errs, fmt.Errorf("missing -credentials"))
	} else {
		if err := loadJSON[credentials](credentialsFilename, &dbCredentials); err != nil {
			errs = append(errs, err)
		}
	}

	if err := setupLogging(logLevelName); err != nil {
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
	logger.Info.Printf("########## SALT SENSOR SERVICE BEGINS ##########")

	logger.Verbose.Printf("Log level: %s", logLevelNames[logLevel])
	logger.Verbose.Printf("Credentials file: %s", credentialsFilename)

	if !httpOnly {

		logger.Verbose.Printf("Key file: %s", keyFilename)
		logger.Verbose.Printf("Cert file: %s", certFilename)

	} else {

		logger.Warning.Printf("NO HTTPS ENABLED - EVERYTHING SERVED THROUGH HTTP:5002!!!!")
	}

	// database config

	logger.Debug.Printf("Database is '%s'", dbCredentials.Password)
	logger.Debug.Printf("Username is '%s'", dbCredentials.Username)

	dbDSNString = (&mysql.Config{
		User:                 dbCredentials.Username,
		Passwd:               dbCredentials.Password,
		Net:                  "tcp",
		Addr:                 "localhost:3306",
		DBName:               "salt_sensor",
		AllowNativePasswords: true,
		ParseTime:            true,
	}).FormatDSN()

	// task admin

	var waitGroup sync.WaitGroup

	taskCount := 2

	if httpOnly {
		taskCount = 1
	}

	waitGroup.Add(taskCount)

	runServer := func(name string, listener func() error) {

		logger.Verbose.Printf("%s server starting", name)

		err := listener()

		if errors.Is(err, http.ErrServerClosed) {
			logger.Verbose.Printf("%s server closed", name)

		} else if err != nil {
			logger.Error.Printf("Error starting %s server: %s", name, err.Error())
		}
		waitGroup.Done()
	}

	// HTTP always available

	httpRouter := makeRouter()

	httpRouter.PUT("/reading", logged(putReading)) // PUT /reading only available via HTTP
	httpRouter.GET("/readings", logged(getReadings))
	httpRouter.GET("/devices", logged(getDevices))

	go runServer("HTTP", func() error {
		return http.ListenAndServe(":5002", httpRouter)
	})

	if !httpOnly {

		httpsRouter := makeRouter()

		httpsRouter.GET("/readings", logged(getReadings))
		httpsRouter.GET("/devices", logged(getDevices))

		go runServer("HTTPS", func() error {
			return http.ListenAndServeTLS(":443", certFilename, keyFilename, httpsRouter)
		})
	}

	waitGroup.Wait()

	logger.Info.Printf("---------- SALT SENSOR SERVICE ENDS ----------")
}
