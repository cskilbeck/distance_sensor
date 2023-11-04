//////////////////////////////////////////////////////////////////////
// Salt Sensor Server

package main

import (
	"bytes"
	"database/sql"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"html/template"
	"net/http"
	"net/url"
	"os"
	"strconv"
	"strings"
	"time"

	"salt_server/dailyalarm"
	"salt_server/emailsender"
	"salt_server/logger"

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
	Id      int    `json:"id"`      // device Id
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

type global_credentials struct {
	Username     string `json:"username"`
	Password     string `json:"password"`
	SMTPServer   string `json:"smtp_server"`
	SMTPAccount  string `json:"smtp_account"`
	SMTPPassword string `json:"smtp_password"`
}

//////////////////////////////////////////////////////////////////////
// Warning email admin

type emailWarnings struct {
	Subject  string
	Header   string
	Warnings []string
	Footer   string
}

const emailTemplate string = `
	<h3 style="color: #c04000;">{{.Header}}</h3>
	<ul>
		{{range .Warnings}}
			<li>{{.}}</li>
		{{end}}
	</ul>
	<h5>{{.Footer}}</h5>
`

//////////////////////////////////////////////////////////////////////
// DB admin

var credentials global_credentials

const databaseName = "salt_sensor_test"

// const databaseName = "salt_sensor"

var dbDSNString string

//////////////////////////////////////////////////////////////////////

func saltDistanceToPercent(distance uint16) uint16 {

	const min_dist = 120
	const max_dist = 235

	max_range := max_dist - min_dist

	if distance < min_dist {
		distance = min_dist
	}

	if distance > max_dist {
		distance = max_dist
	}
	return uint16((max_dist - int(distance)) * 100 / max_range)
}

//////////////////////////////////////////////////////////////////////

func sendWarningEmail(data emailWarnings, recipients ...string) error {

	var err error

	t := template.New("email")

	if t, err = t.Parse(emailTemplate); err != nil {
		logger.Error.Printf("Error parsing email template: %s", err.Error())
		return err
	}

	html := new(bytes.Buffer)

	if err = t.Execute(html, data); err != nil {
		logger.Error.Printf("Error executing email template: %s", err.Error())
		return err
	}

	if err = emailsender.SendEmail(credentials.SMTPServer, 587,
		credentials.SMTPAccount, credentials.SMTPPassword,
		"Water Softener", data.Subject, html.String(),
		"text/html", recipients...); err != nil {

		logger.Error.Printf("Error sending email: %s", err.Error())
	}
	logger.Info.Printf("Sent warning email to %d recipient(s)", len(recipients))
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

func openDatabase() (*sql.DB, error) {

	var err error
	var db *sql.DB

	if db, err = sql.Open("mysql", dbDSNString); err != nil {

		return nil, err
	}

	logger.Debug.Println("Database opened")

	return db, nil
}

//////////////////////////////////////////////////////////////////////
// query some readings from the database

func queryReadings(from, to time.Time, count uint64, device uint64) (response getReadingsResponse, status int, err error) {

	var db *sql.DB

	if db, err = openDatabase(); err != nil {
		return getReadingsResponse{}, http.StatusInternalServerError, fmt.Errorf("error opening database: %s", err.Error())
	}
	defer db.Close()

	// get the device id

	deviceAddress := fmt.Sprintf("%012x", device)

	var deviceAddress48 uint64

	if err = db.QueryRow(`SELECT device_id
							FROM devices
							WHERE device_address = ?;`, deviceAddress).Scan(&deviceAddress48); err != nil {

		if err == sql.ErrNoRows {
			return getReadingsResponse{}, http.StatusBadRequest, fmt.Errorf("device %s not found", deviceAddress)
		}
	}

	logger.Debug.Printf("Device ID %d", deviceAddress48)

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

		return getReadingsResponse{}, http.StatusInternalServerError, err
	}

	defer sqlStatement.Close()

	var query *sql.Rows

	if query, err = sqlStatement.Query(deviceAddress48, from.Format(time.RFC3339), to.Format(time.RFC3339), count); err != nil {
		return getReadingsResponse{}, http.StatusInternalServerError, err
	}

	// put the readings in an object for json output response

	response = getReadingsResponse{}

	rows := 0

	for query.Next() {
		var tim time.Time
		var vbat uint16
		var distance uint16
		var rssi int8
		err = query.Scan(&tim, &vbat, &distance, &rssi)
		if err != nil {
			return getReadingsResponse{}, http.StatusInternalServerError, err
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
	return response, http.StatusOK, nil
}

//////////////////////////////////////////////////////////////////////
// query for all the devices

func queryDevices(db *sql.DB) (response getDevicesResponse, status int, err error) {

	// get the devices

	var query *sql.Rows

	if query, err = db.Query(`SELECT device_id, device_address, IFNULL(device_name, '-') FROM devices`); err != nil {
		return getDevicesResponse{}, http.StatusInternalServerError, err
	}

	// put the devices in an object for json output response

	response = getDevicesResponse{}

	rows := 0

	for query.Next() {
		var id int
		var addr string
		var name string
		if err = query.Scan(&id, &addr, &name); err != nil {
			return getDevicesResponse{}, http.StatusInternalServerError, err
		}
		rows += 1
		response.Device = append(response.Device, device{Id: id, Address: addr, Name: name})
	}
	logger.Debug.Printf("Fetched %d rows", rows)
	response.Rows = rows

	return response, http.StatusOK, nil
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

	var deviceAddress48 uint64

	// these are optional with defaults

	from := time.Time{}
	to := time.Now().Add(time.Hour * 24)
	count := uint64(1000)

	// scan the parameters

	if deviceAddress48, err = parseQueryParamUnsigned("device", 16, 48, queryParams); err != nil {
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

	var response getReadingsResponse
	var httpStatus int
	response, httpStatus, err = queryReadings(from, to, count, deviceAddress48)

	if err != nil {
		respondError(w, httpStatus, err.Error())
	}

	sendResponse(w, http.StatusOK, &response)
}

//////////////////////////////////////////////////////////////////////
// get all the devices

func getDevices(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var db *sql.DB
	var err error

	if db, err = openDatabase(); err != nil {
		respondError(w, http.StatusInternalServerError, fmt.Sprintf("Error opening database: %s", err.Error()))
	}
	defer db.Close()

	response, status, err := queryDevices(db)

	if err != nil {
		respondError(w, status, err.Error())
	}
	sendResponse(w, http.StatusOK, &response)
}

//////////////////////////////////////////////////////////////////////
// add a reading to the database

func insertReading(device uint64, vbat uint64, distance_mm int32, flags uint64, rssi int64) (sleepCount int, err error) {

	logger.Verbose.Printf("vbat: %d, reading: %d, flags: %d, device: %012x, rssi: %d, distance: %d", vbat, distance_mm, flags, device, rssi, distance_mm)

	var db *sql.DB

	if db, err = openDatabase(); err != nil {
		return 0, fmt.Errorf("error opening database: %s", err.Error())
	}
	defer db.Close()

	// get device id or insert new device

	deviceAddress := fmt.Sprintf("%012x", device)

	logger.Debug.Printf("Get device id for %s", deviceAddress)

	var deviceAddress48 int64
	var insert sql.Result

	if err = db.QueryRow(`SELECT
							device_id, sleep_count
							FROM devices
							WHERE device_address = ?;`, deviceAddress).Scan(&deviceAddress48, &sleepCount); err != nil {

		if err == sql.ErrNoRows {

			logger.Info.Printf("New device: %s", deviceAddress)

			if insert, err = db.Exec(`INSERT INTO devices
										(device_address) VALUES (?)`, deviceAddress); err == nil {

				deviceAddress48, err = insert.LastInsertId()
			}
		}
	}

	if err != nil {
		return 0, fmt.Errorf("database error adding device %s: %s", deviceAddress, err.Error())
	}

	logger.Debug.Printf("Device ID %d", deviceAddress48)

	// add the reading

	var readingId int64

	if insert, err = db.Exec(`INSERT INTO readings
								(device_id, reading_vbat, reading_distance, reading_flags, reading_rssi, reading_timestamp)
                               VALUES (?, ?, ?, ?, ?, CURRENT_TIMESTAMP());`, deviceAddress48, vbat, distance_mm, flags, rssi); err == nil {

		readingId, err = insert.LastInsertId()
	}

	if err != nil {
		return 0, fmt.Errorf("database error adding reading: %s", err.Error())
	}

	logger.Debug.Printf("Reading ID %d", readingId)

	return sleepCount, nil
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
	var resolution int64 = 0

	query := r.URL.Query()

	if vbat, err = parseQueryParamUnsigned("vbat", 10, 16, query); err != nil {
		errors = append(errors, err.Error())
	}

	if distance, err = parseQueryParamUnsigned("distance", 10, 16, query); err != nil {
		errors = append(errors, err.Error())
	}

	if device, err = parseQueryParamUnsigned("device", 16, 48, query); err != nil {
		errors = append(errors, err.Error())
	}

	if flags, err = parseQueryParamUnsigned("flags", 10, 16, query); err != nil {
		errors = append(errors, err.Error())
	}

	if rssi, err = parseQueryParamSigned("rssi", 10, 8, query); err != nil {
		errors = append(errors, err.Error())
	}

	// if resolution not specified, then distance is in mm

	if len(query["resolution"]) != 0 {

		if resolution, err = parseQueryParamSigned("resolution", 10, 32, query); err != nil {
			errors = append(errors, err.Error())
		}
	}

	if len(errors) != 0 {
		respondError(w, http.StatusBadRequest, errors...)
		return
	}

	distance_mm := int32(distance)

	if resolution != 0 {

		// counter counts at this many per second (eg 8000000)

		time_scale := 48000000.0 / float32(resolution)

		const speed_of_sound_mm_per_sec = 346000.0

		distance_mm = int32(float32(distance) / time_scale * speed_of_sound_mm_per_sec / 2.0)
	}

	var sleepCount int
	sleepCount, err = insertReading(device, vbat, distance_mm, flags, rssi)

	if err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}
	sendResponse(w, http.StatusOK, &putReadingResponse{Settings: sensorSettings{SleepCount: sleepCount}})
}

//////////////////////////////////////////////////////////////////////
// putReading2 for the newer sensor
// no resolution (it's already in mm)
// sleep_count return is in seconds, not some dodgy other thing

func putReading2(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var err error

	// parse the query string

	var errors = make([]string, 0)

	var vbat uint64
	var distance uint64
	var device uint64
	var flags uint64
	var rssi int64

	query := r.URL.Query()

	if vbat, err = parseQueryParamUnsigned("vbat", 10, 16, query); err != nil {
		errors = append(errors, err.Error())
	}

	if distance, err = parseQueryParamUnsigned("distance", 10, 16, query); err != nil {
		errors = append(errors, err.Error())
	}

	if device, err = parseQueryParamUnsigned("device", 16, 48, query); err != nil {
		errors = append(errors, err.Error())
	}

	if flags, err = parseQueryParamUnsigned("flags", 10, 16, query); err != nil {
		errors = append(errors, err.Error())
	}

	if rssi, err = parseQueryParamSigned("rssi", 10, 8, query); err != nil {
		errors = append(errors, err.Error())
	}

	// if resolution not specified, then distance is in mm

	if len(errors) != 0 {
		respondError(w, http.StatusBadRequest, errors...)
		return
	}

	var sleepCountSeconds int
	sleepCountSeconds, err = insertReading(device, vbat, int32(distance), flags, rssi)

	if err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}
	sendResponse(w, http.StatusOK, &putReadingResponse{Settings: sensorSettings{SleepCount: sleepCountSeconds}})
}

//////////////////////////////////////////////////////////////////////
// Check if salt level below threshold for each device, send warning emails to registered recipients if it is

func checkSaltLevels() {

	logger.Info.Printf("Checking Salt Levels daily")

	// open the database

	var db *sql.DB
	var err error

	if db, err = openDatabase(); err != nil {
		logger.Error.Printf("Error opening database: %s", err.Error())
		return
	}
	defer db.Close()

	// get all the devices

	devices, _, err := queryDevices(db)

	if err != nil {
		logger.Error.Printf("Can't query devices!? (%s)", err.Error())
		return
	}

	// for each device

	for _, device := range devices.Device {

		var deviceAddress48 uint64
		var err error

		// check the device address is valid 48 bit hex (e.g. a mac address)

		if deviceAddress48, err = strconv.ParseUint(device.Address, 16, 48); err != nil {

			logger.Error.Printf("Device %s has bad address: %s", device.Address, err.Error())
			continue
		}

		// get the most recent reading for this device

		logger.Verbose.Printf("Notifications for device %s (%s) (id = %d)?", device.Address, device.Name, device.Id)

		if reading, _, err := queryReadings(time.Time{}, time.Now().Add(time.Hour*24), 1, deviceAddress48); err != nil {

			logger.Error.Printf("Error querying readings for device %s : %s", device.Address, err.Error())

		} else if reading.Rows == 0 {

			logger.Info.Printf("No readings for device %s (%s)", device.Address, device.Name)

		} else {

			// get the list of notification recipients and their warning levels for this device

			var query *sql.Rows

			if query, err = db.Query(`SELECT
											notification_distance_warning_threshold,
											notification_vbat_warning_threshold,
											notification_time_warning_threshold,
											account_email
										FROM notifications
											LEFT JOIN accounts ON
												notifications.account_id = accounts.account_id
										WHERE device_id = ?;`, device.Id); err != nil {

				if err == sql.ErrNoRows {

					logger.Info.Printf("No notifications for device %s", device.Address)
					continue
				}
				logger.Error.Printf("Error getting notifications for %s: %s", device.Address, err.Error())
				continue
			}

			// for each notification

			for query.Next() {

				var distance_threshold int32
				var vbat_threshold int32
				var time_threshold int32
				var email string

				// get the notification levels

				if err = query.Scan(&distance_threshold, &vbat_threshold, &time_threshold, &email); err != nil {

					logger.Error.Printf("Error reading notification data for device %s", device.Address)
					continue
				}

				warnings := []string{}

				// warn about salt level if required

				saltPercent := saltDistanceToPercent(reading.Distance[0])

				if saltPercent < uint16(distance_threshold) {
					warnings = append(warnings, fmt.Sprintf("Salt level is %d%%", saltPercent))
				}

				// warn about vbat level if required

				vbat := reading.Vbat[0]

				if vbat < uint16(vbat_threshold) {
					warnings = append(warnings, fmt.Sprintf("Battery level is %.1f volts", float64(vbat)/100.0))
				}

				// warn if no readings for > ?? hours if required (or any other warnings are required)

				last_reading := time.Unix(int64(reading.Time[0]), 0)
				hours_since := int32(time.Now().UTC().Sub(last_reading).Hours())

				if len(warnings) != 0 || hours_since > time_threshold {
					warnings = append(warnings, fmt.Sprintf("Last reading was %d hours ago", hours_since))
				}

				if len(warnings) == 0 {

					logger.Info.Printf("No warning for %s on %s(%s)", email, device.Name, device.Address)
					continue
				}

				// send the warning email to this notification recipient

				logger.Verbose.Printf("Sending alert email to %s for device %s (%s)", email, device.Name, device.Address)

				emailData := emailWarnings{
					Subject:  "Alert from the Water Softener",
					Header:   "Warning! The Salt Sensor says...",
					Warnings: warnings,
					Footer:   "This email is from the Salt Sensor Server",
				}
				sendWarningEmail(emailData, email)
			}
		}
	}
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
	var logLevelName string

	flag.StringVar(&credentialsFilename, "credentials", "", "Specify credentials filename (required)")
	flag.StringVar(&logLevelName, "log_level", logger.LogLevelNames[logger.LogLevel], fmt.Sprintf("Specify log level (%s)", strings.Join(logger.LogLevelNames[:], "|")))
	flag.Parse()

	// check command line parameters, setup logging and database credentials

	errs := make([]error, 0)

	if len(logLevelName) == 0 {
		errs = append(errs, fmt.Errorf("missing -log_level"))
	}

	if len(credentialsFilename) == 0 {
		errs = append(errs, fmt.Errorf("missing -credentials"))
	} else {
		if err := loadJSON[global_credentials](credentialsFilename, &credentials); err != nil {
			errs = append(errs, err)
		}
	}

	if err := logger.Init(logLevelName); err != nil {
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

	logger.Verbose.Printf("Log level: %s", logger.LogLevelNames[logger.LogLevel])
	logger.Verbose.Printf("Credentials file: %s", credentialsFilename)

	// database config

	logger.Verbose.Printf("Database is '%s'", databaseName)
	logger.Verbose.Printf("Username is '%s'", credentials.Username)

	logger.Verbose.Printf("SMTP Server is '%s'", credentials.SMTPServer)
	logger.Verbose.Printf("SMTP Account is '%s'", credentials.SMTPAccount)

	dbDSNString = (&mysql.Config{
		User:                 credentials.Username,
		Passwd:               credentials.Password,
		Net:                  "tcp",
		Addr:                 "localhost:3306",
		DBName:               databaseName,
		AllowNativePasswords: true,
		ParseTime:            true,
	}).FormatDSN()

	// daily alarm to check levels and send alert email if necessary

	var alarmChannel chan dailyalarm.AlarmNotify
	var err error

	if alarmChannel, err = dailyalarm.Set(6, 0, checkSaltLevels); err != nil {

		logger.Error.Printf("Error setting up daily alarm: %s", err.Error())
	}

	// HTTP always available

	httpRouter := makeRouter()

	httpRouter.PUT("/reading", logged(putReading))   // PUT /reading only available via HTTP
	httpRouter.PUT("/reading2", logged(putReading2)) // PUT /reading only available via HTTP
	httpRouter.GET("/readings", logged(getReadings))
	httpRouter.GET("/devices", logged(getDevices))

	logger.Verbose.Printf("HTTP server starting")

	err = http.ListenAndServe(":5002", httpRouter)

	if errors.Is(err, http.ErrServerClosed) {
		logger.Verbose.Printf("HTTP server closed")

	} else if err != nil {
		logger.Error.Printf("Error starting HTTP server: %s", err.Error())
	}

	// quit the daily alarm

	alarmChannel <- dailyalarm.AlarmQuit

	logger.Info.Printf("---------- SALT SENSOR SERVICE ENDS ----------")
}
