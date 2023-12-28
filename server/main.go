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
	"io"
	"net/http"
	"net/url"
	"os"
	"strconv"
	"strings"
	"time"

	"salt_server/dailyalarm"
	"salt_server/emailsender"
	log "salt_server/logger"

	"github.com/go-sql-driver/mysql"
	"github.com/julienschmidt/httprouter"
)

//////////////////////////////////////////////////////////////////////
// Settings to send back to gadget

type sensorSettings struct {
	SleepCount int `json:"sleep_count"`
}

//////////////////////////////////////////////////////////////////////
// paramaters for PUT /reading
// v1 also requires resolution int64
// this is all v2 needs

type readingParams struct {
	vbat     uint64 // vbat in 0.01V units (e.g. 395 = 3.95V)
	distance uint64 // distance in mm (v2) or... 'units' (v1)
	device   uint64 // 48 bit mac address of the device sending the reading
	flags    uint64 // flags (only v1)
	rssi     int64  // wifi rssi
}

//////////////////////////////////////////////////////////////////////
// Device identifier

type device struct {
	Id             int    `json:"id"`              // device Id
	Address        string `json:"address"`         // mac address
	Name           string `json:"name"`            // user defined name
	SensorType     string `json:"sensor_type"`     // type of sensor (V1, V2 etc)
	MinDist        uint16 `json:"min_dist"`        // min distance for this sensor type
	MaxDist        uint16 `json:"max_dist"`        // max distance for this sensor type
	MinVbat        uint16 `json:"min_vbat"`        // min vbat for this sensor type
	MaxVbat        uint16 `json:"max_vbat"`        // max vbat for this sensor type
	MaxSigma       uint16 `json:"max_sigma"`       // max sigma_mm for this sensor type
	MinReflectance uint8  `json:"min_reflectance"` // min reflectance for this sensor type
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
// Empty OK response

type okResponse struct {
	Ok string `json:"ok"`
}

//////////////////////////////////////////////////////////////////////
// HTTPS GET /readings response

type getReadingsResponse struct {
	Rows        int      `json:"rows"`        // # of results
	Time        []int64  `json:"time"`        // seconds since unix epoch
	Vbat        []uint16 `json:"vbat"`        // mV * 10
	Distance    []uint16 `json:"distance"`    // millimetres
	Rssi        []int8   `json:"rssi"`        // dBm
	Sigma       []uint16 `json:"sigma"`       // sigma mm (confidence, looks like)
	Reflectance []uint8  `json:"reflectance"` // reflectance (units? no clue)
}

//////////////////////////////////////////////////////////////////////

type getDevicesResponse struct {
	Rows   int      `json:"rows"`   // # of results
	Device []device `json:"device"` // devices
}

//////////////////////////////////////////////////////////////////////

type getDeviceCountResponse struct {
	DeviceCount uint64 `json:"device_count"` // # of devices
}

//////////////////////////////////////////////////////////////////////
// Credentials for database, smtp

type global_credentials struct {
	DBUsername   string `json:"db_username"`
	DBPassword   string `json:"db_password"`
	DBDatabase   string `json:"database"`
	SMTPServer   string `json:"smtp_server"`
	SMTPAccount  string `json:"smtp_account"`
	SMTPPassword string `json:"smtp_password"`
}

//////////////////////////////////////////////////////////////////////
// Warning email admin

type emailWarnings struct {
	Subject  string
	Header   string
	From     string
	FromLink string
	Warnings []string
	Footer   string
}

const emailTemplate string = `
	<h3 style="color: #c04000;">{{.Header}}</h3>
	<h4>From Salt Sensor <a href={{.FromLink}}>{{.From}}</a></h4>
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

var dbDSNString string

//////////////////////////////////////////////////////////////////////

func (device device) saltDistanceToPercent(distance uint16) uint16 {

	max_range := device.MaxDist - device.MinDist

	if distance < device.MinDist {
		distance = device.MinDist
	}

	if distance > device.MaxDist {
		distance = device.MaxDist
	}
	return uint16((device.MaxDist - distance) * 100 / max_range)
}

//////////////////////////////////////////////////////////////////////

func sendWarningEmail(data emailWarnings, recipients ...string) error {

	var err error

	t := template.New("email")

	if t, err = t.Parse(emailTemplate); err != nil {
		log.Error.Printf("Error parsing email template: %s", err.Error())
		return err
	}

	html := new(bytes.Buffer)

	if err = t.Execute(html, data); err != nil {
		log.Error.Printf("Error executing email template: %s", err.Error())
		return err
	}

	if err = emailsender.SendEmail(
		credentials.SMTPServer,
		587,
		credentials.SMTPAccount,
		credentials.SMTPPassword,
		"Water Softener",
		data.Subject,
		html.String(),
		"text/html",
		recipients...); err != nil {

		log.Error.Printf("Error sending email: %s", err.Error())
	}
	log.Info.Printf("Sent warning email to %d recipient(s)", len(recipients))
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

		log.Fatal.Panicf("Can't marshal json response: %s", err.Error())
	}

	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET, PUT")
	w.Header().Set("Access-Control-Allow-Headers", "Origin, Content-Type")
	w.Header().Set("Content-Type", "application/json")

	w.WriteHeader(status)

	if _, err = w.Write(body); err != nil {

		log.Error.Printf("Can't write response: %s", err.Error())
	}
}

//////////////////////////////////////////////////////////////////////

func respondOK[T any](w http.ResponseWriter, response *T) {

	sendResponse(w, http.StatusOK, response)
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

		log.Verbose.Printf("%s %s", r.Method, r.URL)
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

	log.Debug.Println("Database opened")

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

	log.Debug.Printf("Device ID %d", deviceAddress48)

	// get the readings

	log.Debug.Printf("From: %s, To: %s, Count: %d", from.Format(time.RFC3339), to.Format(time.RFC3339), count)

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
	log.Debug.Printf("Fetched %d rows", rows)
	response.Rows = rows
	return response, http.StatusOK, nil
}

//////////////////////////////////////////////////////////////////////

func doWarningEmail(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	checkSaltLevels()

	respondOK(w, &okResponse{Ok: "OK"})
}

//////////////////////////////////////////////////////////////////////

func getDeviceCount(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var db *sql.DB
	var err error

	if db, err = openDatabase(); err != nil {
		respondError(w, http.StatusInternalServerError, fmt.Sprintf("Error opening database: %s", err.Error()))
	}
	defer db.Close()

	var deviceCount uint64

	if err = db.QueryRow(`SELECT COUNT(*) FROM devices;`).Scan(&deviceCount); err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
	}
	respondOK(w, &getDeviceCountResponse{DeviceCount: deviceCount})
}

//////////////////////////////////////////////////////////////////////

func getDevice(db *sql.DB, deviceAddress string) (device, error) {

	row := db.QueryRow(`SELECT
							device_id,
							device_address,
							IFNULL(device_name, '-'),
							sensors.sensor_type,
							sensors.sensor_min_distance,
							sensors.sensor_max_distance,
							sensors.sensor_min_vbat,
							sensors.sensor_max_vbat,
							sensors.sensor_max_sigma,
							sensors.sensor_min_reflectance
						FROM devices
						INNER JOIN sensors
							ON devices.sensor_id = sensors.sensor_id
						WHERE device_address = ?`, deviceAddress)

	var d device

	if err := row.Scan(&d.Id,
		&d.Address,
		&d.Name,
		&d.SensorType,
		&d.MinDist,
		&d.MaxDist,
		&d.MinVbat,
		&d.MaxVbat,
		&d.MaxSigma,
		&d.MinReflectance); err != nil {

		return device{}, err
	}

	return d, nil
}

//////////////////////////////////////////////////////////////////////
// query for all the devices

func queryDevices(db *sql.DB) (response getDevicesResponse, status int, err error) {

	// get the devices

	var query *sql.Rows

	if query, err = db.Query(`SELECT
									device_id,
									device_address,
									IFNULL(device_name, '-'),
									sensors.sensor_type,
									sensors.sensor_min_distance,
									sensors.sensor_max_distance,
									sensors.sensor_min_vbat,
									sensors.sensor_max_vbat
								FROM devices
								INNER JOIN sensors
								ON devices.sensor_id = sensors.sensor_id;`); err != nil {

		return getDevicesResponse{}, http.StatusInternalServerError, err
	}

	// put the devices in an object for json output response

	response = getDevicesResponse{}

	rows := 0

	for query.Next() {
		var device device
		if err = query.Scan(&device.Id,
			&device.Address,
			&device.Name,
			&device.SensorType,
			&device.MinDist,
			&device.MaxDist,
			&device.MinVbat,
			&device.MaxVbat); err != nil {

			return getDevicesResponse{}, http.StatusInternalServerError, err
		}
		rows += 1
		response.Device = append(response.Device, device)
	}
	log.Debug.Printf("Fetched %d rows", rows)
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

	var errs = make([]string, 0)

	queryParams := r.URL.Query()

	// device is required

	var deviceAddress48 uint64

	// these are optional with defaults

	from := time.Time{}
	to := time.Now().Add(time.Hour * 24)
	count := uint64(1000)

	// scan the parameters

	if deviceAddress48, err = parseQueryParamUnsigned("device", 16, 48, queryParams); err != nil {
		errs = append(errs, err.Error())
	}

	if len(queryParams["from"]) != 0 {
		if from, err = parseQueryParamTime("from", queryParams); err != nil {
			errs = append(errs, err.Error())
		}
	}

	if len(queryParams["to"]) != 0 {
		if to, err = parseQueryParamTime("to", queryParams); err != nil {
			errs = append(errs, err.Error())
		}
	}

	if len(queryParams["count"]) != 0 {
		if count, err = parseQueryParamUnsigned("count", 10, 16, queryParams); err != nil {
			errs = append(errs, err.Error())
		} else if count > 10000 {
			errs = append(errs, fmt.Sprintf("count %d out of range, must be < 10000", count))
		}
	}

	// send error(s) if anything wrong

	if len(errs) != 0 {
		respondError(w, http.StatusBadRequest, errs...)
		return
	}

	var response getReadingsResponse
	var httpStatus int
	response, httpStatus, err = queryReadings(from, to, count, deviceAddress48)

	if err != nil {
		respondError(w, httpStatus, err.Error())
	}

	respondOK(w, &response)
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
	respondOK(w, &response)
}

//////////////////////////////////////////////////////////////////////
// add a reading to the database

func insertReading(device uint64, vbat uint64, distance_mm int32, flags uint64, rssi int64) (sleepCount int, err error) {

	log.Verbose.Printf("vbat: %d, reading: %d, flags: %d, device: %012x, rssi: %d, distance: %d", vbat, distance_mm, flags, device, rssi, distance_mm)

	var db *sql.DB

	if db, err = openDatabase(); err != nil {
		return 0, fmt.Errorf("error opening database: %s", err.Error())
	}
	defer db.Close()

	// get device id or insert new device

	deviceAddress := fmt.Sprintf("%012x", device)

	log.Debug.Printf("Get device id for %s", deviceAddress)

	var deviceAddress48 int64
	var insert sql.Result

	if err = db.QueryRow(`SELECT
							device_id, sleep_count
							FROM devices
							WHERE device_address = ?;`, deviceAddress).Scan(&deviceAddress48, &sleepCount); err != nil {

		if err == sql.ErrNoRows {

			log.Info.Printf("New device: %s", deviceAddress)

			if insert, err = db.Exec(`INSERT INTO devices
										(device_address) VALUES (?)`, deviceAddress); err == nil {

				deviceAddress48, err = insert.LastInsertId()
			}
		}
	}

	if err != nil {
		return 0, fmt.Errorf("database error adding device %s: %s", deviceAddress, err.Error())
	}

	log.Debug.Printf("Device ID %d", deviceAddress48)

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

	log.Debug.Printf("Reading ID %d", readingId)

	return sleepCount, nil
}

//////////////////////////////////////////////////////////////////////
// add a reading to the database, this time taking into account the sensor_type

func insertReading_ext(device uint64, sensor_type string, vbat uint64, distance_mm int32, sigma uint16, reflectance uint8, flags uint64, rssi int64) (sleepCount int, err error) {

	log.Verbose.Printf("ext: vbat: %d, sensor_type: %s, distance: %d, flags: %d, device: %012x, rssi: %d", vbat, sensor_type, distance_mm, flags, device, rssi)

	var db *sql.DB

	if db, err = openDatabase(); err != nil {
		return 0, fmt.Errorf("error opening database: %s", err.Error())
	}
	defer db.Close()

	// find sensor_type in sensors

	// get device id or insert new device

	deviceAddress := fmt.Sprintf("%012x", device)

	log.Debug.Printf("Get device id for %s", deviceAddress)

	var deviceAddress48 int64
	var insert sql.Result
	var device_sensor_type string

	log.Debug.Printf("A")

	if err = db.QueryRow(`SELECT
							device_id, sleep_count, sensors.sensor_type
							FROM devices
							INNER JOIN
								sensors ON
									devices.sensor_id = sensors.sensor_id
							WHERE device_address = ?;`, deviceAddress).Scan(&deviceAddress48, &sleepCount, &device_sensor_type); err != nil {

		if err == sql.ErrNoRows {

			log.Info.Printf("New device: %s", deviceAddress)

			var sensor_id uint64

			if err = db.QueryRow(`SELECT sensor_id from sensors WHERE sensor_type=?;`, sensor_type).Scan(&sensor_id); err != nil {

				return 0, fmt.Errorf("sensor_type '%s' not found: %s", sensor_type, err.Error())
			}

			if insert, err = db.Exec(`INSERT INTO
										devices(sensor_id, device_address)
										VALUES (?, ?)`, sensor_id, deviceAddress); err == nil {

				deviceAddress48, err = insert.LastInsertId()
				device_sensor_type = sensor_type
				sleepCount = 21600
			}
		}
	}

	log.Debug.Printf("B")

	if err != nil {
		return 0, fmt.Errorf("database error adding device %s: %s", deviceAddress, err.Error())
	}

	if strings.Compare(device_sensor_type, sensor_type) != 0 {
		return 0, fmt.Errorf("wrong device type (should be %s, got %s)", device_sensor_type, sensor_type)
	}

	log.Debug.Printf("Device ID %d", deviceAddress48)

	// add the reading

	var readingId int64

	if insert, err = db.Exec(`INSERT INTO readings
								(device_id,
								reading_vbat,
								reading_distance,
								reading_sigma,
								reading_reflectance,
								reading_flags,
								reading_rssi,
								reading_timestamp)
                               VALUES (?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP());`,
		deviceAddress48,
		vbat,
		distance_mm,
		sigma,
		reflectance,
		flags,
		rssi); err == nil {

		readingId, err = insert.LastInsertId()
	}

	if err != nil {
		log.Error.Printf("Huh? %s", err.Error())
		return 0, fmt.Errorf("database error adding reading: %s", err.Error())
	}

	log.Debug.Printf("Reading ID %d", readingId)

	return sleepCount, nil
}

//////////////////////////////////////////////////////////////////////
// common reading parameters:
// vbat uint64			vbat in 100mV units (i.e. 843 = 8.43 volts)
// distance uint64		distance in counter units (see resolution)
// device uint64		device mac address in hex
// flags uint64			flags for reference
// rssi int64			WiFi strength in dBm

func getReadingParams(r *http.Request, params httprouter.Params) (readingParams, []string) {

	var err error

	var p readingParams

	var errs = make([]string, 0)

	query := r.URL.Query()

	if p.vbat, err = parseQueryParamUnsigned("vbat", 10, 16, query); err != nil {
		errs = append(errs, err.Error())
	}

	if p.distance, err = parseQueryParamUnsigned("distance", 10, 16, query); err != nil {
		errs = append(errs, err.Error())
	}

	if p.device, err = parseQueryParamUnsigned("device", 16, 48, query); err != nil {
		errs = append(errs, err.Error())
	}

	if p.flags, err = parseQueryParamUnsigned("flags", 10, 16, query); err != nil {
		errs = append(errs, err.Error())
	}

	if p.rssi, err = parseQueryParamSigned("rssi", 10, 8, query); err != nil {
		errs = append(errs, err.Error())
	}
	return p, errs
}

//////////////////////////////////////////////////////////////////////
// put a new reading v1 (for distance_sensor_v1)
// extra parameter:
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

func putReading_v1(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	var err error

	// parse the query string

	queryParams, errs := getReadingParams(r, params)

	var resolution int64 = 0

	query := r.URL.Query()

	if resolution, err = parseQueryParamSigned("resolution", 10, 32, query); err != nil {
		errs = append(errs, err.Error())
	}

	if len(errs) != 0 {
		respondError(w, http.StatusBadRequest, errs...)
		return
	}

	// counter counts at this many per second (eg 8000000)

	time_scale := 48000000.0 / float32(resolution)

	const speed_of_sound_mm_per_sec = 346000.0

	distance_mm := int32(float32(queryParams.distance) / time_scale * speed_of_sound_mm_per_sec / 2.0)

	var sleepCount int
	sleepCount, err = insertReading(queryParams.device, queryParams.vbat, distance_mm, queryParams.flags, queryParams.rssi)

	if err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}
	respondOK(w, &putReadingResponse{Settings: sensorSettings{SleepCount: sleepCount}})
}

//////////////////////////////////////////////////////////////////////
// put a new reading v2 for distance_sensor_v2
// no resolution (it's already in mm)
// sleep_count returned is in seconds, not some dodgy other thing

func putReading_v2(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	queryParams, errs := getReadingParams(r, params)

	if len(errs) != 0 {
		respondError(w, http.StatusBadRequest, errs...)
		return
	}

	sleepCountSeconds, err := insertReading(queryParams.device, (queryParams.vbat+5)/10, int32(queryParams.distance), queryParams.flags, queryParams.rssi)

	if err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}
	respondOK(w, &putReadingResponse{Settings: sensorSettings{SleepCount: sleepCountSeconds}})
}

//////////////////////////////////////////////////////////////////////
// post a new reading v3 for distance_sensor_v2
// post data is binary 8x8 distance and 8x8 sigma

func postReading_v3(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	queryParams, errs := getReadingParams(r, params)

	var err error
	var body_length int64

	if body_length, err = parseQueryParamSigned("body_length", 10, 32, r.URL.Query()); err != nil {
		errs = append(errs, err.Error())
	}

	if !r.URL.Query().Has("sensor_type") {
		errs = append(errs, "missing query parameter: sensor_type")
	}

	// check body_length is sane here, say, less than, oh, I dunno, 2KB
	if body_length <= 0 || body_length > 2047 {
		errs = append(errs, fmt.Sprintf("body length %d is out of range (1..2048)", body_length))
	}

	if len(errs) != 0 {
		respondError(w, http.StatusBadRequest, errs...)
		return
	}

	sensor_type := r.URL.Query().Get("sensor_type")

	// what's in it?

	// 64 x uint16 of sigma_mm (seems to be some confidence thing?)
	// 64 x int16 of distance (mm)
	// 64 x uint8 of reflectance (some unit)

	// so body_length should be 2 * 64 + 2 * 64 + 1 * 64 = 320

	good_body_length := 2*64 + 2*64 + 1*64

	var body []byte

	if body, err = io.ReadAll(r.Body); err != nil {
		respondError(w, http.StatusBadRequest, fmt.Sprintf("Body length should be %d", good_body_length))
	}

	if len(body) != int(body_length) {
		respondError(w, http.StatusBadRequest)
	}

	log.Debug.Printf("BODY[0] = %d", body[0])

	// A valid reading has reflectance > ?? and sigma < ??, find the furthest valid reading

	// if no valid entries found, it's empty, put in a [max_distance?] reading and set the `error_reading_distance` flag (1<<4)

	// get these from the database

	var db *sql.DB

	if db, err = openDatabase(); err != nil {

		respondError(w, http.StatusInternalServerError, fmt.Sprintf("error opening database: %s", err.Error()))
	}
	defer db.Close()

	deviceAddress := fmt.Sprintf("%012x", queryParams.device)

	var dev device

	dev, err = getDevice(db, deviceAddress)

	if err != nil && err != sql.ErrNoRows {

		respondError(w, http.StatusBadRequest, fmt.Sprintf("error getting device %s: %s", deviceAddress, err.Error()))
	}

	// offsets into the post body of the different arrays
	sigma_idx := 0
	distance_idx := 128    // 64 * sizeof(uint16)
	reflectance_idx := 256 // (64 + 64) * sizeof(uint16)

	var nearest uint16 = dev.MaxDist
	var best_sigma uint16
	var best_reflectance uint8

	// 8x8 resolution = 64 entries to check

	var sigma_log string
	var distance_log string
	var reflectance_log string

	var sep string = ""

	log.Debug.Print("SIGMA  :  DISTANCE  :  REFLECTANCE")

	for i := 0; i < 64; i++ {

		sigma_mm := uint16(body[sigma_idx]) | (uint16(body[sigma_idx+1]) << 8)

		distance_mm := uint16(body[distance_idx]) | (uint16(body[distance_idx+1]) << 8)

		reflectance := body[reflectance_idx]

		if sigma_mm < dev.MaxSigma && reflectance > dev.MinReflectance && distance_mm < nearest {
			nearest = distance_mm
			best_sigma = sigma_mm
			best_reflectance = reflectance
		}

		sigma_log = fmt.Sprintf("%s%s% 5d", sigma_log, sep, sigma_mm)
		distance_log = fmt.Sprintf("%s%s% 5d", distance_log, sep, distance_mm)
		reflectance_log = fmt.Sprintf("%s%s% 4d", reflectance_log, sep, reflectance)

		sep = " "

		if (i & 7) == 7 {

			log.Debug.Printf("%s  :  %s  :  %s", sigma_log, distance_log, reflectance_log)
			sigma_log = ""
			distance_log = ""
			reflectance_log = ""
			sep = ""
		}

		sigma_idx += 2       // sizeof(uint16)
		distance_idx += 2    // sizeof(uint16)
		reflectance_idx += 1 // sizeof(uint8)
	}

	flags := queryParams.flags

	if nearest < dev.MinDist || nearest >= dev.MaxDist {

		nearest = dev.MaxDist
		flags = flags | (1 << 4) // error_reading_distance
	}

	sleepCountSeconds, err := insertReading_ext(queryParams.device, sensor_type, (queryParams.vbat+5)/10, int32(nearest), best_sigma, best_reflectance, flags, queryParams.rssi)

	if err != nil {
		respondError(w, http.StatusInternalServerError, err.Error())
		return
	}
	respondOK(w, &putReadingResponse{Settings: sensorSettings{SleepCount: sleepCountSeconds}})
}

//////////////////////////////////////////////////////////////////////
// Check if salt level below threshold for each device, send warning emails to registered recipients if it is

func checkSaltLevels() {

	log.Info.Printf("Checking Salt Levels daily")

	// open the database

	var db *sql.DB
	var err error

	if db, err = openDatabase(); err != nil {
		log.Error.Printf("Error opening database: %s", err.Error())
		return
	}
	defer db.Close()

	// get all the devices

	devices, _, err := queryDevices(db)

	if err != nil {
		log.Error.Printf("Can't query devices!? (%s)", err.Error())
		return
	}

	// for each device

	for _, device := range devices.Device {

		var deviceAddress48 uint64
		var err error

		// check the device address is valid 48 bit hex (e.g. a mac address)

		if deviceAddress48, err = strconv.ParseUint(device.Address, 16, 48); err != nil {

			log.Error.Printf("Device %s has bad address: %s", device.Address, err.Error())
			continue
		}

		// get the most recent reading for this device

		log.Verbose.Printf("Notifications for device %s (%s) (id = %d)?", device.Address, device.Name, device.Id)

		if reading, _, err := queryReadings(time.Time{}, time.Now().Add(time.Hour*24), 1, deviceAddress48); err != nil {

			log.Error.Printf("Error querying readings for device %s : %s", device.Address, err.Error())

		} else if reading.Rows == 0 {

			log.Info.Printf("No readings for device %s (%s)", device.Address, device.Name)

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

					log.Info.Printf("No notifications for device %s", device.Address)
					continue
				}
				log.Error.Printf("Error getting notifications for %s: %s", device.Address, err.Error())
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

					log.Error.Printf("Error reading notification data for device %s", device.Address)
					continue
				}

				warnings := []string{}

				// warn about salt level if required

				saltPercent := device.saltDistanceToPercent(reading.Distance[0])

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

					log.Info.Printf("No warning for %s on %s(%s)", email, device.Name, device.Address)
					continue
				}

				// send the warning email to this notification recipient

				log.Verbose.Printf("Sending alert email to %s for device %s (%s)", email, device.Name, device.Address)

				emailData := emailWarnings{
					Subject:  "Alert from the Water Softener",
					Header:   "Warning!",
					From:     device.Name,
					FromLink: fmt.Sprintf("https://vibue.com/d/d8cfdc5e-d585-4988-9547-293aae64c29b/salt-sensor?orgId=1&var-sensor=%s", device.Address),
					Warnings: warnings,
					Footer:   "This email is from the Salt Sensor Server",
				}
				sendWarningEmail(emailData, email)
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////

func main() {

	// command line parameters

	var credentialsFilename string
	var logLevelName string

	flag.StringVar(&credentialsFilename, "credentials", "", "Specify credentials filename (required)")
	flag.StringVar(&logLevelName, "log_level", log.LogLevelNames[log.LogLevel], fmt.Sprintf("Specify log level (%s)", strings.Join(log.LogLevelNames[:], "|")))
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

	if err := log.Init(logLevelName); err != nil {
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

	log.Info.Println()
	log.Info.Printf("########## SALT SENSOR SERVICE BEGINS ##########")

	log.Verbose.Printf("Log level: %s", log.LogLevelNames[log.LogLevel])
	log.Verbose.Printf("Credentials file: %s", credentialsFilename)

	// database config

	log.Verbose.Printf("Database is '%s'", credentials.DBDatabase)
	log.Verbose.Printf("Username is '%s'", credentials.DBUsername)

	log.Verbose.Printf("SMTP Server is '%s'", credentials.SMTPServer)
	log.Verbose.Printf("SMTP Account is '%s'", credentials.SMTPAccount)

	dbDSNString = (&mysql.Config{
		User:                 credentials.DBUsername,
		Passwd:               credentials.DBPassword,
		Net:                  "tcp",
		Addr:                 "localhost:3306",
		DBName:               credentials.DBDatabase,
		AllowNativePasswords: true,
		ParseTime:            true,
	}).FormatDSN()

	// daily alarm to check levels and send alert email if necessary

	var alarmChannel chan dailyalarm.AlarmNotify
	var err error

	if alarmChannel, err = dailyalarm.Set(6, 0, checkSaltLevels); err != nil {

		log.Error.Printf("Error setting up daily alarm: %s", err.Error())
	}

	router := httprouter.New()

	router.NotFound = http.HandlerFunc(notFound)
	router.MethodNotAllowed = http.HandlerFunc(methodNotAllowed)

	router.PUT("/reading", logged(putReading_v1))
	router.PUT("/reading2", logged(putReading_v2))

	router.GET("/readings", logged(getReadings))
	router.GET("/devices", logged(getDevices))
	router.GET("/numdevices", logged(getDeviceCount))
	router.GET("/checksaltlevels", logged(doWarningEmail))

	router.POST("/reading3", logged(postReading_v3))

	log.Verbose.Printf("HTTP server starting")

	err = http.ListenAndServe(":5002", router)

	if errors.Is(err, http.ErrServerClosed) {
		log.Verbose.Printf("HTTP server closed")

	} else if err != nil {
		log.Error.Printf("Error starting HTTP server: %s", err.Error())
	}

	// quit the daily alarm

	alarmChannel <- dailyalarm.AlarmQuit

	log.Info.Printf("---------- SALT SENSOR SERVICE ENDS ----------")
}
