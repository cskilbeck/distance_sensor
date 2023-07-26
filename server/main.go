//////////////////////////////////////////////////////////////////////
// Salt Sensor Server

package main

import (
    "database/sql"
    "encoding/json"
    "errors"
    "flag"
    "fmt"
    "github.com/go-sql-driver/mysql"
    "io"
    "log"
    "net/http"
    "os"
    "strconv"
)

//////////////////////////////////////////////////////////////////////
// HTTP response body

type Response struct {
    Status      string `json:"status"`
    Error       string `json:"error"`
    Description string `json:"description"`
}

//////////////////////////////////////////////////////////////////////
// Database credentials

type Credentials struct {
    Username string `json:"username"`
    Password string `json:"password"`
}

//////////////////////////////////////////////////////////////////////

var db *sql.DB

var db_credentials Credentials

var Debug *log.Logger
var Verbose *log.Logger
var Info *log.Logger
var Warning *log.Logger
var Error *log.Logger

//////////////////////////////////////////////////////////////////////

func init_log_level(log_level_name string) {

    log_names := [5]string{
        "debug",
        "verbose",
        "info",
        "warning",
        "error",
    }

    var log_handle []io.Writer = make([]io.Writer, 5)

    var log_level = 2 // level is info by default

    found := true

    if len(log_level_name) != 0 {

        found = false

        for i, name := range log_names {

            if log_level_name == name {

                log_level = i
                found = true
                break
            }
        }
    }

    for i := 0; i < log_level; i++ {

        log_handle[i] = io.Discard
    }

    for i := log_level; i < 5; i++ {

        log_handle[i] = os.Stdout
    }

    Debug = log.New(log_handle[0], "DEBUG:", log.Ldate|log.Ltime|log.Lshortfile)
    Verbose = log.New(log_handle[1], "VERBOSE:", log.Ldate|log.Ltime|log.Lshortfile)
    Info = log.New(log_handle[2], "INFO:", log.Ldate|log.Ltime|log.Lshortfile)
    Warning = log.New(log_handle[3], "WARNING:", log.Ldate|log.Ltime|log.Lshortfile)
    Error = log.New(log_handle[4], "ERROR:", log.Ldate|log.Ltime|log.Lshortfile)

    if !found {
        Error.Printf("Unknown log level \"%s\", defaulting to log level \"%s\"", log_level_name, log_names[log_level])
    }

    Debug.Printf("Log level set to \"%s\"", log_names[log_level])
}

//////////////////////////////////////////////////////////////////////

func load_credentials(credentials_filename string) {

    if len(credentials_filename) == 0 {

        log.Fatalf("Credentials filename not specified, use -credentials <filename>")
    }

    if content, err := os.ReadFile(credentials_filename); err != nil {

        log.Fatalf("Can't load %s: %s", credentials_filename, err)

    } else if err := json.Unmarshal(content, &db_credentials); err != nil {

        log.Fatalf("Can't parse %s: %s", credentials_filename, err)
    }
}

//////////////////////////////////////////////////////////////////////

func open_database() {

    cfg := mysql.Config{
        User:                 db_credentials.Username,
        Passwd:               db_credentials.Password,
        Net:                  "tcp",
        Addr:                 "localhost:3306",
        DBName:               "salt_sensor",
        AllowNativePasswords: true,
    }

    Debug.Printf("Opening database %s with username %s", cfg.DBName, cfg.User)

    var err error
    db, err = sql.Open("mysql", cfg.FormatDSN())
    if err != nil {
        log.Fatalf("Can't open database, %s", err)
    }
    Debug.Println("Database opened")
}

//////////////////////////////////////////////////////////////////////

func close_database() {
    Debug.Println("Closing database")
    db.Close()
}

//////////////////////////////////////////////////////////////////////

func get_device_id(name string) (id int64, err error) {

    Debug.Printf("Get device id for %s\n", name)

    _, err = db.Exec(`INSERT INTO devices (device_name)
                              SELECT * FROM (SELECT ?) AS tmp
                              WHERE NOT EXISTS
                              (SELECT * FROM devices WHERE device_name=?) LIMIT 1;`, name, name)
    if err != nil {
        Error.Printf("Device insert error %s", err)
        return 0, err
    }

    var device_id int64

    if err := db.QueryRow(`SELECT device_id FROM devices WHERE device_name = ?;`, name).Scan(&device_id); err != nil {

        Error.Printf("Device lookup error %s", err)
        return 0, err
    }
    return device_id, nil
}

//////////////////////////////////////////////////////////////////////

func add_reading(device_id int64, vbat uint16, distance uint16) (id int64, err error) {

    res, err := db.Exec(`INSERT INTO readings (device_id, reading_vbat, reading_distance, reading_timestamp)
                         VALUES (?,?,?,CURRENT_TIMESTAMP());`, device_id, vbat, distance)

    if err == nil {
        return res.LastInsertId()
    } else {
        return 0, err
    }
}

//////////////////////////////////////////////////////////////////////

var method_not_allowed = Response{

    Status: "error",
    Error:  "Method not allowed",
}

var missing_query_paramters = Response{

    Status: "error",
    Error:  "Missing query parameter(s)",
}

var bad_parameter = Response{

    Status: "error",
    Error:  "Bad parameter",
}

var device_id_error = Response{

    Status: "error",
    Error:  "Can't add or find device in database",
}

var reading_id_error = Response{

    Status: "error",
    Error:  "Can't add reading to database",
}

var json_ok = Response{

    Status: "OK",
}

//////////////////////////////////////////////////////////////////////

func get_json(r Response) []byte {
    s, err := json.Marshal(r)
    if err == nil {
        return s
    }
    return []byte(`{"huh":"say wha?"}`)
}

//////////////////////////////////////////////////////////////////////

func http_reading_handler(w http.ResponseWriter, r *http.Request) {

    Debug.Printf("GOT a GET!")

    if r.Method != "GET" {
        w.WriteHeader(http.StatusMethodNotAllowed)
        w.Write(get_json(method_not_allowed))
        return
    }

    vbat_param := r.URL.Query()["vbat"]
    distance_param := r.URL.Query()["distance"]
    device_param := r.URL.Query()["device"]

    if len(vbat_param) != 1 || len(distance_param) != 1 || len(device_param) != 1 {
        w.WriteHeader(http.StatusBadRequest)
        w.Write(get_json(missing_query_paramters))
        return
    }

    const bad_param_fmt = "Can't parse %s for parameter %s"

    vbat, err := strconv.ParseUint(vbat_param[0], 10, 16)
    if err != nil {
        r := bad_parameter
        r.Description = fmt.Sprintf(bad_param_fmt, vbat_param[0], "vbat")
        w.WriteHeader(http.StatusBadRequest)
        w.Write(get_json(r))
        return
    }

    distance, err := strconv.ParseUint(distance_param[0], 10, 16)
    if err != nil {
        var r = bad_parameter
        r.Description = fmt.Sprintf(bad_param_fmt, distance_param[0], "distance")
        w.WriteHeader(http.StatusBadRequest)
        w.Write(get_json(r))
        return
    }

    device, err := strconv.ParseUint(device_param[0], 16, 48)
    if err != nil {
        var r = bad_parameter
        r.Description = fmt.Sprintf(bad_param_fmt, device_param[0], "device")
        w.WriteHeader(http.StatusBadRequest)
        w.Write(get_json(r))
        return
    }

    device_name := fmt.Sprintf("%012x", device)

    open_database()
    defer close_database()

    device_id, err := get_device_id(device_name)
    if err != nil {
        w.WriteHeader(http.StatusInternalServerError)
        w.Write(get_json(device_id_error))
        return
    }
    Debug.Printf("Device connects: ID %d\n", device_id)

    reading_id, err := add_reading(device_id, uint16(vbat), uint16(distance))
    if err != nil {
        w.WriteHeader(http.StatusBadRequest)
        w.Write(get_json(reading_id_error))
        return
    }
    Debug.Printf("Reading ID %d\n", reading_id)
    w.WriteHeader(http.StatusOK)
    w.Write(get_json(json_ok))
}

//////////////////////////////////////////////////////////////////////

func main() {

    var credentials_filename string
    var log_level_name string

    flag.StringVar(&credentials_filename, "credentials", "", "Where is the credentials filename")
    flag.StringVar(&log_level_name, "log_level", "info", "Specify log level (debug|verbose|info|warning|error)")
    flag.Parse()

    init_log_level(log_level_name)

    load_credentials(credentials_filename)

    Info.Printf("Serving HTTP...\n")

    http.HandleFunc("/reading", http_reading_handler)

    err := http.ListenAndServe("0.0.0.0:5002", nil)

    if errors.Is(err, http.ErrServerClosed) {

        Info.Printf("Server closed\n")

    } else if err != nil {

        log.Fatal("Error starting server: %s\n", err)
    }
}
