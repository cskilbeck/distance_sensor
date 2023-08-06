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
    "net/url"
    "os"
    "strconv"
    "strings"
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

var db *sql.DB

var db_credentials Credentials

var Debug *log.Logger
var Verbose *log.Logger
var Info *log.Logger
var Warning *log.Logger
var Error *log.Logger

//////////////////////////////////////////////////////////////////////

func Err(format string, args ...any) error {

    return errors.New(fmt.Sprintf(format, args...))
}

//////////////////////////////////////////////////////////////////////

func set_log_level(log_level_name string) (e error) {

    log_names := [5]string{
        "debug",
        "verbose",
        "info",
        "warning",
        "error",
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

    var loggers []*log.Logger = make([]*log.Logger, 5)

    for i := 0; i < log_level; i++ {
        loggers[i] = log.New(io.Discard, "", 0)
    }

    for i := log_level; i < 5; i++ {
        loggers[i] = log.New(os.Stdout, strings.ToUpper(log_names[i])+": ", 0)
    }

    Debug = loggers[0]
    Verbose = loggers[1]
    Info = loggers[2]
    Warning = loggers[3]
    Error = loggers[4]

    if !found {
        return Err("Unknown log level \"%s\"", log_level_name)
    }

    Debug.Printf("Log level set to \"%s\"", log_names[log_level])

    return nil
}

//////////////////////////////////////////////////////////////////////

func load_credentials(credentials_filename string) (e error) {

    if len(credentials_filename) == 0 {

        return errors.New("Missing credentials filename")
    }

    var err error
    var content []byte

    if content, err = os.ReadFile(credentials_filename); err != nil {

        return Err("Loading credentials, can't %s", err)
    }

    if err = json.Unmarshal(content, &db_credentials); err != nil {

        return Err("Error parsing %s: %s", credentials_filename, err)
    }

    Verbose.Printf("Loaded credentials file %s", credentials_filename)
    return nil
}

//////////////////////////////////////////////////////////////////////

func open_database() (e error) {

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
        return Err("Can't open database, %s", err)
    }
    Debug.Println("Database opened")
    return nil
}

//////////////////////////////////////////////////////////////////////

func close_database() {

    Debug.Println("Closing database")
    db.Close()
}

//////////////////////////////////////////////////////////////////////

func get_device_id(name string) (id int64, err error) {

    Debug.Printf("Get device id for %s", name)

    _, err = db.Exec(`INSERT INTO devices (device_address)
                              SELECT * FROM (SELECT ?) AS tmp
                              WHERE NOT EXISTS
                              (SELECT * FROM devices WHERE device_address=?) LIMIT 1;`, name, name)
    if err != nil {

        Error.Printf("Device insert error %s", err)
        return 0, err
    }

    var device_id int64

    if err := db.QueryRow(`SELECT device_id FROM devices WHERE device_address = ?;`, name).Scan(&device_id); err != nil {

        Error.Printf("Device lookup error %s", err)
        return 0, err
    }
    return device_id, nil
}

//////////////////////////////////////////////////////////////////////

func add_reading(device_id int64, vbat uint16, distance uint16, flags uint16) (int64, error) {

    res, err := db.Exec(`INSERT INTO readings (device_id, reading_vbat, reading_distance, reading_flags, reading_timestamp)
                         VALUES (?,?,?,?,CURRENT_TIMESTAMP());`, device_id, vbat, distance, flags)

    if err == nil {
        return res.LastInsertId()
    } else {
        Error.Printf("add_reading: %s", err.Error())
        return 0, err
    }
}

//////////////////////////////////////////////////////////////////////

func get_json(r Response) []byte {

    if s, err := json.MarshalIndent(r, "", "  "); err == nil {
        return s
    }

    return []byte(`{"huh":"say wha?"}`)
}

//////////////////////////////////////////////////////////////////////

func responds(status string, errs []string) []byte {

    var r Response
    r.Status = status
    r.Info = errs
    return get_json(r)
}

//////////////////////////////////////////////////////////////////////

func respondf(status string, err string, args ...any) []byte {

    var r Response
    r.Status = status
    r.Info = make([]string, 1)
    r.Info[0] = fmt.Sprintf(err, args...)
    return get_json(r)
}

//////////////////////////////////////////////////////////////////////

func parse_param(name string, base int, bits int, values *url.Values) (uint64, error) {

    param := (*values)[name]

    if len(param) < 1 {
        return 0, Err("Missing parameter %s", name)
    }

    if len(param) > 1 {
        return 0, Err("Duplicate parameter %s", name)
    }

    v, err := strconv.ParseUint(param[0], base, bits)

    if err != nil {
        return 0, Err("Bad value for '%s' (%s)", name, err.Error())
    }
    return v, nil
}

//////////////////////////////////////////////////////////////////////

func http_reading_handler(w http.ResponseWriter, r *http.Request) {

    w.Header().Set("Content-Type", "application/json")

    if r.Method != "GET" {
        w.WriteHeader(http.StatusMethodNotAllowed)
        w.Write(respondf("error", "Method %s not allowed", r.Method))
        return
    }

    Verbose.Printf("GET %s", r.URL)

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
        w.Write(responds("error", errors))
        return
    }

    Debug.Printf("vbat: %d, distance: %d, flags: %d, device: %012x", vbat, distance, flags, device)

    if err := open_database(); err != nil {
        w.WriteHeader(http.StatusInternalServerError)
        w.Write(respondf("error", "Error opening database: %s", err.Error()))
        return
    }
    defer close_database()

    device_name := fmt.Sprintf("%012x", device)

    device_id, err := get_device_id(device_name)
    if err != nil {
        w.WriteHeader(http.StatusInternalServerError)
        w.Write(respondf("error", "Error accessing database for device %s: %s", device_name, err.Error()))
        return
    }
    Debug.Printf("Device ID %d", device_id)

    reading_id, err := add_reading(device_id, uint16(vbat), uint16(distance), uint16(flags))
    if err != nil {
        w.WriteHeader(http.StatusInternalServerError)
        w.Write(respondf("error", "Error adding reading to database: %s", err.Error()))
        return
    }
    Debug.Printf("Reading ID %d", reading_id)
    w.WriteHeader(http.StatusOK)
    w.Write(respondf("OK", "reading id %d", reading_id))
    Debug.Printf("OK")
}

//////////////////////////////////////////////////////////////////////

func main() {

    var credentials_filename string
    var log_level_name string

    flag.StringVar(&credentials_filename, "credentials", "", "Specify credentials filename (required)")
    flag.StringVar(&log_level_name, "log_level", "info", "Specify log level (debug|verbose|info|warning|error)")
    flag.Parse()

    errs := make([]error, 0)

    if err := set_log_level(log_level_name); err != nil {
        errs = append(errs, err)
    }

    if err := load_credentials(credentials_filename); err != nil {
        errs = append(errs, err)
    }

    if len(errs) != 0 {

        for _, e := range errs {
            fmt.Printf("%s", e.Error())
        }

        fmt.Printf("\nUsage:\n")
        flag.PrintDefaults()
        fmt.Println()

        exit_code := 0
        if len(errs) != 0 {
            exit_code = 1
        }
        os.Exit(exit_code)
    }

    Info.Printf("Serving HTTP...")

    http.HandleFunc("/reading", http_reading_handler)

    err := http.ListenAndServe("0.0.0.0:5002", nil)

    if errors.Is(err, http.ErrServerClosed) {

        Info.Printf("Server closed")

    } else if err != nil {

        log.Fatal("Error starting server: %s", err)
    }
}
