//////////////////////////////////////////////////////////////////////
// Sort of STUN type websockets thing which passes Alexa message to devices

package main

import (
    "bytes"
    "fmt"
    "io/ioutil"
    "log"
    "net/http"
    "os"
    "time"

    "github.com/gorilla/websocket"
    "github.com/valyala/fastjson"
)

//////////////////////////////////////////////////////////////////////
// A DeviceConnection is admin for a connected device

type DeviceConnection struct {
    deviceId  string          // redundant but handy
    socket    *websocket.Conn // current socket
    messages  chan []byte     // AlexaMessages
    murder    chan bool       // please die
    graveyard chan bool       // I died
}

//////////////////////////////////////////////////////////////////////
// Send this back to a freshly connected device

type WelcomeMessage struct {
    DeviceID string `json:"device_id"`
    Message  string `json:"message"`
}

//////////////////////////////////////////////////////////////////////
// Punner for http server discrimination

type AlexaMessage struct {
}

//////////////////////////////////////////////////////////////////////

// map of currently connected devices
var clients = make(map[string]*DeviceConnection)

// websocket admin
var upgrader = websocket.Upgrader{}

// logging
var debugHandle = os.Stdout
var verboseHandle = os.Stdout
var infoHandle = os.Stdout
var warningHandle = os.Stdout
var errorHandle = os.Stdout

var Debug = log.New(debugHandle, "DEBUG:   ", log.Ldate|log.Ltime|log.Lshortfile)
var Verbose = log.New(verboseHandle, "VERBOSE: ", log.Ldate|log.Ltime|log.Lshortfile)
var Info = log.New(infoHandle, "INFO:    ", log.Ldate|log.Ltime|log.Lshortfile)
var Warning = log.New(warningHandle, "WARNING: ", log.Ldate|log.Ltime|log.Lshortfile)
var Error = log.New(errorHandle, "ERROR:   ", log.Ldate|log.Ltime|log.Lshortfile)

//////////////////////////////////////////////////////////////////////

func GetArray(v *fastjson.Value, name string) ([]*fastjson.Value, error) {
    value := v.Get(name)
    if value == nil {
        return nil, fmt.Errorf("Can't find key %s", name)
    }
    data, err := value.Array()
    if err != nil {
        return nil, fmt.Errorf("%s isn't an Array")
    }
    return data, nil
}

//////////////////////////////////////////////////////////////////////

func GetBool(v *fastjson.Value, name string) (bool, error) {
    value := v.Get(name)
    if value == nil {
        return false, fmt.Errorf("Can't find key %s", name)
    }
    data, err := value.Bool()
    if err != nil {
        return false, fmt.Errorf("%s isn't a Bool")
    }
    return data, nil
}

//////////////////////////////////////////////////////////////////////

func GetInt(v *fastjson.Value, name string) (int, error) {
    value := v.Get(name)
    if value == nil {
        return 0, fmt.Errorf("Can't find key %s", name)
    }
    data, err := value.Int()
    if err != nil {
        return 0, fmt.Errorf("%s isn't an Int")
    }
    return data, nil
}

//////////////////////////////////////////////////////////////////////

func GetString(v *fastjson.Value, name string) (string, error) {
    value := v.Get(name)
    if value == nil {
        return "", fmt.Errorf("Can't find key %s", name)
    }
    data, err := value.StringBytes()
    if err != nil {
        return "", fmt.Errorf("%s isn't a String")
    }
    return string(data), nil
}

//////////////////////////////////////////////////////////////////////
// A device connects! Set up a DeviceConnection

func (m *DeviceConnection) ServeHTTP(w http.ResponseWriter, r *http.Request) {

    Info.Printf("connection from %s", r.RemoteAddr)

    // is it a websocket connecting?
    ws, err := upgrader.Upgrade(w, r, nil)
    if err != nil {
        Error.Printf(err.Error())
        return
    }

    // get the json hello from the device
    _, data, err := ws.ReadMessage()
    if err != nil {
        Error.Printf("Error reading DeviceID message: %s", err.Error())
        return
    }

    // parse it
    var p fastjson.Parser
    v, err := p.Parse(string(bytes.Trim(data, "\x00")))
    if err != nil {
        Error.Printf("Error parsing json: %s", err.Error())
        return
    }

    // check for device_id field
    deviceId, err := GetString(v, "device_id")
    if err != nil {
        Error.Printf("Can't get DeviceID: %s", err.Error())
        return
    }
    Info.Printf("Hello %v\n", deviceId)

    // ACK the device
    msg := WelcomeMessage{deviceId, "WELCOME TO BIGSERV!"}
    err = ws.WriteJSON(&msg)
    if err != nil {
        Error.Printf("%v", err.Error())
        return
    }

    // is it a new device?
    device, found := clients[deviceId]

    // yes, setup socket and channel for feeder
    if found {
        Info.Printf("Device %s reconnecting", deviceId)
        clients[deviceId].end()
    }
    device = &DeviceConnection{deviceId, ws, make(chan []byte), make(chan bool), make(chan bool)}
    clients[deviceId] = device
    go device.handleMessages()
}

//////////////////////////////////////////////////////////////////////
// Send Alexa messages over to the device feeders

func (m *AlexaMessage) ServeHTTP(w http.ResponseWriter, r *http.Request) {

    body, err := ioutil.ReadAll(r.Body)
    if err != nil {
        Error.Printf("ReadBody: %v", err.Error())
        return
    }
    deviceId := r.Header.Get("X-DeviceID")
    if len(deviceId) == 0 {
        Error.Printf("No X-DeviceID header")
        return
    }

    if _, ok := clients[deviceId]; !ok {
        Error.Printf("Device %s not connected, can't send %s", deviceId, string(body))
        return
    }
    clients[deviceId].messages <- body
}

//////////////////////////////////////////////////////////////////////
// device feeder, wait for alexa messages and send to device websocket

const (
    // Time allowed to write a message to the peer.
    writeWait = 5 * time.Second

    // Time allowed to read the next pong message from the peer.
    pongWait = 60 * time.Second

    // Send pings to peer with this period. Must be less than pongWait.
    //pingPeriod = 6 * time.Second
    pingPeriod = 50 * time.Second
)

//////////////////////////////////////////////////////////////////////
// readPump() just handles pings for now

func (device *DeviceConnection) readPump() {

    device.socket.SetReadDeadline(time.Now().Add(pongWait))
    device.socket.SetPongHandler(func(string) error {
        then := time.Now().Add(pongWait)
        device.socket.SetReadDeadline(then)
        return nil
    })
    for {
        Info.Printf("readPump for %s starts", device.deviceId)
        _, message, err := device.socket.ReadMessage()
        if err != nil {
            if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
                log.Printf("error: %v", err)
            }
            break
        }
        Verbose.Printf("PING GOT: [%s]! This shouldn't happen, right?", string(message))
    }
    Info.Printf("Device %s readPump closing", device.deviceId)
    device.end()
}

//////////////////////////////////////////////////////////////////////
// handleMessages() forwards Alexa commands to a device

func (device *DeviceConnection) handleMessages() {

    go device.readPump()

    Info.Printf("Device %v starts", device.deviceId)

    ticker := *time.NewTicker(pingPeriod)

    defer func() {
        Info.Printf("Device %s cleanup begins", device.deviceId)
        ticker.Stop()
        device.socket.Close()
        delete(clients, device.deviceId) // RACE?
        device.graveyard <- true
        Info.Printf("Device %s cleanup complete", device.deviceId)
    }()

    for {
        select {

        case alexa_message := <-device.messages:
            //Verbose.Printf("Device %s: %s", device.deviceId, string(alexa_message))
            device.socket.SetWriteDeadline(time.Now().Add(writeWait))
            err := device.socket.WriteMessage(websocket.TextMessage, alexa_message)
            if err != nil {
                Error.Printf("%s", err.Error())
            }
            Info.Printf("Device %s: %s", device.deviceId, string(alexa_message))

        case <-ticker.C:
            device.socket.SetWriteDeadline(time.Now().Add(writeWait))
            if err := device.socket.WriteMessage(websocket.PingMessage, nil); err != nil {
                Error.Printf("Error sending PING: %s", err.Error())
                return
            }

        case <-device.murder:
            Info.Printf("Device %s closing", device.deviceId)
            return
        }
    }
}

//////////////////////////////////////////////////////////////////////
// end() shuts down a DeviceConnection, usually because of a ping
//    timeout because device gone away or device reconnects and
//    we need to tear down the existing connection to make way
//    for the new one

func (device *DeviceConnection) end() {
    Info.Printf("Device %s shutting down", device.deviceId)
    device.murder <- true
    _ = <-device.graveyard
    Info.Printf("Device %s shutdown complete", device.deviceId)
}

//////////////////////////////////////////////////////////////////////

func was_main() {

    Info.Printf("MAIN!")

    upgrader.CheckOrigin = func(r *http.Request) bool {
        return true
    }

    Verbose.Printf("Spawn Alexa listener")

    alexa_port := 5001
    go func() {
        Info.Printf("starting http alexa server on port %v", alexa_port)
        http.ListenAndServe(fmt.Sprintf(":%v", alexa_port), &AlexaMessage{})
    }()

    Verbose.Printf("Device listener")

    device_port := 5000
    Info.Printf("starting http device server on port %v", device_port)
    http.ListenAndServe(fmt.Sprintf(":%v", device_port), &DeviceConnection{})
}
