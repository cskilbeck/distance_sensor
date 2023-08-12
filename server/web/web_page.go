package main

import (
	"fmt"
	// "os"
	// "text/template"
)

type Device struct {
	device_id      int64
	device_address string
	device_name    string
}

type Reading struct {
	reading_id        int64
	device_id         int64
	reading_vbat      int16
	reading_distance  int16
	reading_flags     int16
	reading_timestamp int64
}

func main() {
	fmt.Printf("Hello world\n")
}
