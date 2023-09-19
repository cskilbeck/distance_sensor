package logger

import (
	"fmt"
	"io"
	"log"
	"os"
	"strings"
)

//////////////////////////////////////////////////////////////////////
// Logging admin

const (
	LogLevelDebug int = iota
	LogLevelVerbose
	LogLevelInfo
	LogLevelWarning
	LogLevelError
	LogLevelFatal
	numLogLevels
)

var LogLevelNames = [numLogLevels]string{
	"debug",
	"verbose",
	"info",
	"warning",
	"error",
	"fatal",
}

var LogLevel = LogLevelInfo // default log level is info

var Debug *log.Logger
var Verbose *log.Logger
var Info *log.Logger
var Warning *log.Logger
var Error *log.Logger
var Fatal *log.Logger

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

func Init(log_level_name string) (e error) {

	log_writers := [numLogLevels]io.Writer{
		os.Stdout,
		os.Stdout,
		os.Stdout,
		os.Stderr,
		os.Stderr,
		os.Stderr,
	}

	longest_name := longestStringLength(LogLevelNames[:])

	// find the log level by name

	found := false

	if len(log_level_name) != 0 {

		for i, name := range LogLevelNames {

			if strings.EqualFold(log_level_name, name) {

				LogLevel = i
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

	for i := 0; i < LogLevel; i++ {

		logs[i] = log.New(io.Discard, "", 0)
	}

	// stdout or stderr for enabled levels

	for i := LogLevel; i < numLogLevels; i++ {

		name := LogLevelNames[i]
		padded := name + strings.Repeat(" ", longest_name-len(name)) + ":"
		logs[i] = log.New(log_writers[i], padded, log.Ldate|log.Ltime|log.Lmicroseconds)
	}

	// setup for use

	Debug = logs[LogLevelDebug]
	Verbose = logs[LogLevelVerbose]
	Info = logs[LogLevelInfo]
	Warning = logs[LogLevelWarning]
	Error = logs[LogLevelError]
	Fatal = logs[LogLevelFatal]

	return nil
}
