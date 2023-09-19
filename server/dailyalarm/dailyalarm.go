package dailyalarm

import (
	"fmt"
	"time"
)

//////////////////////////////////////////////////////////////////////

type AlarmNotify int

const (
	AlarmQuit AlarmNotify = iota
	AlarmFire
)

//////////////////////////////////////////////////////////////////////
// Set up a daily alarm which calls `callback()` at hh:mm every day
// Returns a channel of AlarmNotify, send it
//
//		`dailyalarm.AlarmQuit` to quit the daily alarm
//		`dailyalarm.AlarmFire` to manually fire it

func Set(hh, mm int, callback func()) (chan AlarmNotify, error) {

	if hh < 0 || hh > 23 || mm < 0 || mm > 59 {
		return nil, fmt.Errorf("time specified (%02d:%02d) is out of range", hh, mm)
	}

	alarm := make(chan AlarmNotify)

	go func() {

		for {

			now := time.Now().UTC()

			day := now.Day()

			if now.Hour() >= hh {
				day += 1
			}

			after := time.Until(time.Date(now.Year(), now.Month(), day, hh, mm, 0, 0, time.UTC))

			time.AfterFunc(after, func() {
				alarm <- AlarmFire
			})

			if <-alarm == AlarmQuit {
				return
			}
			callback()
		}
	}()

	return alarm, nil
}
