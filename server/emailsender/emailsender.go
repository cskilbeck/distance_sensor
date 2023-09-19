package emailsender

import (
	"bytes"
	"fmt"
	"mime/quotedprintable"
	"net/smtp"
	"strings"
)

//////////////////////////////////////////////////////////////////////
// Send an email
//
// parameter	eg
//
// server		"smtp.gmail.com"
//
// port			587
//
// user			"myaccount@gmail.com"
//
// password		"mypassword"
//
// from			"Tommy"
//
// subject		"About that thing"
//
// message		"The email message" or
//				"<h3>Hello</h3><p><i>World!</i>"
//
// contentType	"text/plain" or
//				"text/html"
//
// recipients	"someone@outlook.com" or
//				"someone@outlook.com, another@aol.com" or
//				"someone@outlook.com", "another@aol.com" or
//				[]string{"someone@outlook.com", "another@outlook.com"} or
//				the_recipients_slice

func SendEmail(server string, port int, user, password, from, subject, message, contentType string, recipients ...string) error {

	smtpTemplate := []string{
		"From: %s",
		"To: %s",
		"Subject: %s",
		"MIME-Version: 1.0",
		"Content-Type: %s; charset=\"utf-8\"",
		"Content-Transfer-Encoding: quoted-printable",
		"Content-Disposition: inline",
		"",
		"%s\r\n",
	}

	var body bytes.Buffer
	q := quotedprintable.NewWriter(&body)
	_, err := q.Write([]byte(message))
	q.Close()
	if err != nil {
		return err
	}

	template := strings.Join(smtpTemplate, "\r\n")
	rcp := strings.Join(recipients, ",")
	name := fmt.Sprintf("%s <%s>", from, user)
	lines := fmt.Sprintf(template, name, rcp, subject, contentType, body.String())
	auth := smtp.PlainAuth("", user, password, server)
	url := fmt.Sprintf("%s:%d", server, port)
	return smtp.SendMail(url, auth, user, recipients, []byte(lines))
}
