DROP DATABASE salt_sensor_test;

CREATE DATABASE salt_sensor_test;

USE salt_sensor_test;

-- user account

CREATE TABLE accounts (
        account_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        account_email VARCHAR(250),
        PRIMARY KEY(account_id));

-- for warning users about low levels etc

CREATE TABLE notifications (
        notification_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        device_id INT NOT NULL,
        account_id INT NOT NULL,
        PRIMARY KEY(notification_id));

-- devices

CREATE TABLE devices (
        device_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        device_address VARCHAR(12) UNIQUE NOT NULL,
        device_name VARCHAR(64) UNIQUE,
        device_account_owner INT,
        device_warning_threshold SMALLINT UNSIGNED DEFAULT 10,
        vbat_warning_threshold SMALLINT UNSIGNED DEFAULT 750,
        time_warning_threshold SMALLINT UNSIGNED DEFAULT 24,
        sleep_count SMALLINT DEFAULT 304,
        PRIMARY KEY(device_id));

-- readings

CREATE TABLE readings (
        reading_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        device_id INT UNSIGNED NOT NULL,
        reading_vbat SMALLINT UNSIGNED NOT NULL,
        reading_distance SMALLINT UNSIGNED NOT NULL,
        reading_flags SMALLINT UNSIGNED NOT NULL,
        reading_rssi TINYINT SIGNED NOT NULL,
        reading_timestamp DATETIME NOT NULL,
        PRIMARY KEY(reading_id));

CREATE INDEX readings_index 
ON readings (device_id, reading_timestamp);

-- DEBUG

INSERT INTO accounts (account_email) VALUES ('charlie@skilbeck.com');   -- 1
INSERT INTO accounts (account_email) VALUES ('rachel@skilbeck.com');    -- 2

INSERT INTO devices (
                device_address,
                device_name,
                device_account_owner,
                device_warning_threshold,
                vbat_warning_threshold,
                time_warning_threshold,
                sleep_count)
        VALUES (
                'AABBCCDDEEFF',
                'device1',
                1,
                10,
                750,
                24,
                304);

INSERT INTO readings (
                device_id,
                reading_vbat,
                reading_distance,
                reading_flags,
                reading_rssi,
                reading_timestamp)
        VALUES (
                1,
                740,
                230,
                6,
                -50,
                '2023-09-07 14:01:57');

INSERT INTO notifications (
                device_id,
                account_id)
        VALUES (
                1,
                1);

INSERT INTO notifications (
                device_id,
                account_id)
        VALUES (
                1,
                2);

