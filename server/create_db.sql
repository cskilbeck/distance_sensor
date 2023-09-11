DROP DATABASE salt_sensor;

CREATE DATABASE salt_sensor;

USE salt_sensor;

CREATE TABLE devices (
        device_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        device_address VARCHAR(12) UNIQUE NOT NULL,
        device_name VARCHAR(64) UNIQUE,
        device_email VARCHAR(250),
        device_warning_threshold SMALLINT UNSIGNED DEFAULT 10,
        vbat_warning_threshold SMALLINT UNSIGNED DEFAULT 750,
        time_warning_threshold SMALLINT UNSIGNED DEFAULT 24,
        sleep_count SMALLINT DEFAULT 304,
        PRIMARY KEY(device_id));

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
