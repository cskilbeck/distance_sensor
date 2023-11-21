DROP DATABASE salt_sensor;

CREATE DATABASE salt_sensor;

USE salt_sensor;

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
        notification_distance_warning_threshold INT NOT NULL DEFAULT 10,
        notification_vbat_warning_threshold INT NOT NULL DEFAULT 750,
        notification_time_warning_threshold INT NOT NULL DEFAULT 24,
        PRIMARY KEY(notification_id));

-- sensors

CREATE TABLE sensors (
        sensor_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        sensor_name VARCHAR(64) NOT NULL,
        sensor_min_vbat SMALLINT NOT NULL,
        sensor_max_vbat SMALLINT NOT NULL,
        sensor_min_distance SMALLINT NOT NULL,
        sensor_max_distance SMALLINT NOT NULL,
        PRIMARY KEY(sensor_id));

-- devices

CREATE TABLE devices (
        device_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        sensor_id INT UNSIGNED NOT NULL,
        device_address VARCHAR(12) UNIQUE NOT NULL,
        device_name VARCHAR(64) UNIQUE,
        sleep_count SMALLINT DEFAULT 21600,
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
