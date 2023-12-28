-- table names are singular
-- primary key ids are table_id
-- foreign keys are table_othertable_id

DROP DATABASE salt_sensor_test;

CREATE DATABASE salt_sensor_test;

USE salt_sensor_test;

-- sensor (e.g. V1, V2, some other kind of sensor)

CREATE TABLE sensor (
        sensor_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        sensor_name VARCHAR(64) NOT NULL,
        sensor_min_vbat SMALLINT NOT NULL,
        sensor_max_vbat SMALLINT NOT NULL,
        sensor_min_distance SMALLINT NOT NULL,
        sensor_max_distance SMALLINT NOT NULL,
        PRIMARY KEY(sensor_id));

-- device (i.e. an instance of a sensor)

CREATE TABLE device (
        device_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        device_sensor_id INT UNSIGNED NOT NULL,
        device_address VARCHAR(12) UNIQUE NOT NULL,
        device_name VARCHAR(64) UNIQUE,
        device_sleep_count SMALLINT DEFAULT 304,
        PRIMARY KEY(device_id),
        FOREIGN KEY(device_sensor_id) REFERENCES sensor(sensor_id));

-- account (just an email for now)

CREATE TABLE account (
        account_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        account_email VARCHAR(250),
        PRIMARY KEY(account_id));

-- notification (for warning users about low levels)

CREATE TABLE notification (
        notification_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        notification_device_id INT NOT NULL,
        notification_account_id INT NOT NULL,
        notification_distance_warning_threshold INT NOT NULL DEFAULT 10,
        notification_vbat_warning_threshold INT NOT NULL DEFAULT 750,
        notification_time_warning_threshold INT NOT NULL DEFAULT 24,
        PRIMARY KEY(notification_id),
        FOREIGN KEY(notification_device_id) REFERENCES device(device_id),
        FOREIGN KEY(notification_account_id) REFERENCES account(account_id));

-- reading (an actual reading of distance, vbat, rssi)

CREATE TABLE reading (
        reading_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        reading_device_id INT UNSIGNED NOT NULL,
        reading_vbat SMALLINT UNSIGNED NOT NULL,
        reading_distance SMALLINT UNSIGNED NOT NULL,
        reading_flags SMALLINT UNSIGNED NOT NULL,
        reading_rssi TINYINT SIGNED NOT NULL,
        reading_timestamp DATETIME NOT NULL,
        PRIMARY KEY(reading_id),
        FOREIGN KEY(reading_device_id) REFERENCES device(device_id));

-- device_view (select devices with sensor details incorporated)

CREATE VIEW device_view AS
        SELECT * FROM device
        INNER JOIN sensor
        WHERE device_sensor_id = sensor_id;

-- readings_index

CREATE INDEX readings_index
ON readings (device_id, reading_timestamp);
